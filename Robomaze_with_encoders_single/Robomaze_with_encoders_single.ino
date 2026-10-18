// Robomaze_with_encoders - ESP32 DevKit V1 left-wall follower with TCS34725 floor-colour handling (black hole, silver checkpoint, blue wait, red danger zone).
// Sensors: 2x HC-SR04 (front, left), MPU6050 gyro (Z axis). No encoders are assumed.
// Logic lives in nav_core.h (host-tested); this file is the hardware layer.
#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <Preferences.h>
// Single-file build: config.h, nav_core.h and hw_types.h are merged below.

// ======================================== config.h ========================================
// config.h (COLOUR + ENCODERS version) - every tunable value for the 2x L298N (four independent motors) wall-following robot.
// Pin numbers here are a PROPOSED wiring (no wiring documentation existed). Wire the robot
// to match this table, or edit the pins here - the static_asserts at the bottom reject
// GPIOs that are unsafe for their role.
#include <stdint.h>

#define ROBOT_L298N_COUNT 2
#define BENCH_ENCODER_MONITOR 0  // 1 = at boot print encoder counts for 20 s (turn wheels by hand), then continue
#define BENCH_MOTOR_TEST 0   // 1 = at boot spin each motor 1 s (wheels OFF the ground), then halt

namespace cfg {

// ------------------------------- Pins (ESP32 DevKit V1) -------------------------------
constexpr uint8_t kPinSda = 21;
constexpr uint8_t kPinScl = 22;

constexpr uint8_t kPinFrontTrig = 5;    // strapping pin, but output-only use as HC-SR04 trigger is safe
constexpr uint8_t kPinFrontEcho = 34;   // input-only; 5V echo -> 1k/2k divider
constexpr uint8_t kPinLeftTrig  = 14;
constexpr uint8_t kPinLeftEcho  = 35;   // input-only; 5V echo -> 1k/2k divider

// L298N #1 (FRONT axle): channel A = front-left motor, channel B = front-right motor
constexpr uint8_t kPinD1EnA = 25, kPinD1In1 = 26, kPinD1In2 = 27;
constexpr uint8_t kPinD1EnB = 33, kPinD1In3 = 32, kPinD1In4 = 13;
// L298N #2 (REAR axle): channel A = rear-left motor, channel B = rear-right motor
constexpr uint8_t kPinD2EnA = 16, kPinD2In1 = 17, kPinD2In2 = 18;
constexpr uint8_t kPinD2EnB = 19, kPinD2In3 = 23, kPinD2In4 = 4;

// Wheel encoders: ONE single-channel (pulse) encoder per side. GPIO36/39 are input-only with no internal
// pull-up: use push-pull Hall modules (3.3 V logic) or add a 10k pull-up to 3V3. 5 V outputs need a divider.
constexpr uint8_t kPinEncLeft = 36;
constexpr uint8_t kPinEncRight = 39;

// Per-motor direction inversion (set true if a motor is mounted/wired the opposite way).
constexpr bool kInvertFrontLeft = false, kInvertFrontRight = false;
constexpr bool kInvertRearLeft = false, kInvertRearRight = false;

// ------------------------------- Sensor timing / limits -------------------------------
constexpr uint32_t kSensorSlotMs = 30;      // one trigger every 30 ms, alternating front/left
constexpr float kMaxRangeCm = 200.0f;       // farther echoes are treated as "no echo"
constexpr float kMinRangeCm = 2.0f;         // below this an echo is implausible
constexpr float kSoundCmPerUs = 0.0343f;    // 20 C
constexpr uint32_t kEchoTimeoutUs = (uint32_t)(2.0f * kMaxRangeCm / kSoundCmPerUs) + 3000u;
constexpr float kLeftEmaAlpha = 0.5f;       // smoothing applied after the 3-sample median
constexpr uint32_t kFrontStaleMs = 300;
constexpr uint32_t kLeftStaleMs = 300;
constexpr uint16_t kImplausibleFaultCount = 8;   // consecutive implausible echoes -> SENSOR_FAULT
constexpr uint16_t kFrontNoEchoFaultCount = 45;  // ~2.7 s of front "no echo" -> SENSOR_FAULT

// ------------------------------- Distances -------------------------------
constexpr float kWallTargetCm = 9.0f;       // sensor-to-wall distance
constexpr float kFrontStopCm = 7.0f;        // stop at or inside this
constexpr float kFrontResumeCm = 9.0f;      // false-alarm release (hysteresis)
constexpr float kFrontSlowZoneCm = 30.0f;   // ramp speed down between here and kFrontStopCm
constexpr float kWallTrackMaxCm = 30.0f;    // beyond this the wall is not tracked
constexpr float kOpenEnterCm = 25.0f;       // left reading above this counts as "open"
constexpr float kOpenExitCm = 18.0f;        // left reading below this counts as "wall"

// ------------------------------- Opening detection -------------------------------
constexpr uint8_t kMinWallSamples = 5;       // wall must be seen this many samples before an opening counts
constexpr uint8_t kOpenConfirmSamples = 4;   // consecutive open samples (~240 ms)
constexpr uint32_t kOpenConfirmMinMs = 200;
constexpr uint32_t kConfirmTimeoutMs = 1200;
constexpr uint32_t kOpenBlockMs = 800;       // after an aborted confirmation
constexpr uint32_t kTurnCooldownMs = 1500;   // after any turn, ignore openings
constexpr uint32_t kOpeningAdvanceMs = 250;  // creep forward so the pivot lines up with the opening (TUNE)
constexpr uint32_t kNoWallTimeoutMs = 4000;  // no wall and no turn for this long -> SAFE_STOP

// ------------------------------- Speeds (fraction 0..1 of usable range) -------------------------------
constexpr float kCruiseSpeed = 0.55f;
constexpr float kSlowSpeed = 0.35f;

// ------------------------------- Wall controller (outer loop) -------------------------------
constexpr float kWallDeadbandCm = 0.7f;
constexpr float kWallKp = 2.0f;             // degrees of heading offset per cm of error
constexpr float kWallKd = 0.6f;             // degrees per (cm/s)
constexpr float kWallDerivAlpha = 0.3f;
constexpr float kMaxWallOffsetDeg = 12.0f;

// ------------------------------- Heading controller (inner loop) -------------------------------
constexpr float kHeadKp = 0.035f;           // steer per degree
constexpr float kHeadKd = 0.004f;           // steer per deg/s
constexpr float kMaxSteer = 0.45f;
constexpr int8_t kGyroZSign = +1;           // +1: positive rate = counter-clockwise (left) seen from above

// ------------------------------- Turns -------------------------------
constexpr float kTurnTolDeg = 3.0f;
constexpr float kTurnKp = 0.02f;            // speed per degree remaining
constexpr float kTurnMaxSpeed = 0.55f;
constexpr float kTurnMinSpeed = 0.30f;
constexpr uint32_t kTurnTimeoutMs90 = 5000;
constexpr uint32_t kTurnSettleMs = 200;
constexpr float kTurnSettleRateDps = 6.0f;
constexpr uint8_t kTurnMaxCorrections = 3;
constexpr uint32_t kTurnStallMs = 1200;
constexpr float kTurnStallMinDeg = 4.0f;
constexpr float kTurnWrongWayDeg = 15.0f;

// ------------------------------- Obstacle handling -------------------------------
constexpr uint8_t kAssessSamples = 2;
constexpr uint32_t kStopSettleMs = 250;
constexpr uint32_t kAssessTimeoutMs = 1500;
constexpr uint32_t kPostTurnSettleMs = 350;
constexpr uint32_t kSettleTimeoutMs = 1500;
constexpr uint32_t kRecoveryPauseMs = 800;
constexpr uint8_t kMaxTurnsPerEpisode = 3;
constexpr uint8_t kMaxRecoveries = 2;
constexpr uint32_t kEpisodeResetMs = 1500;

// ------------------------------- Startup -------------------------------
constexpr uint32_t kStartDelayMs = 3000;
constexpr uint8_t kSensorCheckSamples = 5;
constexpr uint32_t kSensorCheckTimeoutMs = 3000;

// ------------------------------- MPU6050 -------------------------------
constexpr uint8_t kImuAddr = 0x68;
constexpr uint32_t kI2cHz = 400000;
constexpr uint32_t kImuPollUs = 5000;       // 200 Hz
constexpr uint32_t kImuStaleMs = 60;
constexpr float kGyroLsbPerDps = 65.5f;     // +-500 dps range
constexpr float kGyroDeadbandDps = 0.12f;
constexpr float kImuMaxRateDps = 480.0f;    // near saturation -> treated as implausible
constexpr uint16_t kCalSamples = 300;
constexpr uint32_t kCalIntervalMs = 5;
constexpr float kCalMaxStdDps = 0.6f;       // more noise than this = robot was moving
constexpr float kCalMaxBiasDps = 25.0f;
constexpr uint8_t kCalRetries = 3;
constexpr uint8_t kImuFailLimit = 5;
constexpr uint16_t kImuFrozenLimit = 250;

// ------------------------------- Motors -------------------------------
constexpr uint32_t kPwmHz = 5000;
constexpr uint8_t kPwmBits = 8;
constexpr uint8_t kPwmMin = 110;            // minimum duty that reliably starts the motors (TUNE)
constexpr uint8_t kPwmMax = 230;            // maximum safe duty (TUNE; L298N drops ~2 V)
constexpr float kCmdDeadband = 0.03f;
constexpr float kAccelPerSec = 2.0f;
constexpr float kDecelPerSec = 5.0f;
constexpr uint32_t kBrakeMs = 100;
constexpr uint32_t kReverseDwellMs = 20;

// ------------------------------- TCS34725 floor colour sensor -------------------------------
// I2C (shares SDA/SCL with the MPU6050, address 0x29). Mount 3-5 mm above the floor, looking down,
// at the FRONT of the robot, shielded from ambient light. Tie its LED pin to 3V3 (always on).
constexpr bool kColorRequired = true;        // true: no colour sensor / stale colour = SENSOR_FAULT (holes are undetectable)
constexpr uint8_t kColorAddr = 0x29;
constexpr uint8_t kColorAtime = 0xF6;        // 24 ms integration
constexpr uint8_t kColorGain = 0x01;         // 4x
constexpr uint32_t kColorPollMs = 30;
constexpr uint32_t kColorStaleMs = 300;
constexpr uint8_t kColorConfirmSamples = 3;  // consecutive identical classes before an event fires
constexpr uint8_t kColorRearmSamples = 3;    // non-matching samples needed before the same colour can fire again
constexpr float kColorMaxDist = 0.10f;       // feature-space radius around a reference; farther = normal floor
constexpr uint8_t kColorFailLimit = 5;
// Default reference readings {R, G, B, Clear}. THESE ARE PLACEHOLDERS: calibrate on your real tiles
// (see README, colour calibration mode). Calibrated values are stored in flash and override these.
constexpr uint16_t kRefBlack[4]  = {200, 200, 200, 700};
constexpr uint16_t kRefSilver[4] = {9000, 9000, 9000, 30000};
constexpr uint16_t kRefBlue[4]   = {1500, 3500, 6000, 11000};
constexpr uint16_t kRefRed[4]    = {6000, 1500, 1300, 9500};
constexpr uint16_t kRefFloor[4]  = {14000, 14000, 14000, 45000};   // ordinary floor (so bright white is not mistaken for silver)

// ------------------------------- Floor-colour behaviour -------------------------------
constexpr uint32_t kHazardStopMs = 150;      // stop first ...
constexpr uint32_t kBackoffMs = 300;         // ... then reverse this long (blind: no rear sensor)
constexpr uint8_t kMaxHazardsPerEpisode = 4;
constexpr uint32_t kBlueAdvanceMs = 300;     // creep onto the tile before waiting (TUNE)
constexpr uint32_t kBlueWaitMs = 5000;       // stay on each blue tile this long, consecutively
constexpr uint32_t kBlueDropoutMs = 400;     // blue lost this long during the wait restarts the timer
constexpr uint8_t kBlueMaxResets = 3;        // then give up on that tile and drive on
constexpr uint32_t kBlueRearmMaxMs = 3000;
constexpr bool kBlueWaitEveryVisit = true;   // false: wait only the first time a tile is visited
constexpr uint8_t kMaxBlueTiles = 16;
constexpr float kTileCm = 30.0f;             // tile size, used to tell blue tiles apart (dead reckoning)
constexpr float kCmPerSecFullSpeed = 40.0f;  // open-loop speed at command 1.0 (CALIBRATE on the floor)
constexpr float kDangerSpeedScale = 1.0f;    // speed multiplier after the red (Dangerous Zone) entrance

// ------------------------------- Wheel encoders -------------------------------
constexpr bool kEncodersEnabled = true;      // false: fall back to time-based moves and speed-model dead reckoning
constexpr float kEncPulsesPerRev = 20.0f;    // pulses per WHEEL revolution, measured after the gearbox (CALIBRATE)
constexpr float kWheelDiameterCm = 6.5f;     // (MEASURE)
constexpr float kCmPerTick = 3.14159265f * kWheelDiameterCm / kEncPulsesPerRev;
constexpr uint32_t kEncMinPulseUs = 200;     // ISR ignores pulses closer than this (noise / bounce)
constexpr float kOpeningAdvanceCm = 3.5f;    // distance-based replacement for kOpeningAdvanceMs (TUNE)
constexpr float kBlueAdvanceCm = 4.0f;       // distance-based replacement for kBlueAdvanceMs
constexpr float kBackoffCm = 4.0f;           // distance-based replacement for kBackoffMs
constexpr uint32_t kAdvanceTimeoutMs = 2500; // an encoder-based move that takes longer is abandoned
constexpr uint32_t kStallWindowMs = 1500;    // wheel commanded but not turning over this window = fault
constexpr float kStallMinCmd = 0.25f;        // average |command| that must produce motion
constexpr float kStallMinCm = 1.0f;          // minimum travel per window at that command

// ------------------------------- Diagnostics -------------------------------
constexpr uint32_t kDiagPeriodMs = 250;

// ------------------------------- Compile-time sanity -------------------------------
constexpr bool outputOk(uint8_t p) {
  return p == 4 || p == 5 || p == 13 || p == 14 || (p >= 16 && p <= 19) || p == 21 || p == 22 ||
         p == 23 || (p >= 25 && p <= 27) || p == 32 || p == 33;
}
constexpr bool inputOnly(uint8_t p) { return p == 34 || p == 35 || p == 36 || p == 39; }

static_assert(outputOk(kPinFrontTrig) && outputOk(kPinLeftTrig), "trigger pins must be output-capable");
static_assert(inputOnly(kPinFrontEcho) && inputOnly(kPinLeftEcho), "echo pins are input-only by design");
static_assert(outputOk(kPinD1EnA) && outputOk(kPinD1In1) && outputOk(kPinD1In2) && outputOk(kPinD1EnB) &&
                  outputOk(kPinD1In3) && outputOk(kPinD1In4),
              "L298N #1 pins must be output-capable, non-flash, non-strapping");
static_assert(outputOk(kPinD2EnA) && outputOk(kPinD2In1) && outputOk(kPinD2In2) && outputOk(kPinD2EnB) &&
                  outputOk(kPinD2In3) && outputOk(kPinD2In4),
              "L298N #2 pins must be output-capable, non-flash, non-strapping");
static_assert(kFrontStopCm < kFrontResumeCm && kFrontResumeCm < kFrontSlowZoneCm, "front thresholds ordered");
static_assert(kWallTargetCm < kOpenExitCm && kOpenExitCm < kOpenEnterCm && kOpenEnterCm <= kWallTrackMaxCm,
              "wall/open thresholds must be ordered for hysteresis");
static_assert(kColorConfirmSamples >= 1 && kBlueWaitMs > kBlueDropoutMs, "colour timing");
static_assert(inputOnly(kPinEncLeft) && inputOnly(kPinEncRight) && kPinEncLeft != kPinEncRight, "encoder pins");
static_assert(kPwmMin < kPwmMax, "PWM range");
static_assert(kTurnMinSpeed <= kTurnMaxSpeed && kSlowSpeed <= kCruiseSpeed, "speed ordering");
static_assert(kSensorSlotMs * 1000u > kEchoTimeoutUs, "sensor slot must outlast the echo timeout (cross-talk)");

}  // namespace cfg

// ======================================== nav_core.h ========================================
// nav_core.h (COLOUR + ENCODERS version) - hardware-independent navigation logic (no Arduino headers).
// Compiled both into the ESP32 sketch and into the host unit tests.
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
namespace nav {

using namespace cfg;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }
constexpr float kDegToRad = 0.01745329252f;

// ============================== Range filtering ==============================
enum class RangeKind : uint8_t { Valid, NoEcho, Implausible };

class RangeFilter {
 public:
  void push(RangeKind kind, float cm, uint32_t nowMs) {
    ++seq_;
    lastKind_ = kind;
    lastMs_ = nowMs;
    if (kind == RangeKind::Valid) {
      ++validRun_; noEchoRun_ = 0; implRun_ = 0;
      raw_ = cm;
      lastValidMs_ = nowMs;
      hasValue_ = true;
      ring_[ringPos_] = cm;
      ringPos_ = (ringPos_ + 1) % 3;
      if (ringCount_ < 3) ++ringCount_;
      const float med = median();
      filtered_ = (ringCount_ == 1) ? med : filtered_ + kLeftEmaAlpha * (med - filtered_);
    } else if (kind == RangeKind::NoEcho) {
      ++noEchoRun_; validRun_ = 0; implRun_ = 0;
    } else {
      ++implRun_; validRun_ = 0; noEchoRun_ = 0;
    }
  }
  uint32_t seq() const { return seq_; }
  RangeKind lastKind() const { return lastKind_; }
  uint32_t lastMs() const { return lastMs_; }
  float rawCm() const { return raw_; }             // last valid raw value
  float filteredCm() const { return filtered_; }   // median-of-3 then EMA
  bool recentValid(uint32_t now, uint32_t staleMs) const { return hasValue_ && (now - lastValidMs_) <= staleMs; }
  uint16_t validRun() const { return validRun_; }
  uint16_t noEchoRun() const { return noEchoRun_; }
  uint16_t implausibleRun() const { return implRun_; }

 private:
  float median() const {
    if (ringCount_ == 1) return ring_[0];
    if (ringCount_ == 2) return 0.5f * (ring_[0] + ring_[1]);
    const float a = ring_[0], b = ring_[1], c = ring_[2];
    const float mx = a > b ? (a > c ? a : c) : (b > c ? b : c);
    const float mn = a < b ? (a < c ? a : c) : (b < c ? b : c);
    return a + b + c - mx - mn;
  }
  uint32_t seq_ = 0, lastMs_ = 0, lastValidMs_ = 0;
  RangeKind lastKind_ = RangeKind::NoEcho;
  float raw_ = 0, filtered_ = 0, ring_[3] = {0, 0, 0};
  uint8_t ringPos_ = 0, ringCount_ = 0;
  bool hasValue_ = false;
  uint16_t validRun_ = 0, noEchoRun_ = 0, implRun_ = 0;
};

// ============================== Wall controller (outer loop) ==============================
// Output = heading OFFSET (deg, + = toward the left wall) added to the heading reference.
// The heading controller then tracks (reference + offset), so the two loops cascade instead of fighting.
class WallController {
 public:
  void reset() { hasPrev_ = false; deriv_ = 0; offset_ = 0; }
  float offsetDeg() const { return offset_; }
  float errorCm() const { return err_; }
  void decay() { offset_ *= 0.9f; }
  // distCm must already be perpendicular-corrected. Call once per fresh left sample.
  float update(float distCm, uint32_t nowMs) {
    err_ = distCm - kWallTargetCm;                       // + = too far from wall -> steer left
    float eff = 0.0f;
    if (err_ > kWallDeadbandCm) eff = err_ - kWallDeadbandCm;
    else if (err_ < -kWallDeadbandCm) eff = err_ + kWallDeadbandCm;
    if (hasPrev_ && nowMs > prevMs_) {
      const float dt = (nowMs - prevMs_) * 0.001f;
      deriv_ += kWallDerivAlpha * (((err_ - prevErr_) / dt) - deriv_);
    }
    hasPrev_ = true; prevErr_ = err_; prevMs_ = nowMs;
    offset_ = clampf(kWallKp * eff + kWallKd * deriv_, -kMaxWallOffsetDeg, kMaxWallOffsetDeg);
    return offset_;
  }

 private:
  bool hasPrev_ = false;
  float prevErr_ = 0, deriv_ = 0, offset_ = 0, err_ = 0;
  uint32_t prevMs_ = 0;
};

// ============================== Heading controller (inner loop) ==============================
inline float headingSteer(float targetDeg, float headingDeg, float rateDps) {
  return clampf(kHeadKp * (targetDeg - headingDeg) - kHeadKd * rateDps, -kMaxSteer, kMaxSteer);
}

struct WheelCmd { float left, right; };

// steer > 0 turns left (left side slower). Forward only, never reverses while following.
inline WheelCmd mixDrive(float base, float steer) {
  float l = base * (1.0f - steer), r = base * (1.0f + steer);
  const float m = l > r ? l : r;
  if (m > 1.0f) { l /= m; r /= m; }
  return {l < 0 ? 0 : l, r < 0 ? 0 : r};
}

// ============================== Turn controller ==============================
enum class TurnStatus : uint8_t { Idle, Rotating, Settling, Done, Failed };
enum class TurnFail : uint8_t { None, Timeout, Stall, WrongWay, ImuRate, Overshoot };

class TurnController {
 public:
  // targetHeadingDeg is absolute (same frame as the integrated heading). nominalDeg only scales the timeout.
  void begin(float targetHeadingDeg, float startHeadingDeg, float nominalDeg, uint32_t nowMs) {
    target_ = targetHeadingDeg; start_ = startHeadingDeg; nominal_ = nominalDeg;
    dir_ = signf(target_ - start_);
    beginMs_ = nowMs; stallRefHeading_ = start_; stallRefMs_ = nowMs;
    corrections_ = 0; status_ = TurnStatus::Rotating; fail_ = TurnFail::None;
    cmdL_ = cmdR_ = 0; rem_ = target_ - start_; heading_ = start_;
  }
  TurnStatus update(float headingDeg, float rateDps, uint32_t nowMs) {
    heading_ = headingDeg;
    rem_ = target_ - headingDeg;
    if (status_ == TurnStatus::Done || status_ == TurnStatus::Failed || status_ == TurnStatus::Idle) {
      cmdL_ = cmdR_ = 0;
      return status_;
    }
    const float scale = fabsf(nominal_) / 90.0f;
    const uint32_t timeoutMs = (uint32_t)(kTurnTimeoutMs90 * (scale < 1.0f ? 1.0f : scale));
    if (fabsf(rateDps) > kImuMaxRateDps) return failWith(TurnFail::ImuRate);
    if (nowMs - beginMs_ > timeoutMs) return failWith(TurnFail::Timeout);

    if (status_ == TurnStatus::Rotating) {
      if (corrections_ == 0 && (headingDeg - start_) * dir_ < -kTurnWrongWayDeg) return failWith(TurnFail::WrongWay);
      if (fabsf(headingDeg - stallRefHeading_) >= kTurnStallMinDeg) {
        stallRefHeading_ = headingDeg; stallRefMs_ = nowMs;
      } else if (nowMs - stallRefMs_ > kTurnStallMs) {
        return failWith(TurnFail::Stall);
      }
      if (fabsf(rem_) <= kTurnTolDeg) {
        status_ = TurnStatus::Settling; settleMs_ = nowMs; cmdL_ = cmdR_ = 0;
      } else {
        const float sp = clampf(kTurnKp * fabsf(rem_), kTurnMinSpeed, kTurnMaxSpeed);
        const float s = signf(rem_);
        cmdL_ = -s * sp; cmdR_ = s * sp;                 // spin in place; + = counter-clockwise (left)
      }
    } else {  // Settling
      cmdL_ = cmdR_ = 0;
      if (nowMs - settleMs_ >= kTurnSettleMs && fabsf(rateDps) <= kTurnSettleRateDps) {
        if (fabsf(rem_) <= kTurnTolDeg) {
          status_ = TurnStatus::Done;
        } else if (corrections_ < kTurnMaxCorrections) {
          ++corrections_; status_ = TurnStatus::Rotating;
          stallRefHeading_ = headingDeg; stallRefMs_ = nowMs;
        } else {
          return failWith(TurnFail::Overshoot);
        }
      }
    }
    return status_;
  }
  TurnStatus status() const { return status_; }
  TurnFail failReason() const { return fail_; }
  float cmdLeft() const { return cmdL_; }
  float cmdRight() const { return cmdR_; }
  bool brake() const { return status_ == TurnStatus::Settling; }
  float targetDeg() const { return target_; }
  float measuredDeg() const { return heading_ - start_; }
  float remainingDeg() const { return rem_; }
  uint8_t corrections() const { return corrections_; }

 private:
  TurnStatus failWith(TurnFail f) { fail_ = f; status_ = TurnStatus::Failed; cmdL_ = cmdR_ = 0; return status_; }
  TurnStatus status_ = TurnStatus::Idle;
  TurnFail fail_ = TurnFail::None;
  float target_ = 0, start_ = 0, nominal_ = 90, dir_ = 1, rem_ = 0, heading_ = 0;
  float stallRefHeading_ = 0, cmdL_ = 0, cmdR_ = 0;
  uint32_t beginMs_ = 0, stallRefMs_ = 0, settleMs_ = 0;
  uint8_t corrections_ = 0;
};

// ============================== Pure decisions (unit-tested) ==============================
enum class LeftState : uint8_t { Open, Blocked, Unknown };
enum class AvoidAction : uint8_t { Resume, TurnLeft, TurnRight, Wait };

inline bool frontBlockedSample(RangeKind k, float cm) { return k == RangeKind::Valid && cm <= kFrontStopCm; }

// Open sample: nothing echoed, or the wall is farther than the enter threshold.
inline bool leftOpenSample(RangeKind k, float cm) {
  return k == RangeKind::NoEcho || (k == RangeKind::Valid && cm > kOpenEnterCm);
}
inline bool leftWallSample(RangeKind k, float cm) { return k == RangeKind::Valid && cm < kOpenExitCm; }

inline AvoidAction decideAvoidance(bool frontBlocked, LeftState left) {
  if (!frontBlocked) return AvoidAction::Resume;
  if (left == LeftState::Open) return AvoidAction::TurnLeft;
  if (left == LeftState::Blocked) return AvoidAction::TurnRight;
  return AvoidAction::Wait;
}

// ============================== Floor colour ==============================
enum class ColorClass : uint8_t { Other, Black, Silver, Blue, Red };

inline const char* colorName(ColorClass c) {
  switch (c) {
    case ColorClass::Black: return "BLACK";
    case ColorClass::Silver: return "SILVER";
    case ColorClass::Blue: return "BLUE";
    case ColorClass::Red: return "RED";
    case ColorClass::Other: return "other";
  }
  return "?";
}

struct ColorSample { uint16_t r, g, b, c; };
struct ColorRef { float f[4]; };

// Feature vector: chromaticity (r,g,b over their sum) plus log-scaled clear-channel brightness.
inline void colorFeatures(uint16_t r, uint16_t g, uint16_t b, uint16_t c, float* f) {
  float sum = (float)r + (float)g + (float)b;
  if (sum < 1.0f) sum = 1.0f;
  f[0] = r / sum; f[1] = g / sum; f[2] = b / sum;
  f[3] = log10f((float)c + 1.0f) / 4.8165f;
}
inline ColorRef makeRef(const uint16_t* v) {
  ColorRef ref;
  colorFeatures(v[0], v[1], v[2], v[3], ref.f);
  return ref;
}
struct ColorRefs { ColorRef black, silver, blue, red, floor; };
inline ColorRefs defaultColorRefs() {
  return {makeRef(kRefBlack), makeRef(kRefSilver), makeRef(kRefBlue), makeRef(kRefRed), makeRef(kRefFloor)};
}
inline float featureDist(const float* a, const float* b) {
  float d = 0;
  for (int i = 0; i < 4; ++i) d += (a[i] - b[i]) * (a[i] - b[i]);
  return sqrtf(d);
}
// Nearest reference within kColorMaxDist wins; the floor reference and anything unmatched are ordinary floor.
inline ColorClass classifyColor(const ColorSample& s, const ColorRefs& refs) {
  if (s.c == 0) return ColorClass::Other;
  float f[4];
  colorFeatures(s.r, s.g, s.b, s.c, f);
  const float d[5] = {featureDist(f, refs.black.f), featureDist(f, refs.silver.f), featureDist(f, refs.blue.f),
                      featureDist(f, refs.red.f), featureDist(f, refs.floor.f)};
  const ColorClass cls[5] = {ColorClass::Black, ColorClass::Silver, ColorClass::Blue, ColorClass::Red,
                             ColorClass::Other};
  int best = 0;
  for (int i = 1; i < 5; ++i) if (d[i] < d[best]) best = i;
  return d[best] <= kColorMaxDist ? cls[best] : ColorClass::Other;
}

class ColorFilter {
 public:
  void push(ColorClass c, uint32_t nowMs) {
    ++seq_; lastMs_ = nowMs;
    if (c == stable_) { if (run_ < 65535) ++run_; } else { stable_ = c; run_ = 1; }
  }
  uint32_t seq() const { return seq_; }
  uint32_t lastMs() const { return lastMs_; }
  ColorClass current() const { return stable_; }
  uint16_t run() const { return run_; }

 private:
  uint32_t seq_ = 0, lastMs_ = 0;
  ColorClass stable_ = ColorClass::Other;
  uint16_t run_ = 0;
};

struct Checkpoint { float x, y, headingDeg; uint32_t ms; uint16_t id; };
struct BlueTile { int16_t cx, cy; uint8_t visits, completed; };

// ============================== Navigation state machine ==============================
enum class State : uint8_t {
  INITIALIZING, IMU_CALIBRATION, SENSOR_CHECK, FORWARD_WALL_FOLLOW, LEFT_OPENING_CONFIRMATION,
  LEFT_OPENING_ADVANCE, OBSTACLE_STOP, TURNING_LEFT_90, TURNING_RIGHT_90, TURNING_AROUND_180,
  SETTLE_AFTER_TURN, OBSTACLE_RECOVERY, HAZARD_BACKOFF, BLUE_ADVANCE, BLUE_WAIT, SENSOR_FAULT, SAFE_STOP
};

inline const char* stateName(State s) {
  switch (s) {
    case State::INITIALIZING: return "INITIALIZING";
    case State::IMU_CALIBRATION: return "IMU_CALIBRATION";
    case State::SENSOR_CHECK: return "SENSOR_CHECK";
    case State::FORWARD_WALL_FOLLOW: return "FORWARD_WALL_FOLLOW";
    case State::LEFT_OPENING_CONFIRMATION: return "LEFT_OPENING_CONFIRM";
    case State::LEFT_OPENING_ADVANCE: return "LEFT_OPENING_ADVANCE";
    case State::OBSTACLE_STOP: return "OBSTACLE_STOP";
    case State::TURNING_LEFT_90: return "TURNING_LEFT_90";
    case State::TURNING_RIGHT_90: return "TURNING_RIGHT_90";
    case State::TURNING_AROUND_180: return "TURNING_AROUND_180";
    case State::SETTLE_AFTER_TURN: return "SETTLE_AFTER_TURN";
    case State::OBSTACLE_RECOVERY: return "OBSTACLE_RECOVERY";
    case State::HAZARD_BACKOFF: return "HAZARD_BACKOFF";
    case State::BLUE_ADVANCE: return "BLUE_ADVANCE";
    case State::BLUE_WAIT: return "BLUE_WAIT";
    case State::SENSOR_FAULT: return "SENSOR_FAULT";
    case State::SAFE_STOP: return "SAFE_STOP";
  }
  return "?";
}

enum class ImuCal : uint8_t { Pending, Done, Failed };

struct NavInputs {
  uint32_t nowMs;
  const RangeFilter* front;
  const RangeFilter* left;
  float headingDeg;   // integrated, unwrapped, + = counter-clockwise
  float rateDps;
  bool imuOk;
  ImuCal imuCal;
  const ColorFilter* colour;
  bool colourOk;
  float odoLeftCm, odoRightCm;   // cumulative signed wheel travel (encoders)
};

struct NavOutputs {
  float left, right;   // -1..1
  bool immediate;      // bypass acceleration ramp (turns, stops)
  bool brake;          // apply electrical brake while stopped
  State state;
};

class Navigator {
 public:
  State state() const { return state_; }
  const char* reason() const { return reason_; }
  float headingRefDeg() const { return ref_; }
  float wallErrorCm() const { return wall_.errorCm(); }
  float wallOffsetDeg() const { return wall_.offsetDeg(); }
  const TurnController& turn() const { return turn_; }
  float cmdLeft() const { return last_.left; }
  float cmdRight() const { return last_.right; }
  bool leftOpenRun(uint8_t n) const { return openRun_ >= n; }
  float odoLeftCm() const { return odoL_; }
  float odoRightCm() const { return odoR_; }
  float xCm() const { return x_; }
  float yCm() const { return y_; }
  bool inDangerZone() const { return inDanger_; }
  uint16_t dangerEntrances() const { return dangerEntrances_; }
  uint16_t checkpointCount() const { return cpCount_; }
  const Checkpoint& lastCheckpoint() const { return cp_; }
  bool takeCheckpoint(Checkpoint& out) { if (!cpPending_) return false; out = cp_; cpPending_ = false; return true; }
  uint8_t blueTileCount() const { return tileCount_; }
  const BlueTile& blueTile(uint8_t i) const { return tiles_[i]; }
  uint16_t blueVisits() const { return blueVisitsTotal_; }
  uint16_t blueCompleted() const { return blueCompletedTotal_; }
  uint16_t eventSeq() const { return eventSeq_; }
  const char* eventText() const { return eventBuf_; }

  void requestStop(const char* why) { if (isActive()) toSafeStop(why); }

  NavOutputs update(const NavInputs& in) {
    now_ = in.nowMs; heading_ = in.headingDeg; rate_ = in.rateDps;
    const RangeFilter& F = *in.front;
    const RangeFilter& L = *in.left;
    frontRef_ = in.front; leftRef_ = in.left; colour_ = in.colour;
    const float dL = in.odoLeftCm - odoL_, dR = in.odoRightCm - odoR_;
    odoL_ = in.odoLeftCm; odoR_ = in.odoRightCm;
    if (started_ && now_ > prevMs_) {
      const float dt = (now_ - prevMs_) * 0.001f;
      if (dt < 0.2f) {
        // Encoder odometry along the gyro heading; without encoders fall back to the speed model.
        const float d = kEncodersEnabled ? 0.5f * (dL + dR) : kCmPerSecFullSpeed * 0.5f * (last_.left + last_.right) * dt;
        x_ += d * cosf(heading_ * kDegToRad);
        y_ += d * sinf(heading_ * kDegToRad);
        if (kEncodersEnabled) trackStall(dL, dR, dt);
      }
    }
    prevMs_ = now_;
    const bool freshC = colour_ && colour_->seq() != seenC_;
    if (colour_) seenC_ = colour_->seq();
    if (freshC) updateArming();
    if (!armed_[1] && (now_ - blueDisarmMs_) > kBlueRearmMaxMs) armed_[1] = true;
    const bool freshF = F.seq() != seenF_;
    const bool freshL = L.seq() != seenL_;
    seenF_ = F.seq(); seenL_ = L.seq();
    if (freshF) { ++freshFSince_; frontBlocked_ = frontBlockedSample(F.lastKind(), F.rawCm()); }
    if (freshL) { ++freshLSince_; trackLeft(L); }
    if (!started_) { started_ = true; stateMs_ = now_; }

    if (isActive()) {
      if (!in.imuOk) return fault("IMU failure");
      if (F.implausibleRun() >= kImplausibleFaultCount) return fault("front sensor implausible");
      if (L.implausibleRun() >= kImplausibleFaultCount) return fault("left sensor implausible");
      if (F.noEchoRun() >= kFrontNoEchoFaultCount) return fault("front sensor no echo");
      if (kColorRequired && !in.colourOk) return fault("colour sensor failure");
      if (stallWhich_ == 1) return fault("wheel stall or encoder failure (left)");
      if (stallWhich_ == 2) return fault("wheel stall or encoder failure (right)");
      if (stallWhich_ == 3) return fault("wheel stall or encoder failure (both)");
    }

    NavOutputs out = {0, 0, true, false, state_};
    switch (state_) {
      case State::INITIALIZING:
        if (now_ - stateMs_ >= kStartDelayMs) enter(State::IMU_CALIBRATION);
        break;
      case State::IMU_CALIBRATION:
        if (in.imuCal == ImuCal::Failed) return fault("IMU calibration failed");
        if (in.imuCal == ImuCal::Done) enter(State::SENSOR_CHECK);
        break;
      case State::SENSOR_CHECK:
        if (F.validRun() >= kSensorCheckSamples && L.validRun() >= kSensorCheckSamples &&
            L.filteredCm() <= kWallTrackMaxCm && (!kColorRequired || in.colourOk)) {
          ref_ = heading_; wall_.reset(); episodeReset();
          enter(State::FORWARD_WALL_FOLLOW);
        } else if (now_ - stateMs_ > kSensorCheckTimeoutMs) {
          return fault("sensor check failed (need valid front echo, left wall and colour sensor)");
        }
        break;
      case State::FORWARD_WALL_FOLLOW: out = stepFollow(F, L, freshL); break;
      case State::LEFT_OPENING_CONFIRMATION: out = stepConfirm(F); break;
      case State::LEFT_OPENING_ADVANCE: out = stepAdvance(F); break;
      case State::OBSTACLE_STOP: out = stepObstacleStop(); break;
      case State::TURNING_LEFT_90:
      case State::TURNING_RIGHT_90:
      case State::TURNING_AROUND_180: out = stepTurn(); break;
      case State::SETTLE_AFTER_TURN: out = stepSettle(); break;
      case State::OBSTACLE_RECOVERY:
        out.brake = true;
        if (now_ - stateMs_ >= kRecoveryPauseMs) enter(State::OBSTACLE_STOP);
        break;
      case State::HAZARD_BACKOFF: out = stepHazardBackoff(); break;
      case State::BLUE_ADVANCE: out = stepBlueAdvance(); break;
      case State::BLUE_WAIT: out = stepBlueWait(); break;
      case State::SENSOR_FAULT:
      case State::SAFE_STOP:
        out.brake = true;
        break;
    }
    out.state = state_;
    last_ = out;
    return out;
  }

 private:
  bool isActive() const {
    return state_ != State::INITIALIZING && state_ != State::IMU_CALIBRATION && state_ != State::SENSOR_CHECK &&
           state_ != State::SENSOR_FAULT && state_ != State::SAFE_STOP;
  }
  void enter(State s) {
    state_ = s; stateMs_ = now_; freshFSince_ = 0; freshLSince_ = 0;
    enterOdoL_ = odoL_; enterOdoR_ = odoR_;
  }
  float travelCm() const { return 0.5f * ((odoL_ - enterOdoL_) + (odoR_ - enterOdoR_)); }
  bool advanceDone(float cm, uint32_t fallbackMs) const {
    if (!kEncodersEnabled) return (now_ - stateMs_) >= fallbackMs;
    return travelCm() >= cm || (now_ - stateMs_) > kAdvanceTimeoutMs;
  }
  // A wheel that is commanded but does not turn over a whole window is a fault (stall, wiring or encoder failure).
  void trackStall(float dL, float dR, float dt) {
    winT_ += dt;
    winCmdL_ += fabsf(last_.left) * dt; winCmdR_ += fabsf(last_.right) * dt;
    winDL_ += fabsf(dL); winDR_ += fabsf(dR);
    if (winT_ * 1000.0f >= kStallWindowMs) {
      const bool l = winCmdL_ / winT_ > kStallMinCmd && winDL_ < kStallMinCm;
      const bool r = winCmdR_ / winT_ > kStallMinCmd && winDR_ < kStallMinCm;
      if (isActive() && (l || r)) stallWhich_ = (l && r) ? 3 : (l ? 1 : 2);
      winT_ = winCmdL_ = winCmdR_ = winDL_ = winDR_ = 0;
    }
  }
  NavOutputs fault(const char* why) {
    reason_ = why; enter(State::SENSOR_FAULT);
    last_ = {0, 0, true, true, state_};
    return last_;
  }
  void toSafeStop(const char* why) { reason_ = why; enter(State::SAFE_STOP); last_ = {0, 0, true, true, state_}; }
  NavOutputs safeStop(const char* why) { toSafeStop(why); return last_; }
  void episodeReset() { turns_ = 0; recoveries_ = 0; hazards_ = 0; stage_ = Stage::None; }

  void eventf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(eventBuf_, sizeof(eventBuf_), fmt, ap);
    va_end(ap);
    ++eventSeq_;
  }

  // ---- floor colour ----
  bool colourFresh() const { return colour_ && colour_->seq() > 0 && (now_ - colour_->lastMs()) <= kColorStaleMs; }
  bool confirmed(ColorClass k) const {
    return colourFresh() && colour_->current() == k && colour_->run() >= kColorConfirmSamples;
  }
  static int armIndex(ColorClass k) { return k == ColorClass::Silver ? 0 : (k == ColorClass::Blue ? 1 : 2); }
  void updateArming() {
    const ColorClass kinds[3] = {ColorClass::Silver, ColorClass::Blue, ColorClass::Red};
    for (int i = 0; i < 3; ++i)
      if (colour_->current() != kinds[i] && colour_->run() >= kColorRearmSamples) armed_[i] = true;
  }
  BlueTile* tileHere(bool create) {
    const int16_t cx = (int16_t)floorf(x_ / kTileCm + 0.5f);
    const int16_t cy = (int16_t)floorf(y_ / kTileCm + 0.5f);
    for (uint8_t i = 0; i < tileCount_; ++i)
      if (tiles_[i].cx == cx && tiles_[i].cy == cy) return &tiles_[i];
    if (!create || tileCount_ >= kMaxBlueTiles) return nullptr;
    tiles_[tileCount_] = {cx, cy, 0, 0};
    return &tiles_[tileCount_++];
  }
  void processSilverRed() {
    if (confirmed(ColorClass::Silver) && armed_[armIndex(ColorClass::Silver)]) {
      armed_[armIndex(ColorClass::Silver)] = false;
      cp_ = {x_, y_, ref_, now_, ++cpCount_};
      cpPending_ = true;
      eventf("SILVER checkpoint #%u saved at est. (%.0f,%.0f) cm, heading %.0f", (unsigned)cp_.id, x_, y_, ref_);
    }
    if (confirmed(ColorClass::Red) && armed_[armIndex(ColorClass::Red)]) {
      armed_[armIndex(ColorClass::Red)] = false;
      ++dangerEntrances_;
      inDanger_ = true;
      eventf("RED: Dangerous Zone entrance #%u at est. (%.0f,%.0f) cm", (unsigned)dangerEntrances_, x_, y_);
    }
  }
  NavOutputs hazardHit() {
    if (++hazards_ > kMaxHazardsPerEpisode) return safeStop("repeated black-tile hazards");
    hazard_ = true;
    eventf("BLACK tile (hole) ahead at est. (%.0f,%.0f) cm: backing off", x_, y_);
    enter(State::HAZARD_BACKOFF);
    return stopped(true);
  }
  // Called from the driving states. Returns true if it took over (out is then valid).
  bool colourPreempt(NavOutputs& out) {
    if (!colour_) return false;
    if (confirmed(ColorClass::Black)) { out = hazardHit(); return true; }
    processSilverRed();
    if (confirmed(ColorClass::Blue) && armed_[1]) {
      armed_[1] = false;
      const BlueTile* t = tileHere(false);
      if (kBlueWaitEveryVisit || !t || t->completed == 0) {
        enter(State::BLUE_ADVANCE);
        out = drive(kSlowSpeed, headingSteer(ref_, heading_, rate_));
        return true;
      }
      blueDisarmMs_ = now_;
    }
    return false;
  }

  NavOutputs stepHazardBackoff() {
    const uint32_t t = now_ - stateMs_;
    if (t < kHazardStopMs) return stopped(true);
    const bool backedOff = kEncodersEnabled ? (travelCm() <= -kBackoffCm || t > kHazardStopMs + kAdvanceTimeoutMs)
                                            : (t >= kHazardStopMs + kBackoffMs);
    if (!backedOff) return {-kSlowSpeed, -kSlowSpeed, false, false, state_};
    enter(State::OBSTACLE_STOP);
    return stopped(true);
  }

  NavOutputs stepBlueAdvance() {
    if (frontBlocked_ && (now_ - frontRef_->lastMs()) <= kFrontStaleMs) { enter(State::OBSTACLE_STOP); return stopped(true); }
    if (confirmed(ColorClass::Black)) return hazardHit();
    if (!advanceDone(kBlueAdvanceCm, kBlueAdvanceMs)) return drive(kSlowSpeed, headingSteer(ref_, heading_, rate_));
    BlueTile* t = tileHere(true);
    if (t && t->visits < 255) ++t->visits;
    ++blueVisitsTotal_;
    waitStart_ = now_; blueLastSeen_ = now_; blueResets_ = 0;
    if (t) eventf("BLUE tile (%d,%d) visit #%u: waiting %lu ms", (int)t->cx, (int)t->cy, (unsigned)t->visits, (unsigned long)kBlueWaitMs);
    else eventf("BLUE tile visit (tile table full): waiting %lu ms", (unsigned long)kBlueWaitMs);
    enter(State::BLUE_WAIT);
    return stopped(true);
  }

  NavOutputs stepBlueWait() {
    NavOutputs out = stopped(true);
    if (confirmed(ColorClass::Blue)) blueLastSeen_ = now_;
    if (now_ - blueLastSeen_ > kBlueDropoutMs) {
      if (++blueResets_ > kBlueMaxResets) {
        eventf("BLUE wait abandoned: blue not seen continuously");
        armed_[1] = false; blueDisarmMs_ = now_;
        enter(State::FORWARD_WALL_FOLLOW);
        return out;
      }
      waitStart_ = now_; blueLastSeen_ = now_;             // consecutive-seconds rule: restart the timer
    }
    if (now_ - waitStart_ >= kBlueWaitMs) {
      BlueTile* t = tileHere(false);
      if (t && t->completed < 255) ++t->completed;
      ++blueCompletedTotal_;
      eventf("BLUE wait complete (%lu ms), moving on", (unsigned long)kBlueWaitMs);
      armed_[1] = false; blueDisarmMs_ = now_;
      enter(State::FORWARD_WALL_FOLLOW);
    }
    return out;
  }

  void trackLeft(const RangeFilter& L) {
    const RangeKind k = L.lastKind();
    const float cm = L.rawCm();
    if (leftOpenSample(k, cm)) { ++openRun_; wallRun_ = 0; }
    else if (leftWallSample(k, cm)) { ++wallRun_; openRun_ = 0; if (wallRun_ >= kMinWallSamples) hadWall_ = true; }
    else { openRun_ = 0; wallRun_ = 0; }   // ambiguous / implausible: counts as neither
  }

  float frontSpeedScale(const RangeFilter& F) const {
    if ((now_ - F.lastMs()) > kFrontStaleMs) return 0.0f;                // stale front: slowest
    if (F.lastKind() == RangeKind::NoEcho) return 1.0f;
    if (F.lastKind() != RangeKind::Valid) return 0.0f;
    return clampf((F.rawCm() - kFrontStopCm) / (kFrontSlowZoneCm - kFrontStopCm), 0.0f, 1.0f);
  }

  NavOutputs drive(float base, float steer) const {
    const WheelCmd w = mixDrive(base, steer);
    return {w.left, w.right, false, false, state_};
  }
  NavOutputs stopped(bool brake) const { return {0, 0, true, brake, state_}; }

  NavOutputs stepFollow(const RangeFilter& F, const RangeFilter& L, bool freshL) {
    if (frontBlocked_ && (now_ - F.lastMs()) <= kFrontStaleMs) { enter(State::OBSTACLE_STOP); return stopped(true); }
    if (now_ - stateMs_ >= kEpisodeResetMs) episodeReset();
    NavOutputs pre = {0, 0, true, false, state_};
    if (colourPreempt(pre)) return pre;

    // Opening candidate (needs an established wall, cooldown expired, no recent aborted confirmation).
    if (freshL && openRun_ >= 1 && hadWall_ && (now_ - lastTurnMs_) >= kTurnCooldownMs && now_ >= openBlockUntil_) {
      enter(State::LEFT_OPENING_CONFIRMATION);
      return drive(kSlowSpeed, headingSteer(ref_, heading_, rate_));
    }

    const bool haveWall = L.recentValid(now_, kLeftStaleMs) && L.filteredCm() <= kWallTrackMaxCm;
    if (haveWall) {
      noWallSince_ = 0;
      if (freshL && L.lastKind() == RangeKind::Valid) {
        const float thetaRad = (heading_ - ref_) * kDegToRad;   // deviation from the wall direction (assumed = ref)
        const float c = cosf(clampf(thetaRad, -1.0f, 1.0f));
        wall_.update(L.filteredCm() * (c < 0.5f ? 0.5f : c), now_);
      }
    } else {
      if (noWallSince_ == 0) noWallSince_ = now_;
      wall_.decay();
      if (now_ - noWallSince_ > kNoWallTimeoutMs) return safeStop("left wall lost for too long");
    }
    float speed = haveWall ? (kSlowSpeed + (kCruiseSpeed - kSlowSpeed) * frontSpeedScale(F)) : kSlowSpeed;
    if (inDanger_) speed *= kDangerSpeedScale;
    return drive(speed, headingSteer(ref_ + wall_.offsetDeg(), heading_, rate_));
  }

  NavOutputs stepConfirm(const RangeFilter& F) {
    if (frontBlocked_ && (now_ - F.lastMs()) <= kFrontStaleMs) { enter(State::OBSTACLE_STOP); return stopped(true); }
    { NavOutputs pre = {0, 0, true, false, state_}; if (colourPreempt(pre)) return pre; }
    if (wallRun_ >= 2) { enter(State::FORWARD_WALL_FOLLOW); return drive(kSlowSpeed, headingSteer(ref_, heading_, rate_)); }
    if (openRun_ >= kOpenConfirmSamples && (now_ - stateMs_) >= kOpenConfirmMinMs) {
      enter(State::LEFT_OPENING_ADVANCE);
    } else if (now_ - stateMs_ > kConfirmTimeoutMs) {
      openBlockUntil_ = now_ + kOpenBlockMs;
      enter(State::FORWARD_WALL_FOLLOW);
    }
    return drive(kSlowSpeed, headingSteer(ref_, heading_, rate_));
  }

  NavOutputs stepAdvance(const RangeFilter& F) {
    if (frontBlocked_ && (now_ - F.lastMs()) <= kFrontStaleMs) { enter(State::OBSTACLE_STOP); return stopped(true); }
    { NavOutputs pre = {0, 0, true, false, state_}; if (colourPreempt(pre)) return pre; }
    if (!advanceDone(kOpeningAdvanceCm, kOpeningAdvanceMs)) return drive(kSlowSpeed, headingSteer(ref_, heading_, rate_));
    if (openRun_ >= 2 && turns_ < kMaxTurnsPerEpisode) {
      beginTurn(ref_ + 90.0f, 90.0f, State::TURNING_LEFT_90);
      return stopped(true);
    }
    openBlockUntil_ = now_ + kOpenBlockMs;                    // opening vanished: false alarm, keep following
    enter(State::FORWARD_WALL_FOLLOW);
    return stopped(true);
  }

  void beginTurn(float targetAbs, float nominal, State s) {
    turn_.begin(targetAbs, heading_, nominal, now_);
    pendingRef_ = targetAbs;
    ++turns_;
    hadWall_ = false; openRun_ = 0; wallRun_ = 0; wall_.reset();
    enter(s);
  }

  LeftState classifyLeft(const RangeFilter& L) const {
    if (openRun_ >= kAssessSamples) return LeftState::Open;
    if (L.validRun() >= kAssessSamples) return LeftState::Blocked;   // valid echoes but not clearly open
    return LeftState::Unknown;
  }

  NavOutputs stepObstacleStop() {
    NavOutputs out = stopped(true);
    if (now_ - stateMs_ < kStopSettleMs || freshFSince_ < kAssessSamples || freshLSince_ < kAssessSamples) {
      if (now_ - stateMs_ > kAssessTimeoutMs) return safeStop("sensors not refreshing during obstacle stop");
      return out;
    }
    return decideAfterStop(out);
  }

  NavOutputs decideAfterStop(NavOutputs out) {
    const RangeFilter& F = *frontRef_;
    const RangeFilter& L = *leftRef_;
    if (!frontBlocked_ && !hazard_) {
      const bool clear = F.lastKind() == RangeKind::NoEcho ||
                         (F.lastKind() == RangeKind::Valid && F.rawCm() > kFrontResumeCm);
      if (clear) { stage_ = Stage::None; enter(State::FORWARD_WALL_FOLLOW); return out; }
      // Between stop and resume thresholds (or implausible): not proven clear, keep waiting.
      if (now_ - stateMs_ > kAssessTimeoutMs) return safeStop("front reading ambiguous");
      return out;
    }
    if (turns_ >= kMaxTurnsPerEpisode) return safeStop("too many turns without progress");
    switch (decideAvoidance(true, classifyLeft(L))) {
      case AvoidAction::TurnLeft:
        stage_ = Stage::None;
        beginTurn(ref_ + 90.0f, 90.0f, State::TURNING_LEFT_90);
        break;
      case AvoidAction::TurnRight:
        stage_ = Stage::RightTried;
        beginTurn(ref_ - 90.0f, 90.0f, State::TURNING_RIGHT_90);
        break;
      default:
        if (now_ - stateMs_ > kAssessTimeoutMs) return safeStop("left side state unknown");
        break;
    }
    return out;
  }

  NavOutputs stepTurn() {
    const TurnStatus st = turn_.update(heading_, rate_, now_);
    if (st == TurnStatus::Done) {
      hazard_ = false;                         // now facing a new direction
      ref_ = pendingRef_;                      // exact multiple of the commanded angle, no tolerance accumulation
      lastTurnMs_ = now_;
      enter(State::SETTLE_AFTER_TURN);
      return stopped(true);
    }
    if (st == TurnStatus::Failed) {
      turn_.update(heading_, rate_, now_);
      if (++recoveries_ > kMaxRecoveries) return safeStop("turn failed repeatedly");
      stage_ = Stage::None;
      enter(State::OBSTACLE_RECOVERY);
      return stopped(true);
    }
    return {turn_.cmdLeft(), turn_.cmdRight(), true, turn_.brake(), state_};
  }

  NavOutputs stepSettle() {
    NavOutputs out = stopped(true);
    if (now_ - stateMs_ < kPostTurnSettleMs || freshFSince_ < kAssessSamples || freshLSince_ < kAssessSamples) {
      if (now_ - stateMs_ > kSettleTimeoutMs) return safeStop("sensors not refreshing after turn");
      return out;
    }
    const bool blackNow = confirmed(ColorClass::Black);
    if (frontBlocked_ || blackNow) {
      if (blackNow) hazard_ = true;
      if (stage_ == Stage::RightTried) {          // right was tried and is also blocked: complete the U-turn
        stage_ = Stage::AroundTried;
        beginTurn(ref_ - 90.0f, 180.0f, State::TURNING_AROUND_180);
        return out;
      }
      if (stage_ == Stage::AroundTried) return safeStop("no safe maneuver: blocked on all tried sides");
      if (blackNow) { eventf("BLACK tile ahead after turn: backing off"); enter(State::HAZARD_BACKOFF); return out; }
      enter(State::OBSTACLE_STOP);
      return out;
    }
    stage_ = Stage::None;
    enter(State::FORWARD_WALL_FOLLOW);
    return out;
  }

  enum class Stage : uint8_t { None, RightTried, AroundTried };

  State state_ = State::INITIALIZING;
  const char* reason_ = "";
  uint32_t now_ = 0, stateMs_ = 0, lastTurnMs_ = 0, openBlockUntil_ = 0, noWallSince_ = 0;
  uint32_t seenF_ = 0, seenL_ = 0;
  uint8_t freshFSince_ = 0, freshLSince_ = 0;
  bool started_ = false, frontBlocked_ = false, hadWall_ = false;
  uint16_t openRun_ = 0, wallRun_ = 0;
  uint8_t turns_ = 0, recoveries_ = 0;
  Stage stage_ = Stage::None;
  float heading_ = 0, rate_ = 0, ref_ = 0, pendingRef_ = 0;
  WallController wall_;
  TurnController turn_;
  NavOutputs last_ = {0, 0, true, true, State::INITIALIZING};
  const RangeFilter* frontRef_ = nullptr;
  const RangeFilter* leftRef_ = nullptr;
  const ColorFilter* colour_ = nullptr;
  uint32_t seenC_ = 0, prevMs_ = 0, blueDisarmMs_ = 0, waitStart_ = 0, blueLastSeen_ = 0;
  bool armed_[3] = {true, true, true};       // silver, blue, red
  bool hazard_ = false, inDanger_ = false, cpPending_ = false;
  uint8_t hazards_ = 0, blueResets_ = 0, tileCount_ = 0;
  uint16_t dangerEntrances_ = 0, cpCount_ = 0, blueVisitsTotal_ = 0, blueCompletedTotal_ = 0, eventSeq_ = 0;
  float x_ = 0, y_ = 0, odoL_ = 0, odoR_ = 0, enterOdoL_ = 0, enterOdoR_ = 0;
  float winT_ = 0, winCmdL_ = 0, winCmdR_ = 0, winDL_ = 0, winDR_ = 0;
  uint8_t stallWhich_ = 0;
  Checkpoint cp_ = {0, 0, 0, 0, 0};
  BlueTile tiles_[kMaxBlueTiles];
  char eventBuf_[96] = {0};

};

}  // namespace nav

// ======================================== hw_types.h ========================================
// hw_types.h - types shared by the sketch (kept in a header so Arduino's auto-prototypes can see them).
#include <stdint.h>

struct Encoder {
  uint8_t pin;
  volatile int32_t ticks = 0;
  volatile int8_t dir = 1;          // sign of the last commanded wheel direction (single-channel encoder)
  volatile uint32_t lastUs = 0;
};

struct Ultrasonic {
  uint8_t trig, echo;
  volatile uint32_t riseUs = 0, widthUs = 0;
  volatile bool rose = false, done = false;
};

// ======================================== Robomaze_with_encoders.ino ========================================
using namespace cfg;
using nav::ImuCal;
using nav::RangeKind;
using nav::State;

// ================================ Motors ================================
struct Motor { uint8_t en, in1, in2; bool leftSide; bool invert; const char* name; };
static const Motor kMotors[] = {
    {kPinD1EnA, kPinD1In1, kPinD1In2, true, kInvertFrontLeft, "FRONT_LEFT"},
    {kPinD1EnB, kPinD1In3, kPinD1In4, false, kInvertFrontRight, "FRONT_RIGHT"},
    {kPinD2EnA, kPinD2In1, kPinD2In2, true, kInvertRearLeft, "REAR_LEFT"},
    {kPinD2EnB, kPinD2In3, kPinD2In4, false, kInvertRearRight, "REAR_RIGHT"},
};
constexpr size_t kMotorCount = sizeof(kMotors) / sizeof(kMotors[0]);

static Encoder encL{kPinEncLeft}, encR{kPinEncRight};
static float curL = 0, curR = 0;
static int8_t lastDir[kMotorCount];
static uint32_t dwellUntilMs[kMotorCount];
static uint32_t brakeUntilMs = 0;
static bool braking = false, stoppedHard = false;
static uint32_t lastSlewUs = 0;

static void coastMotor(size_t i) {
  digitalWrite(kMotors[i].in1, LOW);
  digitalWrite(kMotors[i].in2, LOW);
  ledcWrite(kMotors[i].en, 0);
}

static void writeMotor(size_t i, float cmd, uint32_t nowMs) {
  const Motor& m = kMotors[i];
  if (m.invert) cmd = -cmd;
  const int8_t dir = fabsf(cmd) < kCmdDeadband ? 0 : (cmd > 0 ? 1 : -1);
  if (dir == 0) { coastMotor(i); lastDir[i] = 0; return; }
  if (lastDir[i] != 0 && dir != lastDir[i]) { dwellUntilMs[i] = nowMs + kReverseDwellMs; lastDir[i] = 0; }
  if (nowMs < dwellUntilMs[i]) { coastMotor(i); return; }
  lastDir[i] = dir;
  ledcWrite(m.en, 0);                       // never change direction pins while driven
  digitalWrite(m.in1, dir > 0 ? HIGH : LOW);
  digitalWrite(m.in2, dir > 0 ? LOW : HIGH);
  const float mag = fminf(fabsf(cmd), 1.0f);
  ledcWrite(m.en, (uint32_t)(kPwmMin + (kPwmMax - kPwmMin) * mag));
}

static void updateEncoderDirections() {
  if (curL > kCmdDeadband) encL.dir = 1; else if (curL < -kCmdDeadband) encL.dir = -1;
  if (curR > kCmdDeadband) encR.dir = 1; else if (curR < -kCmdDeadband) encR.dir = -1;
}

static void driveMotors(uint32_t nowMs) {
  updateEncoderDirections();
  for (size_t i = 0; i < kMotorCount; ++i) writeMotor(i, kMotors[i].leftSide ? curL : curR, nowMs);
}

// Central stop: coast, or short electrical brake (IN1=IN2=HIGH, EN full) then coast.
static void stopMotors(bool brake, uint32_t nowMs) {
  curL = curR = 0;
  if (brake) {
    for (size_t i = 0; i < kMotorCount; ++i) {
      ledcWrite(kMotors[i].en, 0);
      digitalWrite(kMotors[i].in1, HIGH);
      digitalWrite(kMotors[i].in2, HIGH);
      ledcWrite(kMotors[i].en, 255);
      lastDir[i] = 0;
    }
    braking = true;
    brakeUntilMs = nowMs + kBrakeMs;
  } else {
    for (size_t i = 0; i < kMotorCount; ++i) { coastMotor(i); lastDir[i] = 0; }
  }
}

static float slew(float cur, float target, float dt) {
  const float lim = (fabsf(target) > fabsf(cur) ? kAccelPerSec : kDecelPerSec) * dt;
  const float d = target - cur;
  return fabsf(d) <= lim ? target : cur + (d > 0 ? lim : -lim);
}

static void setMotorSpeeds(float l, float r, bool immediate, uint32_t nowMs) {
  const uint32_t us = micros();
  float dt = (us - lastSlewUs) * 1e-6f;
  lastSlewUs = us;
  if (dt > 0.1f) dt = 0.1f;
  curL = immediate ? l : slew(curL, l, dt);
  curR = immediate ? r : slew(curR, r, dt);
  braking = false;
  driveMotors(nowMs);
}

static void applyOutputs(const nav::NavOutputs& o, uint32_t nowMs) {
  if (o.left == 0.0f && o.right == 0.0f && o.immediate) {
    if (!stoppedHard) { stopMotors(o.brake, nowMs); stoppedHard = true; }
  } else {
    stoppedHard = false;
    setMotorSpeeds(o.left, o.right, o.immediate, nowMs);
  }
  if (braking && nowMs >= brakeUntilMs) {
    for (size_t i = 0; i < kMotorCount; ++i) coastMotor(i);
    braking = false;
  }
}

static void initMotors() {
  for (size_t i = 0; i < kMotorCount; ++i) {       // safe states before anything else
    pinMode(kMotors[i].in1, OUTPUT); digitalWrite(kMotors[i].in1, LOW);
    pinMode(kMotors[i].in2, OUTPUT); digitalWrite(kMotors[i].in2, LOW);
    pinMode(kMotors[i].en, OUTPUT);  digitalWrite(kMotors[i].en, LOW);
  }
  for (size_t i = 0; i < kMotorCount; ++i) {
    ledcAttach(kMotors[i].en, kPwmHz, kPwmBits);
    ledcWrite(kMotors[i].en, 0);
  }
}

static void IRAM_ATTR encIsr(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  const uint32_t t = (uint32_t)esp_timer_get_time();
  if (t - e->lastUs < kEncMinPulseUs) return;
  e->lastUs = t;
  e->ticks += e->dir;
}
static void initEncoders() {
  pinMode(encL.pin, INPUT);
  pinMode(encR.pin, INPUT);
  attachInterruptArg(encL.pin, encIsr, &encL, RISING);
  attachInterruptArg(encR.pin, encIsr, &encR, RISING);
}
static float odoLeftCm() { return encL.ticks * kCmPerTick; }
static float odoRightCm() { return encR.ticks * kCmPerTick; }

#if BENCH_ENCODER_MONITOR
static void benchEncoderMonitor() {
  Serial.println("ENCODER MONITOR (20 s): turn each wheel by hand exactly ONE revolution and note the ticks.");
  Serial.println("ticks for one wheel revolution = kEncPulsesPerRev in config.h");
  for (uint32_t t0 = millis(); millis() - t0 < 20000; delay(250))
    Serial.printf("L ticks=%ld (%.1f cm)   R ticks=%ld (%.1f cm)\n", (long)encL.ticks, odoLeftCm(), (long)encR.ticks,
                  odoRightCm());
  encL.ticks = 0; encR.ticks = 0;
}
#endif

#if BENCH_MOTOR_TEST
static void benchMotorTest() {
  Serial.println("BENCH MOTOR TEST: wheels OFF the ground. Each motor: 1 s forward (should spin FORWARD).");
  for (size_t i = 0; i < kMotorCount; ++i) {
    Serial.printf("Motor %s...\n", kMotors[i].name);
    writeMotor(i, 0.6f, millis());
    delay(1000);
    coastMotor(i);
    Serial.printf("  encoders after this motor: L=%ld R=%ld ticks\n", (long)encL.ticks, (long)encR.ticks);
    delay(500);
  }
  Serial.println("Done. If a wheel spun backward, flip its invert flag in config.h. Halting.");
  for (;;) delay(1000);
}
#endif

// ================================ Ultrasonic ================================
static void IRAM_ATTR echoIsr(void* arg) {
  Ultrasonic* u = static_cast<Ultrasonic*>(arg);
  const uint32_t t = (uint32_t)esp_timer_get_time();
  if (gpio_get_level((gpio_num_t)u->echo)) { u->riseUs = t; u->rose = true; }
  else if (u->rose && !u->done) { u->widthUs = t - u->riseUs; u->done = true; }
}

static Ultrasonic usFront{kPinFrontTrig, kPinFrontEcho};
static Ultrasonic usLeft{kPinLeftTrig, kPinLeftEcho};
static nav::RangeFilter filtFront, filtLeft;
static Ultrasonic* activeUs = nullptr;
static nav::RangeFilter* activeFilt = nullptr;
static uint32_t slotStartUs = 0;
static bool nextIsFront = true;

static void initUltrasonic(Ultrasonic& u) {
  pinMode(u.trig, OUTPUT); digitalWrite(u.trig, LOW);
  pinMode(u.echo, INPUT);
  attachInterruptArg(u.echo, echoIsr, &u, CHANGE);
}

static void finishSlot(RangeKind kind, float cm, uint32_t nowMs) {
  activeFilt->push(kind, cm, nowMs);
  activeUs = nullptr;
}

static void ultrasonicService(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  if (activeUs) {
    if (activeUs->done) {
      const float cm = activeUs->widthUs * kSoundCmPerUs * 0.5f;
      if (cm < kMinRangeCm) finishSlot(RangeKind::Implausible, cm, nowMs);
      else if (cm > kMaxRangeCm) finishSlot(RangeKind::NoEcho, cm, nowMs);
      else finishSlot(RangeKind::Valid, cm, nowMs);
    } else if (nowUs - slotStartUs > kEchoTimeoutUs) {
      // Rising edge without a falling edge = echo line stuck high (fault); no edge at all = nothing heard.
      finishSlot(activeUs->rose ? RangeKind::Implausible : RangeKind::NoEcho, 0.0f, nowMs);
    }
  }
  if (!activeUs && (nowUs - slotStartUs) >= kSensorSlotMs * 1000u) {
    Ultrasonic* u = nextIsFront ? &usFront : &usLeft;
    nav::RangeFilter* f = nextIsFront ? &filtFront : &filtLeft;
    nextIsFront = !nextIsFront;
    slotStartUs = nowUs;
    if (gpio_get_level((gpio_num_t)u->echo)) {           // echo already high before trigger
      f->push(RangeKind::Implausible, 0.0f, nowMs);
      return;
    }
    u->rose = false; u->done = false;
    activeUs = u; activeFilt = f;
    digitalWrite(u->trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(u->trig, LOW);
  }
}

// ================================ MPU6050 ================================
namespace imu {
static bool ok = false;
static float bias = 0, heading = 0, rate = 0;
static uint32_t lastReadUs = 0, lastGoodMs = 0;
static uint8_t fails = 0, sat = 0;
static int16_t lastRaw = 0;
static uint16_t same = 0;
static ImuCal cal = ImuCal::Pending;
static uint16_t calN = 0;
static double calMean = 0, calM2 = 0;
static uint8_t calTries = 0;
static uint32_t nextCalMs = 0;

static bool writeReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(kImuAddr); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(kImuAddr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)kImuAddr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}
static bool readRaw(int16_t& raw) {
  uint8_t b[2];
  if (!readRegs(0x47, b, 2)) return false;      // GYRO_ZOUT
  raw = (int16_t)((b[0] << 8) | b[1]);
  return true;
}

static bool begin() {
  Wire.begin(kPinSda, kPinScl);
  Wire.setClock(kI2cHz);
  Wire.setTimeOut(10);
  uint8_t who = 0;
  if (!readRegs(0x75, &who, 1)) { Serial.println("IMU: no response on I2C"); return false; }
  if (who != 0x68) {
    if (who == 0x70 || who == 0x71 || who == 0x72 || who == 0x73 || who == 0x98)
      Serial.printf("IMU: WHO_AM_I=0x%02X (MPU6500-family clone), continuing\n", who);
    else { Serial.printf("IMU: unexpected WHO_AM_I=0x%02X\n", who); return false; }
  }
  bool r = writeReg(0x6B, 0x01);   // wake, PLL on X gyro
  delay(50);
  r &= writeReg(0x1A, 0x03);       // DLPF ~44 Hz
  r &= writeReg(0x19, 0x04);       // sample rate 200 Hz
  r &= writeReg(0x1B, 0x08);       // +-500 dps
  ok = r;
  lastReadUs = micros();
  return r;
}

static bool healthy(uint32_t nowMs) { return ok && (nowMs - lastGoodMs) <= kImuStaleMs; }

static void service(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  if (nowUs - lastReadUs < kImuPollUs) return;
  int16_t raw;
  if (!readRaw(raw)) { if (++fails >= kImuFailLimit) ok = false; return; }
  fails = 0;
  if (raw == lastRaw) { if (++same >= kImuFrozenLimit) ok = false; } else same = 0;
  lastRaw = raw;
  const float dt = (nowUs - lastReadUs) * 1e-6f;
  lastReadUs = nowUs;
  float r = ((float)raw / kGyroLsbPerDps - bias) * kGyroZSign;
  if (fabsf(r) > kImuMaxRateDps) { if (++sat > 3) ok = false; rate = 0; return; }
  sat = 0;
  if (fabsf(r) < kGyroDeadbandDps) r = 0;
  if (dt < 0.05f) heading += 0.5f * (rate + r) * dt;   // trapezoidal
  rate = r;
  lastGoodMs = nowMs;
}

static void calibrationStep(uint32_t nowMs) {
  if (cal != ImuCal::Pending) return;
  if (!ok) { cal = ImuCal::Failed; return; }
  if (nowMs < nextCalMs) return;
  nextCalMs = nowMs + kCalIntervalMs;
  int16_t raw;
  if (!readRaw(raw)) { if (++fails >= kImuFailLimit) { ok = false; cal = ImuCal::Failed; } return; }
  const double v = (double)raw / kGyroLsbPerDps;
  ++calN;
  const double d = v - calMean;
  calMean += d / calN;
  calM2 += d * (v - calMean);
  if (calN >= kCalSamples) {
    const double sd = sqrt(calM2 / (calN - 1));
    if (sd <= kCalMaxStdDps && fabs(calMean) <= kCalMaxBiasDps) {
      bias = (float)calMean; heading = 0; rate = 0; lastReadUs = micros(); lastGoodMs = nowMs;
      cal = ImuCal::Done;
      Serial.printf("IMU: calibrated, bias=%.3f dps, noise sd=%.3f dps\n", bias, sd);
    } else {
      Serial.printf("IMU: calibration rejected (sd=%.3f, bias=%.2f) - keep robot still\n", sd, calMean);
      calN = 0; calMean = 0; calM2 = 0;
      if (++calTries >= kCalRetries) cal = ImuCal::Failed;
    }
  }
}
}  // namespace imu

// ================================ TCS34725 floor colour sensor ================================
namespace colour {
static bool ok = false;
static uint8_t fails = 0;
static uint32_t lastPollMs = 0, lastGoodMs = 0;
static nav::ColorRefs refs;
static uint16_t rawRefs[5][4];              // black, silver, blue, red, floor as {R,G,B,C}
static nav::ColorSample last = {0, 0, 0, 0};
static nav::ColorFilter filt;
static Preferences prefs;
static const char* const kKeys[5] = {"black", "silver", "blue", "red", "floor"};

static bool writeReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(kColorAddr); Wire.write(0x80 | reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(kColorAddr); Wire.write(0xA0 | reg);   // auto-increment
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)kColorAddr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}
static bool readSample(nav::ColorSample& s) {
  uint8_t b[8];
  if (!readRegs(0x14, b, 8)) return false;
  s.c = (uint16_t)(b[0] | (b[1] << 8));
  s.r = (uint16_t)(b[2] | (b[3] << 8));
  s.g = (uint16_t)(b[4] | (b[5] << 8));
  s.b = (uint16_t)(b[6] | (b[7] << 8));
  return true;
}

static void buildRefs() {
  refs.black = nav::makeRef(rawRefs[0]); refs.silver = nav::makeRef(rawRefs[1]);
  refs.blue = nav::makeRef(rawRefs[2]);  refs.red = nav::makeRef(rawRefs[3]);
  refs.floor = nav::makeRef(rawRefs[4]);
}
static void loadRefs() {
  const uint16_t* defs[5] = {kRefBlack, kRefSilver, kRefBlue, kRefRed, kRefFloor};
  prefs.begin("colour", false);
  for (int i = 0; i < 5; ++i) {
    memcpy(rawRefs[i], defs[i], sizeof(rawRefs[i]));
    if (prefs.getBytesLength(kKeys[i]) == sizeof(rawRefs[i])) prefs.getBytes(kKeys[i], rawRefs[i], sizeof(rawRefs[i]));
  }
  buildRefs();
}

static bool begin() {
  loadRefs();
  uint8_t id = 0;
  if (!readRegs(0x12, &id, 1) || (id != 0x44 && id != 0x4D)) {
    Serial.printf("COLOUR: TCS34725 not found (ID read 0x%02X)\n", id);
    return false;
  }
  bool r = writeReg(0x01, kColorAtime);
  r &= writeReg(0x0F, kColorGain);
  r &= writeReg(0x00, 0x01);            // power on
  delay(3);
  r &= writeReg(0x00, 0x03);            // power on + RGBC enable
  ok = r;
  lastGoodMs = millis();
  return r;
}

static bool healthy(uint32_t nowMs) { return ok && (nowMs - lastGoodMs) <= kColorStaleMs; }

static void service(uint32_t nowMs) {
  if (!ok || nowMs - lastPollMs < kColorPollMs) return;
  lastPollMs = nowMs;
  nav::ColorSample smp;
  if (!readSample(smp)) { if (++fails >= kColorFailLimit) ok = false; return; }
  fails = 0;
  last = smp;
  lastGoodMs = nowMs;
  filt.push(nav::classifyColor(smp, refs), nowMs);
}

static bool captureAverage(uint16_t out[4]) {
  uint32_t sr = 0, sg = 0, sb = 0, sc = 0;
  const int n = 20;
  for (int i = 0; i < n; ++i) {
    nav::ColorSample smp;
    delay(kColorPollMs);
    if (!readSample(smp)) return false;
    sr += smp.r; sg += smp.g; sb += smp.b; sc += smp.c;
  }
  out[0] = sr / n; out[1] = sg / n; out[2] = sb / n; out[3] = sc / n;
  return true;
}

// Blocking; only entered at boot with the motors off.
static void calibrationMode() {
  Serial.println("COLOUR CALIBRATION (motors off). Put the sensor over a tile, then send:");
  Serial.println("  B=black  S=silver  U=blue  R=red  F=normal floor  (saves that tile)   Z=reset to defaults   Q=quit");
  uint32_t lastPrint = 0;
  for (;;) {
    nav::ColorSample smp;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      if (readSample(smp))
        Serial.printf("R=%u G=%u B=%u C=%u -> %s\n", smp.r, smp.g, smp.b, smp.c,
                      nav::colorName(nav::classifyColor(smp, refs)));
      else Serial.println("colour sensor read failed");
    }
    if (!Serial.available()) { delay(5); continue; }
    const char ch = toupper(Serial.read());
    int idx = ch == 'B' ? 0 : ch == 'S' ? 1 : ch == 'U' ? 2 : ch == 'R' ? 3 : ch == 'F' ? 4 : -1;
    if (idx >= 0) {
      uint16_t v[4];
      if (captureAverage(v)) {
        memcpy(rawRefs[idx], v, sizeof(v));
        prefs.putBytes(kKeys[idx], v, sizeof(v));
        buildRefs();
        Serial.printf("Saved %s = {R=%u,G=%u,B=%u,C=%u}\n", kKeys[idx], v[0], v[1], v[2], v[3]);
      } else Serial.println("capture failed");
    } else if (ch == 'Z') {
      for (int i = 0; i < 5; ++i) prefs.remove(kKeys[i]);
      loadRefs();
      Serial.println("Reference colours reset to config.h defaults");
    } else if (ch == 'Q') {
      Serial.println("Leaving calibration mode");
      return;
    }
  }
}
}  // namespace colour

// ================================ Main ================================
static nav::Navigator navigator;
static State lastPrintedState = State::INITIALIZING;
static uint32_t lastDiagMs = 0;
static uint16_t lastEventSeq = 0;

static void reportDiagnostics(uint32_t nowMs, const nav::NavOutputs& o) {
  if (navigator.state() != lastPrintedState) {
    lastPrintedState = navigator.state();
    Serial.printf("[%lu] STATE -> %s %s\n", (unsigned long)nowMs, nav::stateName(lastPrintedState), navigator.reason());
  }
  if (nowMs - lastDiagMs < kDiagPeriodMs) return;
  lastDiagMs = nowMs;
  if (navigator.eventSeq() != lastEventSeq) {
    lastEventSeq = navigator.eventSeq();
    Serial.printf("[%lu] EVENT %s\n", (unsigned long)nowMs, navigator.eventText());
  }
  if (Serial.availableForWrite() < 300) return;       // never let logging block control
  const nav::TurnController& t = navigator.turn();
  const bool turning = o.state == State::TURNING_LEFT_90 || o.state == State::TURNING_RIGHT_90 ||
                       o.state == State::TURNING_AROUND_180;
  Serial.printf("t=%lu %s F=%.1f%s L=%.1f%s tgt=%.1f err=%.1f hdg=%.1f ref=%.1f off=%.1f w=%.1f cmd=%.2f/%.2f",
                (unsigned long)nowMs, nav::stateName(o.state), filtFront.rawCm(),
                filtFront.lastKind() == RangeKind::Valid ? "" : "(x)", filtLeft.filteredCm(),
                filtLeft.lastKind() == RangeKind::Valid ? "" : "(x)", kWallTargetCm, navigator.wallErrorCm(),
                imu::heading, navigator.headingRefDeg(), navigator.wallOffsetDeg(), imu::rate, o.left, o.right);
  Serial.printf(" col=%s(%u,%u,%u,%u) pos=(%.0f,%.0f) odo=%.1f/%.1f%s", nav::colorName(colour::filt.current()), colour::last.r,
                colour::last.g, colour::last.b, colour::last.c, navigator.xCm(), navigator.yCm(),
                navigator.odoLeftCm(), navigator.odoRightCm(), navigator.inDangerZone() ? " DANGER" : "");
  if (turning)
    Serial.printf(" turn[tgt=%.1f meas=%.1f rem=%.1f corr=%u]", t.targetDeg(), t.measuredDeg(), t.remainingDeg(),
                  t.corrections());
  Serial.println();
}

void setup() {
  initMotors();                                        // 1. safe motor outputs first
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);                                // 2. diagnostics
  delay(200);
  Serial.println("\nRobomaze_with_encoders (floor colour + encoders) booting");
  initEncoders();
#if BENCH_MOTOR_TEST
  benchMotorTest();
#endif
#if BENCH_ENCODER_MONITOR
  benchEncoderMonitor();
#endif
  initUltrasonic(usFront);                             // 4. ultrasonic sensors
  initUltrasonic(usLeft);
  if (!imu::begin()) Serial.println("IMU init FAILED - robot will not move");   // 3. I2C + MPU6050
  if (!colour::begin()) Serial.println("Colour sensor init FAILED - robot will not move");
  else {
    Serial.println("Send 'c' within 5 s for colour calibration mode (otherwise normal start)");
    for (uint32_t t0 = millis(); millis() - t0 < 5000;) {
      if (Serial.available() && toupper(Serial.read()) == 'C') { colour::calibrationMode(); break; }
      delay(10);
    }
  }
  slotStartUs = micros();
}

void loop() {
  const uint32_t nowMs = millis();
  if (Serial.available() && Serial.read() == 'x') navigator.requestStop("serial stop command");

  if (navigator.state() == State::IMU_CALIBRATION) imu::calibrationStep(nowMs);   // stationary only
  else imu::service(nowMs);
  ultrasonicService(nowMs);
  colour::service(nowMs);
  nav::Checkpoint cp;
  if (navigator.takeCheckpoint(cp)) colour::prefs.putBytes("cp", &cp, sizeof(cp));   // survives reboot

  nav::NavInputs in;
  in.nowMs = nowMs;
  in.front = &filtFront;
  in.left = &filtLeft;
  in.headingDeg = imu::heading;
  in.rateDps = imu::rate;
  in.imuOk = imu::healthy(nowMs) || navigator.state() == State::INITIALIZING ||
             navigator.state() == State::IMU_CALIBRATION || navigator.state() == State::SENSOR_CHECK;
  in.imuCal = imu::cal;
  in.colour = &colour::filt;
  in.colourOk = colour::healthy(nowMs);
  in.odoLeftCm = odoLeftCm();
  in.odoRightCm = odoRightCm();
  if (!imu::ok && in.imuCal == ImuCal::Pending) in.imuCal = ImuCal::Failed;

  const nav::NavOutputs out = navigator.update(in);
  applyOutputs(out, nowMs);
  reportDiagnostics(nowMs, out);
  delay(1);
}
