// config.h - every tunable value for the 2x L298N (four independent motors) wall-following robot.
// Pin numbers here are a PROPOSED wiring (no wiring documentation existed). Wire the robot
// to match this table, or edit the pins here - the static_asserts at the bottom reject
// GPIOs that are unsafe for their role.
#pragma once
#include <stdint.h>

#define ROBOT_L298N_COUNT 2
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
static_assert(kPwmMin < kPwmMax, "PWM range");
static_assert(kTurnMinSpeed <= kTurnMaxSpeed && kSlowSpeed <= kCruiseSpeed, "speed ordering");
static_assert(kSensorSlotMs * 1000u > kEchoTimeoutUs, "sensor slot must outlast the echo timeout (cross-talk)");

}  // namespace cfg
