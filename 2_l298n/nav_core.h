// nav_core.h - hardware-independent navigation logic (no Arduino headers).
// Compiled both into the ESP32 sketch and into the host unit tests.
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"

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

// ============================== Navigation state machine ==============================
enum class State : uint8_t {
  INITIALIZING, IMU_CALIBRATION, SENSOR_CHECK, FORWARD_WALL_FOLLOW, LEFT_OPENING_CONFIRMATION,
  LEFT_OPENING_ADVANCE, OBSTACLE_STOP, TURNING_LEFT_90, TURNING_RIGHT_90, TURNING_AROUND_180,
  SETTLE_AFTER_TURN, OBSTACLE_RECOVERY, SENSOR_FAULT, SAFE_STOP
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

  void requestStop(const char* why) { if (isActive()) toSafeStop(why); }

  NavOutputs update(const NavInputs& in) {
    now_ = in.nowMs; heading_ = in.headingDeg; rate_ = in.rateDps;
    const RangeFilter& F = *in.front;
    const RangeFilter& L = *in.left;
    frontRef_ = in.front; leftRef_ = in.left;
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
            L.filteredCm() <= kWallTrackMaxCm) {
          ref_ = heading_; wall_.reset(); episodeReset();
          enter(State::FORWARD_WALL_FOLLOW);
        } else if (now_ - stateMs_ > kSensorCheckTimeoutMs) {
          return fault("sensor check failed (need valid front echo and left wall)");
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
  }
  NavOutputs fault(const char* why) {
    reason_ = why; enter(State::SENSOR_FAULT);
    last_ = {0, 0, true, true, state_};
    return last_;
  }
  void toSafeStop(const char* why) { reason_ = why; enter(State::SAFE_STOP); last_ = {0, 0, true, true, state_}; }
  NavOutputs safeStop(const char* why) { toSafeStop(why); return last_; }
  void episodeReset() { turns_ = 0; recoveries_ = 0; stage_ = Stage::None; }

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
    const float speed = haveWall ? (kSlowSpeed + (kCruiseSpeed - kSlowSpeed) * frontSpeedScale(F)) : kSlowSpeed;
    return drive(speed, headingSteer(ref_ + wall_.offsetDeg(), heading_, rate_));
  }

  NavOutputs stepConfirm(const RangeFilter& F) {
    if (frontBlocked_ && (now_ - F.lastMs()) <= kFrontStaleMs) { enter(State::OBSTACLE_STOP); return stopped(true); }
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
    if (now_ - stateMs_ < kOpeningAdvanceMs) return drive(kSlowSpeed, headingSteer(ref_, heading_, rate_));
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
    if (!frontBlocked_) {
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
    if (frontBlocked_) {
      if (stage_ == Stage::RightTried) {          // right was tried and is also blocked: complete the U-turn
        stage_ = Stage::AroundTried;
        beginTurn(ref_ - 90.0f, 180.0f, State::TURNING_AROUND_180);
        return out;
      }
      if (stage_ == Stage::AroundTried) return safeStop("no safe maneuver: blocked on all tried sides");
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

};

}  // namespace nav
