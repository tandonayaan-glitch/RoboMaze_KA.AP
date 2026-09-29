// Host unit tests for nav_core.h. Build/run: see README (test/run_tests.sh).
#include <stdio.h>
#include <string.h>
#include "../../Robomaze_colour_sensors/nav_core.h"

using namespace nav;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond) do { if (cond) { ++g_pass; } else { ++g_fail; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a=(a), _b=(b); if (fabs(_a-_b) <= (tol)) { ++g_pass; } else { ++g_fail; printf("  FAIL %s:%d  %s=%.3f expected %.3f +-%.3f\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)
#define RUN(fn) do { printf("%s\n", #fn); fn(); } while (0)

// ------------------------------------------------------------------ simulator
struct Sim {
  Navigator nav;
  RangeFilter F, L;
  uint32_t t = 0;
  float heading = 0, rate = 0;
  bool imuOk = true;
  ImuCal cal = ImuCal::Done;
  // scripted world
  RangeKind fKind = RangeKind::Valid, lKind = RangeKind::Valid;
  float fCm = 150, lCm = 9;
  float yawGainDps = 180;      // yaw rate at full differential
  bool plantYaw = true;        // heading follows wheel differential
  NavOutputs out{0, 0, true, false, State::INITIALIZING};
  uint32_t nextSensor = 0, nextColor = 0;
  bool nextFront = true;
  ColorFilter C;
  ColorClass cKind = ColorClass::Other;
  bool colourOk = true;

  void tick(uint32_t dt = 5) {
    t += dt;
    if (plantYaw) rate = yawGainDps * (out.right - out.left) * 0.5f;
    heading += rate * dt * 0.001f;
    if (t >= nextSensor) {
      nextSensor = t + kSensorSlotMs;
      if (nextFront) F.push(fKind, fCm, t); else L.push(lKind, lCm, t);
      nextFront = !nextFront;
    }
    if (t >= nextColor) { nextColor = t + kColorPollMs; C.push(cKind, t); }
    NavInputs in{t, &F, &L, heading, rate, imuOk, cal, &C, colourOk};
    out = nav.update(in);
  }
  void run(uint32_t ms) { for (uint32_t e = t + ms; t < e;) tick(); }
  bool runUntil(State s, uint32_t maxMs) {
    for (uint32_t e = t + maxMs; t < e;) { tick(); if (nav.state() == s) return true; }
    return false;
  }
  void toFollow() { CHECK(runUntil(State::FORWARD_WALL_FOLLOW, 10000)); }
  // establish an established wall (hadWall) and cool-down expiry
  void settleFollow() { toFollow(); run(kTurnCooldownMs + 800); }
};

// ------------------------------------------------------------------ range filter
static void test_range_filter() {
  RangeFilter f;
  f.push(RangeKind::Valid, 10, 1); f.push(RangeKind::Valid, 10, 2); f.push(RangeKind::Valid, 90, 3);
  CHECK_NEAR(f.filteredCm(), 10.0, 0.01);                       // spike rejected by median
  f.push(RangeKind::NoEcho, 0, 4);
  CHECK_NEAR(f.rawCm(), 90.0, 0.01);                            // a missing echo never becomes 0 cm
  CHECK(f.noEchoRun() == 1 && f.validRun() == 0);
  f.push(RangeKind::Implausible, 0.4f, 5);
  CHECK(f.implausibleRun() == 1 && f.noEchoRun() == 0);
  CHECK(!f.recentValid(1000, 300));
  CHECK(f.recentValid(200, 300));
}

// ------------------------------------------------------------------ thresholds
static void test_thresholds() {
  CHECK(frontBlockedSample(RangeKind::Valid, 7.0f));            // exactly 7 cm blocks
  CHECK(frontBlockedSample(RangeKind::Valid, 3.0f));
  CHECK(!frontBlockedSample(RangeKind::Valid, 7.01f));
  CHECK(!frontBlockedSample(RangeKind::NoEcho, 0.0f));          // missing echo is not "0 cm"
  CHECK(!frontBlockedSample(RangeKind::Implausible, 1.0f));
  CHECK(leftOpenSample(RangeKind::NoEcho, 0));
  CHECK(leftOpenSample(RangeKind::Valid, 25.1f));
  CHECK(!leftOpenSample(RangeKind::Valid, 25.0f));
  CHECK(!leftOpenSample(RangeKind::Implausible, 100));
  CHECK(leftWallSample(RangeKind::Valid, 17.9f));
  CHECK(!leftWallSample(RangeKind::Valid, 18.0f));              // hysteresis band 18..25
  CHECK(decideAvoidance(false, LeftState::Open) == AvoidAction::Resume);
  CHECK(decideAvoidance(true, LeftState::Open) == AvoidAction::TurnLeft);
  CHECK(decideAvoidance(true, LeftState::Blocked) == AvoidAction::TurnRight);
  CHECK(decideAvoidance(true, LeftState::Unknown) == AvoidAction::Wait);
}

// ------------------------------------------------------------------ wall controller
static void test_wall_controller() {
  WallController w;
  CHECK_NEAR(w.update(9.0f, 100), 0.0, 1e-4);                   // at target: straight
  w.reset();
  CHECK_NEAR(w.update(9.5f, 100), 0.0, 1e-4);                   // inside deadband
  w.reset();
  CHECK(w.update(13.0f, 100) > 0);                              // too far -> steer left (toward wall)
  w.reset();
  CHECK(w.update(5.0f, 100) < 0);                               // too close -> steer right (away)
  w.reset();
  CHECK_NEAR(w.update(200.0f, 100), kMaxWallOffsetDeg, 1e-3);   // clamped
  w.reset();
  CHECK_NEAR(w.update(0.0f, 100), -kMaxWallOffsetDeg, 1e-3);
  // steering and mixing bounds
  CHECK_NEAR(headingSteer(100, 0, 0), kMaxSteer, 1e-6);
  CHECK_NEAR(headingSteer(-100, 0, 0), -kMaxSteer, 1e-6);
  WheelCmd c = mixDrive(kCruiseSpeed, kMaxSteer);
  CHECK(c.left >= 0 && c.right <= 1.0f && c.right > c.left);
  c = mixDrive(0.9f, 0.45f);
  CHECK(c.right <= 1.0f + 1e-6f && c.left >= 0);
}

// ------------------------------------------------------------------ startup
static void test_startup_gating() {
  Sim s;
  s.run(kStartDelayMs - 200);
  CHECK(s.nav.state() == State::INITIALIZING);
  CHECK(s.out.left == 0 && s.out.right == 0);
  s.cal = ImuCal::Pending;
  s.run(1500);
  CHECK(s.nav.state() == State::IMU_CALIBRATION);
  CHECK(s.out.left == 0 && s.out.right == 0);
  s.cal = ImuCal::Done;
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 3000));
}
static void test_startup_failures() {
  Sim a; a.cal = ImuCal::Failed;
  a.run(6000);
  CHECK(a.nav.state() == State::SENSOR_FAULT && a.out.left == 0 && a.out.right == 0);
  Sim b; b.lKind = RangeKind::NoEcho;                            // no left wall at boot
  b.run(9000);
  CHECK(b.nav.state() == State::SENSOR_FAULT && b.out.left == 0);
  Sim c; c.fKind = RangeKind::Implausible;                       // dead front sensor
  c.run(9000);
  CHECK(c.nav.state() == State::SENSOR_FAULT);
  Sim d; d.lCm = 60;                                             // wall too far to start
  d.run(9000);
  CHECK(d.nav.state() == State::SENSOR_FAULT);
}

// ------------------------------------------------------------------ faults
static void test_imu_failure_stops_motors() {
  Sim s; s.settleFollow();
  s.run(300);
  CHECK(s.out.left > 0 && s.out.right > 0);
  s.imuOk = false;
  s.tick();
  CHECK(s.nav.state() == State::SENSOR_FAULT);
  CHECK(s.out.left == 0 && s.out.right == 0 && s.out.immediate);
  s.imuOk = true;                                                // fault is latched
  s.run(2000);
  CHECK(s.nav.state() == State::SENSOR_FAULT && s.out.left == 0 && s.out.right == 0);
}
static void test_range_sensor_faults() {
  Sim s; s.settleFollow();
  s.lKind = RangeKind::Implausible;
  s.run(kSensorSlotMs * 2 * (kImplausibleFaultCount + 2));
  CHECK(s.nav.state() == State::SENSOR_FAULT && s.out.left == 0);
  Sim f; f.settleFollow();
  f.fKind = RangeKind::Implausible;
  f.run(kSensorSlotMs * 2 * (kImplausibleFaultCount + 2));
  CHECK(f.nav.state() == State::SENSOR_FAULT && f.out.left == 0);
  Sim g; g.settleFollow();
  g.fKind = RangeKind::NoEcho;                                   // front blind for too long
  g.run(kSensorSlotMs * 2 * (kFrontNoEchoFaultCount + 5));
  CHECK(g.nav.state() == State::SENSOR_FAULT && g.out.left == 0);
}
static void test_intermittent_bad_reading_not_fatal() {
  Sim s; s.settleFollow();
  for (int i = 0; i < 20; ++i) {                                 // isolated implausible echoes
    s.fKind = (i % 4 == 0) ? RangeKind::Implausible : RangeKind::Valid;
    s.run(kSensorSlotMs * 2);
  }
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
}
static void test_wall_lost_bounded() {
  Sim s; s.settleFollow();
  s.lKind = RangeKind::NoEcho;
  s.run(kNoWallTimeoutMs + 6000);
  CHECK(s.nav.state() == State::SAFE_STOP);
  CHECK(s.out.left == 0 && s.out.right == 0);
}

// ------------------------------------------------------------------ left opening
static void test_opening_confirmation() {
  Sim s; s.settleFollow();
  s.lKind = RangeKind::NoEcho;
  s.run(kSensorSlotMs * 2 * 2);                                  // ~2 open samples only
  CHECK(s.nav.state() == State::LEFT_OPENING_CONFIRMATION || s.nav.state() == State::FORWARD_WALL_FOLLOW);
  CHECK(s.nav.state() != State::TURNING_LEFT_90);
  s.lKind = RangeKind::Valid; s.lCm = 9;                         // single missed echoes: back to wall following
  s.run(600);
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);

  Sim g; g.settleFollow();                                       // genuine opening
  g.lKind = RangeKind::Valid; g.lCm = 60;
  CHECK(g.runUntil(State::LEFT_OPENING_CONFIRMATION, 1000));
  CHECK(g.runUntil(State::LEFT_OPENING_ADVANCE, 2000));
  CHECK(g.runUntil(State::TURNING_LEFT_90, 2000));
}
static void test_opening_hysteresis_band() {
  Sim s; s.settleFollow();
  s.lCm = 22;                                                    // 18..25 = neither wall nor open
  s.run(3000);
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
  CHECK(s.nav.state() != State::TURNING_LEFT_90);
}
static void test_no_second_turn_without_wall() {
  Sim s; s.settleFollow();
  s.lCm = 60;                                                    // opening, and no wall in the new corridor either
  CHECK(s.runUntil(State::TURNING_LEFT_90, 5000));
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 8000));
  bool turned = false;
  for (int i = 0; i < 3000 / 5; ++i) {
    s.tick();
    const State st = s.nav.state();
    if (st == State::TURNING_LEFT_90 || st == State::TURNING_RIGHT_90 || st == State::TURNING_AROUND_180) turned = true;
  }
  CHECK(!turned);                                                // wall must be re-established first
  s.run(3000);
  CHECK(s.nav.state() == State::SAFE_STOP);                      // and the wall-less crawl is bounded
  CHECK(s.out.left == 0 && s.out.right == 0);
}
static void test_left_turn_completes_and_resumes() {
  Sim s; s.settleFollow();
  const float ref0 = s.nav.headingRefDeg();
  s.lKind = RangeKind::Valid; s.lCm = 60;
  CHECK(s.runUntil(State::TURNING_LEFT_90, 5000));
  s.lCm = 9;                                                     // new corridor's wall appears after turning
  int turningStates = 0;
  State prev = s.nav.state();
  for (int i = 0; i < 4000 && s.nav.state() != State::FORWARD_WALL_FOLLOW; ++i) {
    s.tick();
    if (s.nav.state() == State::TURNING_LEFT_90 && prev != State::TURNING_LEFT_90) ++turningStates;
    prev = s.nav.state();
  }
  CHECK(turningStates == 0);                                     // turn was already running; it did not restart
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
  CHECK_NEAR(s.heading - ref0, 90.0, kTurnTolDeg);
  CHECK_NEAR(s.nav.headingRefDeg(), ref0 + 90.0, 1e-3);
}

// ------------------------------------------------------------------ front obstacle
static void test_front_stop_boundary() {
  Sim s; s.settleFollow();
  s.fCm = 7.01f; s.run(400);
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
  s.fCm = 7.0f; s.run(kSensorSlotMs * 2 + 20);
  CHECK(s.nav.state() == State::OBSTACLE_STOP);
  CHECK(s.out.left == 0 && s.out.right == 0);
}
static void test_front_false_alarm_resumes() {
  Sim s; s.settleFollow();
  s.fCm = 5; s.run(kSensorSlotMs * 2 + 20);
  CHECK(s.nav.state() == State::OBSTACLE_STOP);
  s.fCm = 100;
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 2000));
}
static void test_front_blocked_left_open_turns_left() {
  Sim s; s.settleFollow();
  s.fCm = 5; s.lCm = 60;
  CHECK(s.runUntil(State::TURNING_LEFT_90, 3000));
}
static void test_front_blocked_left_blocked_turns_right() {
  Sim s; s.settleFollow();
  s.fCm = 5; s.lCm = 9;
  CHECK(s.runUntil(State::TURNING_RIGHT_90, 3000));
  s.fCm = 100;                                                   // right route clear
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 8000));
  CHECK_NEAR(s.heading - s.nav.headingRefDeg(), 0.0, kTurnTolDeg + 0.5);
  CHECK(s.nav.headingRefDeg() < -80);
}
static void test_dead_end_u_turn_then_stop() {
  Sim s; s.settleFollow();
  const float ref0 = s.nav.headingRefDeg();
  s.fCm = 5; s.lCm = 9;                                          // everything blocked, world never clears
  CHECK(s.runUntil(State::TURNING_RIGHT_90, 3000));
  CHECK(s.runUntil(State::TURNING_AROUND_180, 8000));            // right also blocked: complete reversal
  CHECK(s.runUntil(State::SAFE_STOP, 8000));                     // still blocked after reversing: stop, don't loop
  s.run(3000);
  CHECK(s.nav.state() == State::SAFE_STOP && s.out.left == 0 && s.out.right == 0);
  CHECK_NEAR(s.nav.headingRefDeg(), ref0 - 180.0, 1e-3);
}
static void test_dead_end_u_turn_escape() {
  Sim s; s.settleFollow();
  const float ref0 = s.nav.headingRefDeg();
  s.fCm = 5; s.lCm = 9;
  CHECK(s.runUntil(State::TURNING_AROUND_180, 12000));
  s.fCm = 100;                                                   // behind us is open
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 8000));
  CHECK_NEAR(s.heading - ref0, -180.0, kTurnTolDeg + 0.5);
}
static void test_left_unknown_never_guesses() {
  Sim s; s.settleFollow();
  s.fCm = 5; s.lKind = RangeKind::Implausible;
  s.run(4000);
  CHECK(s.nav.state() != State::TURNING_LEFT_90 && s.nav.state() != State::TURNING_RIGHT_90);
  CHECK(s.out.left == 0 && s.out.right == 0);
}
static void test_no_overlapping_turns() {
  Sim s; s.settleFollow();
  s.fCm = 5; s.lCm = 60;
  CHECK(s.runUntil(State::TURNING_LEFT_90, 3000));
  s.fCm = 3;                                                     // sensor chatter during the spin
  s.lCm = 9;
  for (int i = 0; i < 100 && s.nav.state() == State::TURNING_LEFT_90; ++i) {
    s.tick();
    CHECK(s.nav.state() == State::TURNING_LEFT_90 || s.nav.state() == State::SETTLE_AFTER_TURN);
  }
}

// ------------------------------------------------------------------ turn controller
static TurnStatus spinTurn(TurnController& tc, float& hdg, float target, float gain, uint32_t maxMs, uint32_t& t,
                           float rateSign = 1.0f) {
  tc.begin(target, hdg, fabsf(target - hdg), t);
  float rate = 0;
  for (uint32_t e = t + maxMs; t < e;) {
    t += 5;
    rate = rateSign * gain * (tc.cmdRight() - tc.cmdLeft()) * 0.5f;
    hdg += rate * 0.005f;
    const TurnStatus st = tc.update(hdg, rate, t);
    if (st == TurnStatus::Done || st == TurnStatus::Failed) return st;
  }
  return tc.status();
}
static void test_turn_90_and_180() {
  TurnController tc; float h = 0; uint32_t t = 0;
  CHECK(spinTurn(tc, h, 90, 180, 8000, t) == TurnStatus::Done);
  CHECK(fabsf(h - 90) <= kTurnTolDeg);
  CHECK(tc.cmdLeft() == 0 && tc.cmdRight() == 0);
  CHECK_NEAR(tc.measuredDeg(), 90, kTurnTolDeg);
  TurnController t2; float h2 = 0; uint32_t tt = 0;
  CHECK(spinTurn(t2, h2, -180, 180, 12000, tt) == TurnStatus::Done);
  CHECK(fabsf(h2 + 180) <= kTurnTolDeg);
  TurnController t3; float h3 = 0; uint32_t t3t = 0;
  CHECK(spinTurn(t3, h3, -90, 180, 8000, t3t) == TurnStatus::Done);
  CHECK(fabsf(h3 + 90) <= kTurnTolDeg);
}
static void test_turn_slows_near_target() {
  TurnController tc; tc.begin(90, 0, 90, 0);
  tc.update(0, 0, 5);
  const float far = tc.cmdRight();
  tc.update(80, 0, 10);
  const float near = tc.cmdRight();
  CHECK(far > near && near >= kTurnMinSpeed - 1e-6f);
}
static void test_turn_tolerance_boundary() {
  TurnController tc; tc.begin(90, 0, 90, 0);
  tc.update(90 - kTurnTolDeg - 0.1f, 0, 5);                      // just outside tolerance
  CHECK(tc.status() == TurnStatus::Rotating);
  TurnController u; u.begin(90, 0, 90, 0);
  u.update(90 - kTurnTolDeg + 0.1f, 0, 5);                       // just inside
  CHECK(u.status() == TurnStatus::Settling);
  u.update(90 - kTurnTolDeg + 0.1f, 0, 5 + kTurnSettleMs + 1);
  CHECK(u.status() == TurnStatus::Done);
}
static void test_turn_overshoot_is_corrected() {
  TurnController tc; tc.begin(90, 0, 90, 0);
  uint32_t t = 5;
  tc.update(97, 0, t);                                           // coasted past the target
  CHECK(tc.status() == TurnStatus::Rotating);
  CHECK(tc.cmdRight() < 0 && tc.cmdLeft() > 0);                  // rotates back (clockwise)
}
static void test_turn_failures() {
  TurnController a; float h = 0; uint32_t t = 0;
  CHECK(spinTurn(a, h, 90, 0.0f, 9000, t) == TurnStatus::Failed);   // motors do nothing -> stall
  CHECK(a.failReason() == TurnFail::Stall);
  TurnController b; float hb = 0; uint32_t tb = 0;
  CHECK(spinTurn(b, hb, 90, 180, 9000, tb, -1.0f) == TurnStatus::Failed);  // gyro sign wrong
  CHECK(b.failReason() == TurnFail::WrongWay);
  TurnController c; c.begin(90, 0, 90, 0);
  c.update(10, 600, 5);
  CHECK(c.status() == TurnStatus::Failed && c.failReason() == TurnFail::ImuRate);
  CHECK(c.cmdLeft() == 0 && c.cmdRight() == 0);
  TurnController d; d.begin(90, 0, 90, 0);
  d.update(10, 30, 5);
  d.update(20, 30, kTurnTimeoutMs90 + 10);
  CHECK(d.status() == TurnStatus::Failed && d.failReason() == TurnFail::Timeout);
}
static void test_turn_failure_is_bounded() {
  Sim s; s.settleFollow();
  s.plantYaw = false;                                            // robot never rotates (e.g. stuck wheels)
  s.fCm = 5; s.lCm = 60;                                         // front blocked, left open: keeps trying to turn left
  int turnEntries = 0;
  State prev = s.nav.state();
  for (int i = 0; i < 60000 / 5 && s.nav.state() != State::SAFE_STOP; ++i) {
    s.tick();
    const State st = s.nav.state();
    if (st == State::TURNING_LEFT_90 && prev != State::TURNING_LEFT_90) ++turnEntries;
    prev = st;
  }
  CHECK(s.nav.state() == State::SAFE_STOP);                      // bounded retries, then stop
  CHECK(turnEntries >= 1 && turnEntries <= kMaxTurnsPerEpisode);
  CHECK(s.out.left == 0 && s.out.right == 0);
}


// ------------------------------------------------------------------ floor colour
static void test_color_classifier() {
  const ColorRefs refs = defaultColorRefs();
  auto cls = [&](const uint16_t* v) { return classifyColor({v[0], v[1], v[2], v[3]}, refs); };
  CHECK(cls(kRefBlack) == ColorClass::Black);
  CHECK(cls(kRefSilver) == ColorClass::Silver);
  CHECK(cls(kRefBlue) == ColorClass::Blue);
  CHECK(cls(kRefRed) == ColorClass::Red);
  CHECK(cls(kRefFloor) == ColorClass::Other);
  const uint16_t noisyBlue[4] = {1575, 3400, 6300, 10800};       // roughly +-5 %
  CHECK(cls(noisyBlue) == ColorClass::Blue);
  const uint16_t whiteFloor[4] = {14500, 14200, 14000, 52000};
  CHECK(cls(whiteFloor) == ColorClass::Other);                   // bright white tile is not silver
  const uint16_t green[4] = {1500, 6000, 1800, 9500};
  CHECK(cls(green) == ColorClass::Other);                        // unlisted colours are normal floor
  CHECK(classifyColor({0, 0, 0, 0}, refs) == ColorClass::Other); // dead sensor never looks like a colour
}
static void test_color_debounce() {
  Sim s; s.settleFollow();
  s.cKind = ColorClass::Black;
  s.run(kColorPollMs * (kColorConfirmSamples - 1) - 5);
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);            // fewer than N samples: not confirmed
  s.cKind = ColorClass::Other; s.run(200);
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
}
static void test_black_hole_backs_off_and_turns() {
  Sim s; s.settleFollow();
  s.cKind = ColorClass::Black;
  CHECK(s.runUntil(State::HAZARD_BACKOFF, 500));
  CHECK(s.out.left == 0 && s.out.right == 0);                    // stops first
  bool reversed = false;
  while (s.nav.state() == State::HAZARD_BACKOFF && s.t < 20000) { s.tick(); if (s.out.left < 0 && s.out.right < 0) reversed = true; }
  CHECK(reversed);
  CHECK(s.nav.state() == State::OBSTACLE_STOP);
  s.cKind = ColorClass::Other;                                   // sensor is off the hole again after backing up
  bool turnedRight = false, resumedEarly = false;
  for (int i = 0; i < 12000 / 5 && s.nav.state() != State::FORWARD_WALL_FOLLOW; ++i) {
    s.tick();
    if (s.nav.state() == State::TURNING_RIGHT_90) turnedRight = true;
  }
  if (!turnedRight) resumedEarly = true;
  CHECK(turnedRight && !resumedEarly);                           // never drives back into the hole
  CHECK_NEAR(s.heading - s.nav.headingRefDeg(), 0.0, kTurnTolDeg + 0.5);
  CHECK(s.nav.headingRefDeg() < -80);
}
static void test_black_hole_left_open_turns_left() {
  Sim s; s.settleFollow();
  s.lCm = 60;
  s.cKind = ColorClass::Black;
  CHECK(s.runUntil(State::HAZARD_BACKOFF, 800));
  s.cKind = ColorClass::Other;
  CHECK(s.runUntil(State::TURNING_LEFT_90, 4000));
}
static void test_black_after_turn_is_detected() {
  Sim s; s.settleFollow();
  s.fCm = 5; s.lCm = 60;
  CHECK(s.runUntil(State::TURNING_LEFT_90, 3000));
  s.cKind = ColorClass::Black; s.fCm = 100; s.lCm = 9;           // new heading faces a hole
  CHECK(s.runUntil(State::HAZARD_BACKOFF, 8000));
}
static void test_black_everywhere_ends_in_safe_stop() {
  Sim s; s.settleFollow();
  s.cKind = ColorClass::Black;
  CHECK(s.runUntil(State::SAFE_STOP, 60000));
  CHECK(s.out.left == 0 && s.out.right == 0);
}
static void test_silver_saves_checkpoint_once() {
  Sim s; s.settleFollow();
  s.run(600);
  s.cKind = ColorClass::Silver;
  s.run(2000);                                                   // sitting on silver for 2 s
  CHECK(s.nav.checkpointCount() == 1);
  Checkpoint cp;
  CHECK(s.nav.takeCheckpoint(cp) && cp.id == 1);
  CHECK(!s.nav.takeCheckpoint(cp));
  CHECK(cp.x > 0);                                               // dead-reckoned forward progress
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW && s.out.left > 0);   // checkpoint does not stop the robot
  s.cKind = ColorClass::Other; s.run(200);
  s.cKind = ColorClass::Silver; s.run(300);
  CHECK(s.nav.checkpointCount() == 2);
}
static void test_red_marks_danger_zone() {
  Sim s; s.settleFollow();
  CHECK(!s.nav.inDangerZone());
  s.cKind = ColorClass::Red; s.run(1500);
  CHECK(s.nav.inDangerZone() && s.nav.dangerEntrances() == 1);
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
}
static void test_blue_waits_five_consecutive_seconds() {
  Sim s; s.settleFollow();
  s.cKind = ColorClass::Blue;
  CHECK(s.runUntil(State::BLUE_ADVANCE, 500));
  CHECK(s.runUntil(State::BLUE_WAIT, 1500));
  const uint32_t t0 = s.t;
  s.run(2500);
  CHECK(s.nav.state() == State::BLUE_WAIT && s.out.left == 0 && s.out.right == 0);
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 4000));
  CHECK(s.t - t0 >= kBlueWaitMs - 10 && s.t - t0 <= kBlueWaitMs + 200);
  CHECK(s.nav.blueTileCount() == 1 && s.nav.blueTile(0).visits == 1 && s.nav.blueTile(0).completed == 1);
  s.run(2000);                                                   // still on blue: must not trigger again
  CHECK(s.nav.blueVisits() == 1 && s.nav.state() == State::FORWARD_WALL_FOLLOW);
}
static void test_blue_tiles_tracked_separately() {
  Sim s; s.settleFollow();
  s.cKind = ColorClass::Blue;
  CHECK(s.runUntil(State::BLUE_WAIT, 1500));
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 7000));
  s.cKind = ColorClass::Other; s.run(3500);                      // drive well over a tile length
  s.cKind = ColorClass::Blue;
  CHECK(s.runUntil(State::BLUE_WAIT, 1500));
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 7000));
  CHECK(s.nav.blueTileCount() == 2 && s.nav.blueCompleted() == 2);
  CHECK(s.nav.blueTile(0).visits == 1 && s.nav.blueTile(1).visits == 1);
  CHECK(s.nav.blueTile(0).cx != s.nav.blueTile(1).cx || s.nav.blueTile(0).cy != s.nav.blueTile(1).cy);
}
static void test_blue_short_dropout_tolerated_long_dropout_restarts() {
  Sim s; s.settleFollow();
  s.cKind = ColorClass::Blue;
  CHECK(s.runUntil(State::BLUE_WAIT, 1500));
  const uint32_t t0 = s.t;
  s.run(1000);
  s.cKind = ColorClass::Other; s.run(150);                       // brief sensor flicker
  s.cKind = ColorClass::Blue;
  CHECK(s.runUntil(State::FORWARD_WALL_FOLLOW, 6000));
  CHECK(s.t - t0 <= kBlueWaitMs + 200);                          // timer was NOT restarted

  Sim g; g.settleFollow();
  g.cKind = ColorClass::Blue;
  CHECK(g.runUntil(State::BLUE_WAIT, 1500));
  g.cKind = ColorClass::Other;                                   // blue really gone
  CHECK(g.runUntil(State::FORWARD_WALL_FOLLOW, 6000));           // bounded: abandons the wait
  CHECK(g.nav.blueCompleted() == 0 && g.nav.blueVisits() == 1);
}
static void test_colour_sensor_failure_stops() {
  Sim s; s.settleFollow();
  s.colourOk = false;
  s.tick();
  CHECK(s.nav.state() == State::SENSOR_FAULT && s.out.left == 0 && s.out.right == 0);
  Sim b; b.colourOk = false;                                     // missing at boot
  b.run(9000);
  CHECK(b.nav.state() == State::SENSOR_FAULT && b.out.left == 0);
}

// ------------------------------------------------------------------ closed loop wall following
static void test_wall_follow_closed_loop() {
  Sim s; s.plantYaw = true;
  float d = 15.0f;                                               // lateral distance to wall, cm
  float theta = 0;                                               // heading relative to wall direction (deg)
  float minD = 99, maxAfter = 0, minAfter = 99;
  s.lCm = d;
  s.toFollow();
  const float refHead = s.heading;
  for (uint32_t i = 0; i < 40000 / 5; ++i) {                     // 40 s
    const float v = 40.0f * 0.5f * (s.out.left + s.out.right);   // cm/s
    theta = s.heading - refHead;
    d -= v * sinf(theta * kDegToRad) * 0.005f;
    s.lCm = d / cosf(theta * kDegToRad);
    s.tick();
    if (s.nav.state() != State::FORWARD_WALL_FOLLOW) break;
    if (i > 25000 / 5) { if (d > maxAfter) maxAfter = d; if (d < minAfter) minAfter = d; }
    if (d < minD) minD = d;
  }
  CHECK(s.nav.state() == State::FORWARD_WALL_FOLLOW);
  CHECK(minD > 5.0f);                                            // never crashes into the wall
  CHECK_NEAR(0.5f * (maxAfter + minAfter), kWallTargetCm, 1.0);  // settles on 9 cm
  CHECK(maxAfter - minAfter < 2.0f);                             // no sustained oscillation
  printf("  closed-loop: min %.2f cm, final band %.2f..%.2f cm\n", minD, minAfter, maxAfter);
}

int main() {
  RUN(test_range_filter);
  RUN(test_thresholds);
  RUN(test_wall_controller);
  RUN(test_startup_gating);
  RUN(test_startup_failures);
  RUN(test_imu_failure_stops_motors);
  RUN(test_range_sensor_faults);
  RUN(test_intermittent_bad_reading_not_fatal);
  RUN(test_wall_lost_bounded);
  RUN(test_opening_confirmation);
  RUN(test_opening_hysteresis_band);
  RUN(test_no_second_turn_without_wall);
  RUN(test_left_turn_completes_and_resumes);
  RUN(test_front_stop_boundary);
  RUN(test_front_false_alarm_resumes);
  RUN(test_front_blocked_left_open_turns_left);
  RUN(test_front_blocked_left_blocked_turns_right);
  RUN(test_dead_end_u_turn_then_stop);
  RUN(test_dead_end_u_turn_escape);
  RUN(test_left_unknown_never_guesses);
  RUN(test_no_overlapping_turns);
  RUN(test_turn_90_and_180);
  RUN(test_turn_slows_near_target);
  RUN(test_turn_tolerance_boundary);
  RUN(test_turn_overshoot_is_corrected);
  RUN(test_turn_failures);
  RUN(test_turn_failure_is_bounded);
  RUN(test_color_classifier);
  RUN(test_color_debounce);
  RUN(test_black_hole_backs_off_and_turns);
  RUN(test_black_hole_left_open_turns_left);
  RUN(test_black_after_turn_is_detected);
  RUN(test_black_everywhere_ends_in_safe_stop);
  RUN(test_silver_saves_checkpoint_once);
  RUN(test_red_marks_danger_zone);
  RUN(test_blue_waits_five_consecutive_seconds);
  RUN(test_blue_tiles_tracked_separately);
  RUN(test_blue_short_dropout_tolerated_long_dropout_restarts);
  RUN(test_colour_sensor_failure_stops);
  RUN(test_wall_follow_closed_loop);
  printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
