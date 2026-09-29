// 1_l298n - ESP32 DevKit V1 left-wall-following robot, ONE L298N (left channel A, right channel B).
// Sensors: 2x HC-SR04 (front, left), MPU6050 gyro (Z axis). No encoders are assumed.
// Logic lives in nav_core.h (host-tested); this file is the hardware layer.
#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "config.h"
#include "nav_core.h"
#include "hw_types.h"

using namespace cfg;
using nav::ImuCal;
using nav::RangeKind;
using nav::State;

// ================================ Motors ================================
struct Motor { uint8_t en, in1, in2; bool leftSide; bool invert; const char* name; };
static const Motor kMotors[] = {
    {kPinD1EnA, kPinD1In1, kPinD1In2, true, kInvertLeft, "LEFT"},
    {kPinD1EnB, kPinD1In3, kPinD1In4, false, kInvertRight, "RIGHT"},
};
constexpr size_t kMotorCount = sizeof(kMotors) / sizeof(kMotors[0]);

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

static void driveMotors(uint32_t nowMs) {
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

#if BENCH_MOTOR_TEST
static void benchMotorTest() {
  Serial.println("BENCH MOTOR TEST: wheels OFF the ground. Each motor: 1 s forward (should spin FORWARD).");
  for (size_t i = 0; i < kMotorCount; ++i) {
    Serial.printf("Motor %s...\n", kMotors[i].name);
    writeMotor(i, 0.6f, millis());
    delay(1000);
    coastMotor(i);
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

// ================================ Main ================================
static nav::Navigator navigator;
static State lastPrintedState = State::INITIALIZING;
static uint32_t lastDiagMs = 0;

static void reportDiagnostics(uint32_t nowMs, const nav::NavOutputs& o) {
  if (navigator.state() != lastPrintedState) {
    lastPrintedState = navigator.state();
    Serial.printf("[%lu] STATE -> %s %s\n", (unsigned long)nowMs, nav::stateName(lastPrintedState), navigator.reason());
  }
  if (nowMs - lastDiagMs < kDiagPeriodMs) return;
  lastDiagMs = nowMs;
  if (Serial.availableForWrite() < 220) return;       // never let logging block control
  const nav::TurnController& t = navigator.turn();
  const bool turning = o.state == State::TURNING_LEFT_90 || o.state == State::TURNING_RIGHT_90 ||
                       o.state == State::TURNING_AROUND_180;
  Serial.printf("t=%lu %s F=%.1f%s L=%.1f%s tgt=%.1f err=%.1f hdg=%.1f ref=%.1f off=%.1f w=%.1f cmd=%.2f/%.2f",
                (unsigned long)nowMs, nav::stateName(o.state), filtFront.rawCm(),
                filtFront.lastKind() == RangeKind::Valid ? "" : "(x)", filtLeft.filteredCm(),
                filtLeft.lastKind() == RangeKind::Valid ? "" : "(x)", kWallTargetCm, navigator.wallErrorCm(),
                imu::heading, navigator.headingRefDeg(), navigator.wallOffsetDeg(), imu::rate, o.left, o.right);
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
  Serial.println("\n1_l298n wall follower booting");
#if BENCH_MOTOR_TEST
  benchMotorTest();
#endif
  initUltrasonic(usFront);                             // 4. ultrasonic sensors
  initUltrasonic(usLeft);
  if (!imu::begin()) Serial.println("IMU init FAILED - robot will not move");   // 3. I2C + MPU6050
  slotStartUs = micros();
}

void loop() {
  const uint32_t nowMs = millis();
  if (Serial.available() && Serial.read() == 'x') navigator.requestStop("serial stop command");

  if (navigator.state() == State::IMU_CALIBRATION) imu::calibrationStep(nowMs);   // stationary only
  else imu::service(nowMs);
  ultrasonicService(nowMs);

  nav::NavInputs in;
  in.nowMs = nowMs;
  in.front = &filtFront;
  in.left = &filtLeft;
  in.headingDeg = imu::heading;
  in.rateDps = imu::rate;
  in.imuOk = imu::healthy(nowMs) || navigator.state() == State::INITIALIZING ||
             navigator.state() == State::IMU_CALIBRATION || navigator.state() == State::SENSOR_CHECK;
  in.imuCal = imu::cal;
  if (!imu::ok && in.imuCal == ImuCal::Pending) in.imuCal = ImuCal::Failed;

  const nav::NavOutputs out = navigator.update(in);
  applyOutputs(out, nowMs);
  reportDiagnostics(nowMs, out);
  delay(1);
}
