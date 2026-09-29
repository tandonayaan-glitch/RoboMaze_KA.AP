# 1_l298n - ESP32 left-wall-following robot (one L298N)

Self-contained project (no shared files with `2_l298n`). Board: **ESP32 DevKit V1**, Arduino-ESP32 core 3.3.x.
No external Arduino libraries: the MPU6050 is driven directly over `Wire`.

## Read first: what is verified and what is proposed
No wiring documentation or existing robot code was found on the development machine, so **the pin table below is a
PROPOSED wiring, not a discovery**. Wire the robot to match it, or edit the pins in `config.h` (the `static_assert`s
reject GPIOs that are flash pins, input-only pins used as outputs, or boot-strapping hazards). The motor arrangement
comes from the project request: **one L298N, channel A = left side, channel B = right side** (two motors, or two per
side wired in parallel). Encoders are assumed **not** to exist and nothing here uses them.

## Wiring (proposed)
| Function | ESP32 GPIO | Notes |
|---|---|---|
| MPU6050 SDA / SCL | 21 / 22 | 3.3 V supply, AD0 low -> address 0x68 |
| Front HC-SR04 TRIG / ECHO | 5 / 34 | ECHO through divider |
| Left HC-SR04 TRIG / ECHO | 14 / 35 | ECHO through divider |
| L298N ENA (left PWM) / IN1 / IN2 | 25 / 26 / 27 | remove the ENA jumper |
| L298N ENB (right PWM) / IN3 / IN4 | 33 / 32 / 13 | remove the ENB jumper |

* **Echo divider (mandatory):** HC-SR04 Echo is 5 V. ECHO -> 1 kOhm -> GPIO -> 2 kOhm -> GND (about 3.3 V). GPIO34/35 are input-only with no pull-down, so the 2 kOhm also stops the line floating.
* HC-SR04 VCC = 5 V, common GND. Left sensor points straight left and level; front sensor straight ahead.
* Add 10 kOhm pull-downs from ENA/ENB to GND so the driver stays off while the ESP32 boots.
* Power: motor battery -> L298N 12 V terminal; ESP32 from its own regulated 5 V. **All grounds common** (battery, L298N, ESP32, sensors). Never drive a motor from a GPIO. The L298N drops roughly 1.4-2 V, so the motors see less than the battery voltage.
* MPU6050: mount flat, Z axis up, rigid, close to the centre of rotation.

## Build and upload (Arduino CLI)
```
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --warnings all 1_l298n
arduino-cli upload  --fqbn esp32:esp32:esp32doit-devkit-v1 -p COM<N> 1_l298n
arduino-cli monitor -p COM<N> -c baudrate=115200
```
`sketch.yaml` already sets the FQBN. **Flash only with the wheels off the ground.**

## Host unit tests
```
bash 1_l298n/test/run_tests.sh      # needs any g++ on PATH
```
They cover the 7 cm / 9 cm thresholds, range filtering, invalid readings, opening confirmation and hysteresis,
left / right / 180 decisions, 90 and 180 degree turn completion, tolerance boundary, overshoot correction,
stall / wrong-way / timeout failures, sensor and IMU faults, bounded retries, and a closed-loop wall-following simulation.

## First power-up procedure
1. **Bench motor test**: set `BENCH_MOTOR_TEST 1` in `config.h`, wheels off the ground, upload. Each motor spins forward for 1 s and its name is printed. If a wheel spins backward set `kInvertLeft` / `kInvertRight`. Set the flag back to `0`.
2. **Gyro sign check**: open the Serial Monitor at 115200 and rotate the still robot by hand counter-clockwise (left, seen from above). `hdg=` must increase; otherwise set `kGyroZSign = -1`. A wrong sign is also caught at run time (the turn fails with `WrongWay` and the robot stops).
3. **Calibration**: after a 3 s start delay the gyro bias is measured for about 1.5 s. **Keep the robot completely still.** The robot never calibrates while moving. If the noise check fails 3 times it enters `SENSOR_FAULT` and does not move.
4. Place the robot parallel to the left wall at about 9 cm with a surface within 2 m in front. Startup needs valid echoes from both sensors, otherwise `SENSOR_FAULT`.
5. Send `x` in the serial monitor at any time for a latched stop.

## Navigation state machine
`INITIALIZING` -> `IMU_CALIBRATION` -> `SENSOR_CHECK` -> `FORWARD_WALL_FOLLOW`, then:

| State | Behaviour |
|---|---|
| FORWARD_WALL_FOLLOW | Cascade: the wall-distance loop (P+D, deadband, clamp) sets a heading *offset* (max 12 deg); the gyro heading loop tracks reference + offset, so the two loops cannot fight. Speed ramps down in the last 30 cm before the front threshold. No usable wall: slow, heading hold only, bounded to 4 s, then `SAFE_STOP`. |
| LEFT_OPENING_CONFIRMATION | Slow. Needs 4 consecutive open samples (no echo, or above 25 cm) after a previously established wall. Seeing the wall again (below 18 cm) aborts. |
| LEFT_OPENING_ADVANCE | Creeps `kOpeningAdvanceMs` so the pivot lines up with the opening, then requires the opening to still be open. |
| OBSTACLE_STOP | A front reading of 7 cm or less stops at once (brake). After 2 fresh readings from each sensor: false alarm -> resume; left open -> turn left; left blocked -> turn right; left unknown -> wait, then `SAFE_STOP`. |
| TURNING_LEFT_90 / RIGHT_90 / AROUND_180 | Spin in place to an absolute target (reference +/- 90), so tolerance never accumulates. Speed falls with remaining angle, has a minimum-speed floor, and ends with stop, settle, verify and bounded corrections. AROUND_180 is the second 90 deg right turn after a right turn found the front still blocked (net 180). |
| SETTLE_AFTER_TURN | Stopped. Needs fresh readings from both sensors before deciding: front blocked again -> next escalation, otherwise wall following. |
| OBSTACLE_RECOVERY | Pause after a failed turn. At most 2 recoveries and 3 turns per episode, then `SAFE_STOP`. |
| SENSOR_FAULT / SAFE_STOP | Latched, motors off with brake. Reset = power cycle. |

Front avoidance outranks opening detection and wall following. A turn is never interrupted or restarted by sensor readings.

## Tuning guide (all in `config.h`)
* **Wall following**: twitchy -> lower `kWallKp` or raise `kWallDeadbandCm`; slow to converge -> raise `kWallKp`; ringing -> raise `kWallKd`. Steady-state error sits inside the deadband (default +/-0.7 cm); reduce it for a tighter 9 cm at the cost of more steering activity.
* **Heading loop**: `kHeadKp` (snappiness), `kHeadKd` (damping), `kMaxSteer`. If the robot snakes, lower `kHeadKp` and raise `kHeadKd`.
* **Motors**: find `kPwmMin` (lowest duty that starts both sides on the floor) and `kPwmMax`; then `kCruiseSpeed`, `kSlowSpeed`, `kAccelPerSec`, `kDecelPerSec`.
* **Turns**: `kTurnKp`, `kTurnMinSpeed` (too low stalls, too high overshoots), `kTurnTolDeg` (3-5 deg), `kTurnSettleMs`. Run 90 deg turns ten times and compare the printed `meas=` with 90.
* **Openings**: `kOpenEnterCm` / `kOpenExitCm` (keep a gap for hysteresis), `kOpenConfirmSamples`, and above all **`kOpeningAdvanceMs`**: increase until the robot pivots at the right point in the opening.
* **Sensors**: `kSensorSlotMs` (must exceed the echo timeout, enforced at compile time), `kMaxRangeCm`, `kFrontNoEchoFaultCount` (raise for very open rooms).

## Serial diagnostics (4 Hz, never blocks control)
`t= STATE F=<front cm> L=<left cm> tgt=9 err=<wall error> hdg=<heading> ref=<reference> off=<wall offset> w=<deg/s> cmd=<L>/<R>` plus `turn[tgt meas rem corr]` while turning. `(x)` after a distance means the last echo was missing or invalid. State changes and fault reasons print as `STATE -> ...`.

## Known limitations
* Only front and left sensors exist: **the right side, the swing of a turn, and the path through an opening are never measured**. Right and 180 deg turns are conservative, in place, gyro-checked and stall-monitored, but the route is not proven clear. A missing left echo may be an opening, a slanted or soft wall, or a sensor fault; it is only trusted after repeated samples following an established wall.
* The MPU6050 has no magnetometer: heading is relative and drifts. Turn accuracy needs physical calibration and testing. Without encoders, speed and travelled distance are open loop (`kOpeningAdvanceMs` is time based); wheel slip shows up only as a stalled turn.
* The wall is assumed parallel to the starting heading, so start parallel to the wall.
* Each HC-SR04 is sampled at about 16 Hz. Braking distance at cruise speed is not zero: validate the 7 cm stop on the real robot and lower `kCruiseSpeed` or raise `kFrontStopCm` if it bumps.
* Front "no echo" for about 2.7 s is treated as a fault, so large open rooms will stop the robot; tune `kFrontNoEchoFaultCount`.
* Faults are latched until power cycle by design.

## Verification status
The sketch compiles to object code with `--warnings all` and no warnings. The final link step and the host unit tests could not be executed on the development machine because a Windows Application Control policy blocked the archiver and assembler executables. The tests type-check but have not been run. **Nothing has been run on hardware.**

Remaining hardware tests: motor direction, gyro sign, calibration noise, sensor readings against a ruler, wall following at 9 cm, 90 / 180 deg accuracy, front stop distance, opening advance distance, and fault injection (unplug each sensor while stationary).
