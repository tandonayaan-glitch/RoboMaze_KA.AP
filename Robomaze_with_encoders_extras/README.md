# Robomaze_with_encoders - wall follower + TCS34725 floor colours + wheel encoders (2x L298N, 4 motors)

Everything from the colour version (`Robomaze_colour_sensors`): left-wall following at 9 cm, 7 cm front stop, gyro 90 / 180 deg
turns, left-opening detection, dead-end U-turn, fault handling, and the floor-colour rules (black hole, silver
checkpoint, blue 5 s wait with per-tile visit counts, red Dangerous Zone entrance) - **plus wheel encoders**.
**Sketch folder = `Robomaze_with_encoders/` (code only; import this into the Arduino IDE).** This README and `test/`
live next to it so they never interfere with the sketch.

**Status:** the sketch compiles to object code with `--warnings all`, but it has NOT been linked, unit tested, or run on
a robot (the development PC blocked the ESP32 archiver and the host assembler through a Windows Application Control
policy). The unit tests type-check but were never executed. Colour references and encoder constants are placeholders
that you must calibrate.

## What the encoders add
| Feature | Behaviour |
|---|---|
| Odometry | Distance = average of both wheel encoders, integrated along the gyro heading -> estimated (x, y) in cm. Used for silver checkpoints and for telling blue tiles apart (`kTileCm` cells). Much better than the speed-model guess of the colour version, but still drifts with wheel slip and gyro drift. |
| Distance-based moves | The creep before a left turn (`kOpeningAdvanceCm`), the creep onto a blue tile (`kBlueAdvanceCm`) and the reverse away from a black hole (`kBackoffCm`) now use travelled distance, not time. Each has a timeout (`kAdvanceTimeoutMs`). |
| Wheel-stall / encoder-failure fault | If a side is commanded (average > 0.25) but moves < 1 cm over 1.5 s, the robot enters `SENSOR_FAULT` and stops. Catches a jammed wheel, a wire that came off, or a dead encoder. |
| Fallback | Set `kEncodersEnabled = false` in `config.h` and it reverts to the time-based moves and speed-model dead reckoning of the colour version. |

Encoders are **not** used for wheel-speed control: steering still comes from the gyro heading loop and the wall-distance loop. Wheel-speed closed loop and encoder/gyro cross-checks during turns are possible future work.

## Encoder wiring (assumed: one single-channel encoder per side)
| Encoder | ESP32 GPIO | Notes |
|---|---|---|
| Left side wheel/motor | 36 (VP) | input-only |
| Right side wheel/motor | 39 (VN) | input-only |

* Power the encoders from 3V3 (or 5 V with a 1k/2k divider on the signal). GPIO36/39 have no internal pull-up: use push-pull Hall modules or add a 10 kOhm pull-up to 3V3 for open-collector outputs.
* Every other usable GPIO is already taken, so this design has **two** encoders (one left, one right, e.g. on the front motors). Four encoders would need multiplexing or an I/O expander.
* Single channel means **direction is inferred from the commanded direction** (the ISR uses the sign of the last motor command). Ticks while the wheel is being pushed backwards by hand are counted as forward. A quadrature encoder (A/B) would remove this limitation but needs twice the pins.
* Change pins in `config.h` (`kPinEncLeft`, `kPinEncRight`); the `static_assert` requires input-only pins.

## Encoder calibration (mandatory)
1. Measure the wheel diameter and set `kWheelDiameterCm`.
2. Set `BENCH_ENCODER_MONITOR 1` in `config.h`, upload, open the Serial Monitor (115200). For 20 s it prints tick counts; turn each wheel exactly one revolution by hand. The tick count for one revolution is `kEncPulsesPerRev` (after the gearbox). Set it, then set the flag back to 0.
3. Drive check: `BENCH_MOTOR_TEST 1` also prints encoder ticks after each motor spin (wheels off the ground); both sides must count up. Then run 1 m on the floor and compare `odo=` in the status line with a ruler; adjust `kWheelDiameterCm`.
4. Tune `kOpeningAdvanceCm`, `kBlueAdvanceCm`, `kBackoffCm` on the floor.

## Colour rules (unchanged from the colour version)
| Floor colour | Behaviour |
|---|---|
| BLACK (hole) | Stop, reverse `kBackoffCm`, treat that direction as blocked: turn left if the left side is open, else right, else U-turn, else `SAFE_STOP`. Re-checked after every turn. Max 4 per episode. |
| SILVER | Save estimated position + heading + counter (RAM and flash), print an `EVENT`, keep driving. Once per tile. |
| BLUE | Creep `kBlueAdvanceCm`, stop, wait **5 s consecutively** (restarts if blue is lost > 0.4 s, gives up after 3 restarts). Visits / completions counted per blue tile. |
| RED | Flag the Dangerous Zone entrance, count it, optional slow-down (`kDangerSpeedScale`, default 1.0). |
| Other | Normal floor. |

Assumptions to confirm: blue is waited on every visit (`kBlueWaitEveryVisit`); "individual blue tile" = odometry cell of size `kTileCm` (30 cm); nothing uses the saved checkpoint yet; nothing special happens in the Dangerous Zone beyond the flag; colours are ignored while spinning in a turn.

## Colour sensor wiring and calibration
TCS34725 on the shared I2C bus (SDA 21, SCL 22, address 0x29, VIN 3V3, LED pin to 3V3). Mount at the front, 3-5 mm above the floor, pointing down, shielded from ambient light, ahead of the front axle so it reaches a hole edge before the wheels.
Calibrate the reference colours (mandatory): reset the board, send `c` within 5 s of `Send 'c' ...`, place the sensor at mounting height over a tile and send `B` black, `S` silver, `U` blue, `R` red, `F` normal floor (values are stored in flash); `Z` resets, `Q` quits. Verify each tile prints the right class.

## Complete pin table (proposed - edit `config.h` if your wiring differs)
| Function | GPIO |
|---|---|
| MPU6050 + TCS34725 (I2C) SDA / SCL | 21 / 22 |
| Front HC-SR04 TRIG / ECHO | 5 / 34 |
| Left HC-SR04 TRIG / ECHO | 14 / 35 |
| L298N #1 ENA / IN1 / IN2 (front-left) | 25 / 26 / 27 |
| L298N #1 ENB / IN3 / IN4 (front-right) | 33 / 32 / 13 |
| L298N #2 ENA / IN1 / IN2 (rear-left) | 16 / 17 / 18 |
| L298N #2 ENB / IN3 / IN4 (rear-right) | 19 / 23 / 4 |
| Left encoder / right encoder | 36 / 39 |

Echo pins need a 1k/2k divider (5 V HC-SR04). Remove the L298N ENA/ENB jumpers, add 10k pull-downs on the four EN pins, share all grounds, and never power motors from a GPIO. Base wiring notes and the "what to change if your wiring differs" guide (including a ready-to-paste ChatGPT prompt) are in the base project README (`Robomaze_2026`).

## Build and upload (Arduino CLI)
```
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --warnings all Robomaze_with_encoders
arduino-cli upload  --fqbn esp32:esp32:esp32doit-devkit-v1 -p COM3 Robomaze_with_encoders
arduino-cli monitor -p COM3 -c baudrate=115200
```
No extra libraries (`Wire` and `Preferences` ship with the ESP32 core; sensors are driven directly). If the upload hangs at "Connecting...", hold BOOT. Flash with the wheels off the ground. First power-up: bench motor test, encoder monitor, gyro sign check (rotate left by hand, `hdg=` must rise), keep the robot still during the 1.5 s gyro calibration. `x` in the serial monitor = latched stop.

## Host unit tests
Needs any g++. Run from the folder that contains both `Robomaze_with_encoders/` and this extras folder:
```
bash Robomaze_with_encoders_extras/test/run_tests.sh
```
Expect `N checks passed, 0 failed`. They cover thresholds, filtering, opening logic, obstacle decisions, turns, faults, colours, and the encoder features (odometry, stall / dead-encoder faults, distance-based advance and back-off). If any fail, do not flash; report the failing line.

## Serial diagnostics
4 Hz status line: state, F/L distances, wall error, heading / reference, gyro rate, motor commands, `col=<class>(R,G,B,C)`, `pos=(x,y)`, `odo=<left>/<right> cm`, `DANGER` after red. `EVENT ...` lines for hazards, checkpoints, blue waits, danger zone.

## Known limitations
* Only two encoders (one per side), single channel: direction is inferred from the motor command; position is an estimate that drifts with wheel slip (skid steering slips a lot in turns - turns use the gyro, not the encoders).
* No right-side or rear sensor: right turns, U-turns and the blind reverse away from a hole are not proven clear.
* Heading is gyro-integrated (no magnetometer) and drifts.
* A colour sensor or encoder fault stops the robot; faults are latched until power cycle.
* Nothing here has been run on hardware; all thresholds and speeds need tuning on the real robot.
