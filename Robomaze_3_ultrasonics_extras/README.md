# Robomaze_3_ultrasonics - wall follower + TCS34725 floor colours + THIRD (right) ultrasonic, no encoders (2x L298N, 4 motors)

Everything from the colour version (`Robomaze_colour_sensors`): left-wall following at 9 cm, 7 cm front stop, gyro turns,
left-opening detection, fault handling, and the floor-colour rules (black hole, silver checkpoint, blue 5 s wait with
visit counts, red Dangerous Zone entrance). **No wheel encoders.** New: a **right-facing HC-SR04** so right turns and
U-turns are checked instead of being blind.
**Sketch folder = `Robomaze_3_ultrasonics/` (code only; import this into the Arduino IDE).** This README and `test/` live
next to it so they never interfere with the sketch.

**Status:** the sketch compiles to object code with `--warnings all`, but it has NOT been linked, unit tested, or run on a robot
(the development PC blocked the ESP32 archiver and the host assembler through a Windows Application Control policy).
The unit tests type-check but were never executed. Colour references are placeholders you must calibrate.

## What the third ultrasonic changes
| Situation | Base / colour version | This version |
|---|---|---|
| Front blocked (or black hole), left open | turn left | turn left (left is still preferred) |
| Front blocked, left blocked | turn right **blind**, then check | turn right **only if the right sensor confirms it is open** (2 fresh open readings) |
| Front, left and right all blocked | right turn, then a second 90 deg | **straight to a 180 deg U-turn** (net 180, clockwise) |
| Right reading missing / implausible | n/a | never guesses: waits, then `SAFE_STOP` (bounded) |
| Startup | needs front + left | also needs **valid right echoes**, so a missing right sensor is caught before moving |
| Fault | front/left implausible | also right implausible = `SENSOR_FAULT` |

After a U-turn the robot re-checks the front; if it is still blocked there is no safe move and it stops. The right sensor is
not used for wall following. Sensor timing: slots of 20 ms in the order front, left, front, right (front every 40 ms,
left and right every 80 ms), always at least one echo timeout apart so the sensors never cross-talk.

Limitation kept: "no echo" on the right counts as open (like on the left). A right sensor that comes unplugged *after*
startup would read as open. Also, the clearance of the swing of the spin is not measured, only the side and front distances.

## Right sensor wiring
| Function | GPIO | Notes |
|---|---|---|
| Right HC-SR04 TRIG | 15 | strapping pin (only affects the boot log); fine as a trigger output |
| Right HC-SR04 ECHO | 36 (VP) | input-only; **ECHO -> 1k -> GPIO36 -> 2k -> GND** (5 V echo) |

Mount it pointing straight right, level, mirrored to the left sensor. GPIO15 is the only spare output-capable pin left in
this design (all other usable GPIOs are already taken), so there is no room for a fourth sensor without an expander.
Change pins in `config.h` (`kPinRightTrig`, `kPinRightEcho`); the `static_assert`s reject illegal pins.

## Colour rules (unchanged from the colour version)
| Floor colour | Behaviour |
|---|---|
| BLACK (hole) | Stop, reverse `kBackoffMs` (time-based, blind: no rear sensor and no encoders), treat that direction as blocked and use the left / right / U-turn decision above. Re-checked after every turn. Max 4 per episode. |
| SILVER | Save estimated position + heading + counter (RAM and flash), print an `EVENT`, keep driving. Once per tile. |
| BLUE | Creep `kBlueAdvanceMs`, stop, wait **5 s consecutively** (restarts if blue lost > 0.4 s, gives up after 3 restarts). Visits / completions counted per blue tile. |
| RED | Flag the Dangerous Zone entrance, count it, optional slow-down (`kDangerSpeedScale`, default 1.0). |
| Other | Normal floor. |

Assumptions to confirm: blue is waited on every visit (`kBlueWaitEveryVisit`); "individual blue tile" = cell of the
**speed-model dead-reckoned position** (`kTileCm` = 30 cm; no encoders, so this drifts and calibration of
`kCmPerSecFullSpeed` matters); nothing uses the saved checkpoint yet; nothing special happens in the Dangerous Zone beyond
the flag; colours are ignored while spinning.

## Colour sensor wiring and calibration
TCS34725 on the shared I2C bus (SDA 21, SCL 22, address 0x29, VIN 3V3, LED pin to 3V3). Mount at the front, 3-5 mm above the
floor, pointing down, shielded from ambient light, ahead of the front axle.
Calibrate the reference colours (mandatory): reset the board, send `c` within 5 s of `Send 'c' ...`, place the sensor at
mounting height over a tile and send `B` black, `S` silver, `U` blue, `R` red, `F` normal floor (stored in flash);
`Z` resets, `Q` quits. Verify each tile prints the right class.

## Complete pin table (proposed - edit `config.h` if your wiring differs)
| Function | GPIO |
|---|---|
| MPU6050 + TCS34725 (I2C) SDA / SCL | 21 / 22 |
| Front HC-SR04 TRIG / ECHO | 5 / 34 |
| Left HC-SR04 TRIG / ECHO | 14 / 35 |
| Right HC-SR04 TRIG / ECHO | 15 / 36 |
| L298N #1 ENA / IN1 / IN2 (front-left) | 25 / 26 / 27 |
| L298N #1 ENB / IN3 / IN4 (front-right) | 33 / 32 / 13 |
| L298N #2 ENA / IN1 / IN2 (rear-left) | 16 / 17 / 18 |
| L298N #2 ENB / IN3 / IN4 (rear-right) | 19 / 23 / 4 |

All three echo pins need a 1k/2k divider. Remove the L298N ENA/ENB jumpers, add 10k pull-downs on the four EN pins, share all
grounds, never power motors from a GPIO. To change pins with ChatGPT, use `PROMPT.md` in the repo root (upload the whole ZIP).

## Build and upload (Arduino CLI)
```
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --warnings all Robomaze_3_ultrasonics
arduino-cli upload  --fqbn esp32:esp32:esp32doit-devkit-v1 -p COM3 Robomaze_3_ultrasonics
arduino-cli monitor -p COM3 -c baudrate=115200
```
No extra libraries (`Wire` and `Preferences` ship with the ESP32 core; sensors are driven directly). If the upload hangs at
"Connecting...", hold BOOT. Flash with the wheels off the ground. First power-up: bench motor test
(`BENCH_MOTOR_TEST 1`), gyro sign check (rotate left by hand, `hdg=` must rise), keep the robot still during the 1.5 s gyro
calibration, place it with a wall on the left, an object ahead and something within 2 m on the right (all three sensors need
valid echoes to start). `x` in the serial monitor = latched stop.

## Host unit tests
Needs any g++. Run from the folder that contains both `Robomaze_3_ultrasonics/` and this extras folder:
```
bash Robomaze_3_ultrasonics_extras/test/run_tests.sh
```
Expect `N checks passed, 0 failed`. They cover the earlier behaviour plus the new right-sensor decisions (right open ->
right turn, all blocked -> U-turn, unknown right -> no guess, right sensor faults, startup check). If any fail, do not flash;
report the failing line.

## Serial diagnostics
4 Hz status line: state, `F=` / `L=` / `R=` distances (`(x)` = last echo missing or invalid), wall error, heading /
reference, gyro rate, motor commands, `col=<class>(R,G,B,C)`, `pos=(x,y)`, `DANGER` after red. `EVENT ...` lines for hazards,
checkpoints, blue waits, danger zone.

## Known limitations
* No encoders: position, "which blue tile", the creep before a left turn, the blue-tile creep and the reverse away from a hole are open-loop (time-based).
* No rear sensor: the reverse away from a hole is blind. Heading is gyro-integrated and drifts.
* Right "no echo" counts as open; the swing of a spin is not measured.
* A colour sensor or any-sensor fault stops the robot; faults are latched until power cycle.
* Nothing here has been run on hardware; all thresholds and speeds need tuning on the real robot.
