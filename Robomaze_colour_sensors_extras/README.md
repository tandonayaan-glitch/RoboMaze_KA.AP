# Robomaze_colour_sensors - wall follower + TCS34725 floor colours (2x L298N, 4 motors)

Everything in the base `Robomaze_2026` firmware (left-wall following at 9 cm, 7 cm front stop, gyro turns, left-opening
detection, dead-end U-turn, fault handling) plus floor-colour rules. **Sketch folder = `Robomaze_colour_sensors/`
(code only, import this into the Arduino IDE).** This README and `test/` live outside it.

**Status:** the sketch compiles to object code with `--warnings all` and no warnings, but it has NOT been linked, unit
tested, or run on a robot (the development PC blocked the ESP32 archiver and the host assembler through a Windows
Application Control policy). The colour reference values are **placeholders that you must calibrate** (below).

## Colour rules implemented
| Floor colour | Behaviour |
|---|---|
| BLACK (hole) | Stops at once, reverses ~4 cm (`kBackoffMs`, blind - no rear sensor), then treats the direction as blocked: turns left if the left side is open, else right, else U-turn, else `SAFE_STOP`. Never drives on. Also checked after every turn. Max 4 per episode. |
| SILVER (checkpoint) | Saves the current estimated position, heading and a counter (RAM + flash, survives reboot), prints an `EVENT` line. Does not stop the robot. Fires once per silver tile (re-arms after 3 non-silver samples). |
| BLUE | Creeps `kBlueAdvanceMs` onto the tile, stops and waits **5 s consecutively** (`kBlueWaitMs`), then drives on. If blue is lost for 400 ms the timer restarts; after 3 restarts it gives up on that tile. Each blue tile has a visit and completed counter (`kMaxBlueTiles` = 16). |
| RED | Recognises the Dangerous Zone entrance: sets a flag, counts entrances, prints an event, optionally scales speed (`kDangerSpeedScale`, default 1.0 = no change). |
| Anything else | Normal floor. |

**Things I assumed (please confirm / tell me the rules):**
* Blue is waited on **every** visit (`kBlueWaitEveryVisit = true`); set `false` to wait only on the first.
* "Individual blue tile" = the tile cell of the robot's **dead-reckoned position** (`kTileCm` = 30 cm). There are no encoders, so this is an estimate that drifts; a revisited tile can be counted as a new one. Adding wheel encoders would fix that.
* Silver "save position" = estimated (x, y) in cm from the start (x forward, y left) plus heading. Nothing yet *uses* the checkpoint (e.g. returning to it); only saving is implemented.
* Nothing special happens inside the Dangerous Zone beyond the flag and optional slow-down, and the flag stays set until reboot. Tell me the zone rules and I will extend it.
* Colours are ignored while the robot is spinning in a turn (the sensor sweeps the floor); they are checked again when the turn ends.

## Wiring the colour sensor
TCS34725 uses I2C, shared with the MPU6050 - **no new GPIO**: VIN -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22 (address 0x29; 3.3 V modules have pull-ups already, do not add more if the bus works). LED pin -> 3V3 (always on) for consistent lighting; INT unused.
Mount it **at the front of the robot, 3-5 mm above the floor, pointing straight down**, shielded from room light. Black-hole detection only works if the sensor reaches the tile edge *before* the wheels do; mount it ahead of the front axle, and tune `kBackoffMs`.

## Colour calibration (mandatory before use)
The reference readings in `config.h` (`kRefBlack`, `kRefSilver`, `kRefBlue`, `kRefRed`, `kRefFloor`) are guesses. Calibrate with your real tiles:
1. Wheels off the ground / robot held over a tile. Open the Serial Monitor at 115200 and reset the board.
2. When it prints `Send 'c' within 5 s...`, send `c`. Calibration mode starts (motors off) and prints `R G B C -> class` twice a second.
3. Place the sensor at its **mounting height** over a tile and send: `B` black, `S` silver, `U` blue, `R` red, `F` normal floor. Each takes a 0.6 s average and is stored in flash. `Z` resets to the config defaults; `Q` leaves calibration mode.
4. Verify by sliding the sensor over each tile: the printed class must be correct and steady. If black/silver/floor are confused, calibrate again at the exact height and lighting of the competition, or lower `kColorMaxDist`.
Re-calibrate whenever lighting, sensor height or floor material changes.

## Build, upload, tests
Full step-by-step setup (installing Arduino CLI, ESP32 core, finding the port) is in the base project README. Commands for this sketch:
```
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --warnings all Robomaze_colour_sensors
arduino-cli upload  --fqbn esp32:esp32:esp32doit-devkit-v1 -p COM3 Robomaze_colour_sensors
arduino-cli monitor -p COM3 -c baudrate=115200
```
No extra Arduino libraries are needed (`Wire` and `Preferences` ship with the ESP32 core; the TCS34725 is driven directly).
Host unit tests (needs g++; run from the folder containing both `Robomaze_colour_sensors/` and this `extras` folder):
```
bash test/run_tests.sh
```

## Pin and hardware changes
Same as the base project: pins live only in `config.h` and the `kMotors[]` table in `Robomaze_colour_sensors.ino`. Colour-specific tuning is in the `TCS34725` and `Floor-colour behaviour` sections of `config.h`.

## Serial diagnostics
Adds `col=<class>(R,G,B,C) pos=(x,y) [DANGER]` to the 4 Hz status line, and `EVENT ...` lines for hazards, checkpoints, blue waits and the danger zone.

## Extra limitations of the colour version
* Position and "which blue tile" are open-loop estimates (no encoders).
* Reversing away from a hole is blind (no rear sensor) and time-based.
* A colour sensor fault stops the robot (`kColorRequired = true`) because holes could not be detected.
* Flash writes at a silver checkpoint can stall the loop for a few milliseconds.
* Silver vs bright white depends entirely on calibration and lighting.
