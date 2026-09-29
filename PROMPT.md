# Change the GPIO pins with ChatGPT (copy-paste prompt)

Use this when your wiring is different from the pins in the code. You paste the prompt, attach two files, fill in your pins,
and ChatGPT gives you back the edited files.

## Steps
1. Pick your version (folder): `Robomaze_2026` (base), `Robomaze_colour_sensors` (+ colour sensor) or `Robomaze_with_encoders` (+ colour sensor + encoders).
2. Open ChatGPT and start a new chat. Attach (or paste the text of) these two files from that folder:
   * `config.h`
   * the `.ino` file (same name as the folder, e.g. `Robomaze_with_encoders.ino`)
3. Copy the whole prompt below, fill in ONLY the values in `[square brackets]`, and send it. Leave a pin as `same` if you did not change it.
4. Replace your two files with the ones ChatGPT returns (Arduino IDE: open the `.ino`, keep the four files in the one folder).
5. Compile and upload (`arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 <folder>` or the Arduino IDE with board "DOIT ESP32 DEVKIT V1").
6. Bench-test before driving: set `BENCH_MOTOR_TEST 1` in `config.h`, upload with the wheels off the ground, check each motor spins forward, flip its `kInvert...` flag if not, then set it back to `0`.

## The prompt (copy everything inside the box)

```
You are helping me adapt an ESP32 DevKit V1 (Arduino-ESP32 core 3.x) robot firmware to MY wiring.
I attached config.h and the main .ino from the project (repo: https://github.com/tandonayaan-glitch/RoboMaze_KA.AP).
Project: left-wall-following maze robot, 2x HC-SR04, MPU6050 (I2C), TB/L298N motor drivers.
Optional parts depending on my version: TCS34725 colour sensor (I2C, shares SDA/SCL, no extra pins) and
2 wheel encoders (one left, one right, single channel).

YOUR JOB
Change ONLY pin numbers, motor rows and motor-invert flags so the code matches my wiring below.
Do NOT change navigation logic, thresholds, speeds, or nav_core.h / hw_types.h. Do not rename anything.

WHERE THINGS LIVE
- Pin numbers: config.h, section "Pins (ESP32 DevKit V1)": constexpr uint8_t kPin... names
  (kPinSda, kPinScl, kPinFrontTrig, kPinFrontEcho, kPinLeftTrig, kPinLeftEcho,
   kPinD1EnA, kPinD1In1, kPinD1In2, kPinD1EnB, kPinD1In3, kPinD1In4,
   kPinD2EnA, kPinD2In1, kPinD2In2, kPinD2EnB, kPinD2In3, kPinD2In4 (only if 2 drivers),
   kPinEncLeft, kPinEncRight (only in the encoders version)).
- Motor table: the kMotors[] array in the .ino. Each row is
  { EN pin, IN1 pin, IN2 pin, leftSide (true = left side of the robot), invert, "NAME" }.
  Keep the rows referencing the kPin... constants; only change leftSide/invert/name if my wiring needs it.
- Direction flags: config.h kInvert... constants (true = that motor is wired/mounted backwards).
- I2C address: kImuAddr (MPU6050, 0x68 or 0x69) and kColorAddr (TCS34725, 0x29) - change only if I say so.

ESP32 PIN RULES YOU MUST ENFORCE (config.h has static_asserts for these)
- Outputs (TRIG, EN, IN pins) must be one of: 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33.
- Input-only pins (echo pins and encoder pins) must be one of: 34, 35, 36, 39. (If I need an echo/encoder
  on a different pin, tell me it needs a code change to the inputOnly() check and explain the risk.)
- Never use GPIO 0, 1, 2, 3, 6-12, 15 (boot/flash/UART pins) for anything.
- I2C default: SDA=21, SCL=22 (shared by MPU6050 and TCS34725).
- No pin may be used twice. EN pins must be PWM-capable (any output pin above is fine).
- HC-SR04 ECHO is 5 V: remind me if I did not say I use a voltage divider (1k + 2k) or level shifter.
- If two functions clash or a pin is illegal, DO NOT guess: tell me which pins clash and suggest free legal pins.

MY VERSION: [Robomaze_2026 | Robomaze_colour_sensors | Robomaze_with_encoders]
NUMBER OF L298N DRIVERS: [1 | 2]

MY WIRING (GPIO numbers; write "same" to keep the default)
- MPU6050 SDA: [same]   SCL: [same]
- Front HC-SR04 TRIG: [same]   ECHO: [same]
- Left  HC-SR04 TRIG: [same]   ECHO: [same]
- Driver 1: ENA: [same]  IN1: [same]  IN2: [same]  ENB: [same]  IN3: [same]  IN4: [same]
    motor on channel A is: [front-left | left side]   motor on channel B is: [front-right | right side]
- Driver 2 (skip if 1 driver): ENA: [ ]  IN1: [ ]  IN2: [ ]  ENB: [ ]  IN3: [ ]  IN4: [ ]
    motor on channel A is: [rear-left]   motor on channel B is: [rear-right]
- Encoders (only encoders version): left encoder pin: [same]   right encoder pin: [same]
- Motors that spin BACKWARDS when told to go forward: [none | list names]
- Anything else that differs (I2C addresses etc.): [none]

WHAT TO RETURN
1. A short table: function | old pin | new pin, and a list of exactly which lines you changed.
2. The COMPLETE updated config.h in one code block.
3. The COMPLETE updated .ino in one code block (or, if only the kMotors[] table changed, that table plus the
   exact place to paste it).
4. A pin-conflict check: confirm no duplicate pins, all outputs/inputs legal per the rules above.
5. A wiring checklist for MY pins (including the 1k/2k echo dividers, 10k pull-downs on EN pins,
   common grounds, and 10k pull-ups on GPIO34-39 encoder/echo lines only where needed).
6. The compile command: arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 <folder>
Do not invent features. If something in my message is ambiguous, ask me before changing code.
```

## Default pins (what the code uses if you change nothing)
| Function | GPIO |
|---|---|
| I2C SDA / SCL (MPU6050, colour sensor) | 21 / 22 |
| Front HC-SR04 TRIG / ECHO | 5 / 34 |
| Left HC-SR04 TRIG / ECHO | 14 / 35 |
| Driver 1 ENA / IN1 / IN2 | 25 / 26 / 27 |
| Driver 1 ENB / IN3 / IN4 | 33 / 32 / 13 |
| Driver 2 ENA / IN1 / IN2 (2-driver versions) | 16 / 17 / 18 |
| Driver 2 ENB / IN3 / IN4 (2-driver versions) | 19 / 23 / 4 |
| Left / right encoder (encoders version) | 36 / 39 |

## Check ChatGPT's answer
* No pin appears twice in `config.h`.
* Echo and encoder pins are 34, 35, 36 or 39. Everything else you wired is an output pin from the list.
* `nav_core.h` and `hw_types.h` were not touched.
* It compiles. If you see a `static_assert` error mentioning pins, ChatGPT picked an illegal pin: paste the error back into the chat.
* Motors: run the bench motor test (step 6 above) before putting the robot on the floor.
