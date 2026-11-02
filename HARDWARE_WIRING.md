# Hardware Wiring - MG995 Base Servo Build

Fork of [EbenKouao/arduino-robot-arm](https://github.com/EbenKouao/arduino-robot-arm) (original by Smartbuilds.io). This document describes the modified hardware: the NEMA 17 base stepper is replaced by an **MG995 positional servo on PCA9685 CH0**.

Wiring overview: [`block-diagrams/mg995-base-servo-wiring.svg`](block-diagrams/mg995-base-servo-wiring.svg) (hand-drawn schematic; the original Fritzing diagrams still show the old stepper build).

> **Status:** the firmware compiles for an Arduino Uno (checked against the real Adafruit PWM Servo Driver Library 3.0.3 with `arduino-cli`). **Nothing here has been tested on physical hardware.** Treat all calibration values as unverified until you have run the calibration sketch.

## 1. Overview

An Arduino Uno drives six servos (one per joint) through a PCA9685 (I2C) and receives single-character commands from an HC-05 Bluetooth module (hand-gesture glove or phone app).

## 2. Original vs. modified

| | Original | This build |
|---|---|---|
| Base rotation | NEMA 17 + A4988, STEP D5 / DIR D4 | MG995 on PCA9685 CH0 |
| Base motion | relative, 120 steps per command | absolute position, incremental, software limits |
| Base power | 12 V jack to driver | external 5-6 V supply (shared servo rail) |
| Servo power | 3.7 V LiPo on PCA9685 V+ | external regulated 5-6 V on V+ |
| Arduino pins D4, D5 | stepper | unused |
| Servo channels | CH0-5, dual-servo elbow on CH0+CH1 | CH0-5, one servo per joint (re-mapped, see 4) |

## 3. Components

- Arduino Uno
- PCA9685 16-channel PWM driver (Hobby Components HCMODU0097 / compatible, address 0x40)
- Adafruit PWM Servo Driver Library (Arduino Library Manager). Install it before compiling.
- MG995 servo (base) + 5 other servos (shoulder, elbow, wrist 1, wrist 2, gripper)
- HC-05 Bluetooth module (breakout with onboard regulator)
- 1 kΩ and 2 kΩ resistors (HC-05 RXD divider)
- Regulated 5-6 V DC supply, **3 A minimum for the MG995 alone**; size it for the whole arm (all servos moving at once draws much more; 5 A or more is a sensible target)
- 470-1000 µF electrolytic capacitor across the servo power rail (recommended)
- Jumper wires, breadboard, USB cable

## 4. PCA9685 channel mapping (this build)

| Channel | Servo | Commands |
|---|---|---|
| CH0 | **MG995 base rotation** | `T`/`S` left, `t`/`O` right |
| CH1 | Shoulder | `P` up, `p` down |
| CH2 | Elbow (single servo) | `C` up, `c` down |
| CH3 | Wrist 1 (up/down) | `U` up, `D` down (`G` = legacy glove alias of `D`) |
| CH4 | Wrist 2 (rotation) | `R` CW, `L` CCW |
| CH5 | Gripper | `F` open, `f` close |

Original → new: shoulder 2→1, elbow (was a mirrored pair on 0+1) → one servo on 2, wrist 1 3→3, wrist 2 4→4, gripper 5→5. There is **no CH6** and no mirrored elbow logic any more: the elbow is one servo driven directly by its own position value (parking 60, range 10-180, step 20, unchanged from the original left-elbow values). A servo that was previously the mirrored half must be removed, and the single elbow servo may need its direction checked: if `C`/`c` move the wrong way, that joint's horn orientation (or the sign of its code) needs adjusting on the real arm.

Channel numbers are constants at the top of `arduino_robot_arm_code.ino`.

## 5. MG995 wiring

| MG995 wire | Connect to |
|---|---|
| Signal (orange/yellow) | PCA9685 CH0 signal pin |
| Red | CH0 V+ pin (the servo rail, fed from the external 5-6 V supply) |
| Brown/Black | CH0 GND pin / external supply GND |

## 6. PCA9685 wiring

| PCA9685 | Connect to |
|---|---|
| VCC | Arduino 5V (logic only) |
| GND | Arduino GND (and external supply GND) |
| SDA | Arduino A4 |
| SCL | Arduino A5 |
| V+ | External servo supply positive |
| OE | not connected |

VCC (logic) and V+ (servo power) are **separate** connections.

## 7. HC-05 wiring

| HC-05 | Connect to |
|---|---|
| VCC | Arduino 5V |
| GND | Arduino GND |
| TXD | Arduino D0 (RX) |
| RXD | Arduino D1 (TX) **through the divider below** |

Baud rate: 4800 (the HC-05 must be set to 4800 via AT mode, as in the original project).

## 8. Voltage divider (HC-05 RXD)

```
Arduino D1 (TX)
      |
     1 kΩ
      |
      +------ HC-05 RXD
      |
     2 kΩ
      |
     GND
```

The HC-05 RXD pin is a 3.3 V logic input; the Arduino TX is 5 V. The divider gives 5 V × 2/3 ≈ 3.3 V. HC-05 TXD → D0 needs no divider (3.3 V is read as HIGH by the Uno).

## 9. External power requirements

- Regulated **5-6 V** DC, 3 A or more. Do not exceed 6 V unless you have confirmed the rating of your PCA9685 board and every servo (not verified for this build: read the servo and board labels).
- External + → PCA9685 V+. External − → PCA9685 GND **and** Arduino GND (common ground is mandatory).
- **Never power the MG995 from the Arduino 5V pin.** The Arduino 5V is only for the PCA9685 VCC and the HC-05.
- Because V+ is shared, all other servos now also run from this supply. Check that your gripper/wrist/shoulder/elbow servos are rated for the voltage you choose.
- The old 12 V adapter, A4988 driver and NEMA 17 are no longer used.

## 10. Bluetooth commands (4800 baud)

| Char | Action | Char | Action |
|---|---|---|---|
| `T` / `S` | Base left (`T` = glove thumb flex above threshold, `S` = original phone command) | `U` / `D` | Wrist 1 up / down (`G` = legacy alias of `D`) |
| `t` / `O` | Base right (`t` = glove thumb flex below threshold, `O` = original phone command) | `R` / `L` | Wrist 2 CW / CCW |
| `P` / `p` | Shoulder up / down | `F` / `f` | Gripper open / close |
| `C` / `c` | Elbow up / down | | |

`T`/`t` were already sent by the glove sketch (commented "thumb 1 - Base Rotation") but the original arm sketch ignored them. The original phone commands `S`/`O` are kept as aliases of `T`/`t`; `robotic_glove_code.ino` is unchanged. Each command moves the base by `servo_base_pos_increment` microseconds, clamped to the limits.

## 11a. Units / migration from the old servo library

The firmware now uses the **Adafruit PWM Servo Driver Library** (`Adafruit_PWMServoDriver`). All positions are **pulse widths in microseconds**.

The previous Hobby Components PCA9685 library's `Servo(ch, pos)` behaviour was read from the library source (v0.1, third-party GitHub copy `dray-hw/robot-arm`; the Hobby Components forum is not reachable, so this copy is not confirmed identical to the original download):

- `Init(SERVO_MODE)` set the prescaler for 50 Hz: `(25 MHz / (4096 * 50)) - 1 = 121` (integer maths), i.e. a real period of 122 * 4096 / 25 MHz = 19.99 ms (50.03 Hz).
- `Servo(ch, pos)` wrote OFF-tick = `min(pos + 110, 590)` (ON-tick 0). `SERVO_TRIM_MIN = 110`, `SERVO_TRIM_MAX = 590`.
- So pulse = `(pos + 110) * 4.88 us`, one old unit = one PCA9685 tick = about 4.88 us:

| Old pos | Ticks | Pulse |
|---|---|---|
| 0 | 110 | ~537 us |
| 10 | 120 | ~586 us |
| 240 | 350 | ~1708 us |
| 400 | 510 | ~2489 us |
| 480 | 590 | ~2879 us |

This assumes a nominal 25 MHz oscillator. The pulse widths are exact for that assumption and say nothing about the real servo travel. Old values were never calibrated to angles. The old range reached ~2.9 ms, beyond the 1000-2000 us envelope now in force, so converted limits are **clipped** to it: base 140-340 became 1220-2196, clipped to 1220-2000 us. Old parking for elbow, wrist 1, wrist 2 and gripper converted to 830 / 766 / 844 / 844 us, below 1000 us, so they are clamped to 1000 us.

Configuration at the top of `arduino_robot_arm_code.ino`:

| Setting | Default | Meaning |
|---|---|---|
| `SERVO_FREQ_HZ` | 50 | PWM frequency |
| `OSC_FREQ_HZ` | 25000000 | nominal oscillator. Real chips vary a few percent. To calibrate: measure the output period T with a scope/logic analyser and set `OSC_FREQ_HZ = 25000000 * 0.020 / T` |
| `SERVO_MIN_US` / `SERVO_MAX_US` | 1000 / 2000 | global test envelope; every pulse is clamped to it |
| `joint_enabled[6]` | base only | a disabled joint gets no pulse and ignores its commands |
| `servo_*_min_pos` / `_max_pos` | see sketch | per-joint limits (us) |
| `servo_*_pos_increment` | old increment * 4.88 | us per command |

Enable joints one at a time, and only after calibrating that joint.

## 11. MG995 calibration

Positions are **pulse widths in microseconds, not degrees**. The calibration sketch works on one channel at a time (default CH0; press `0`-`5` to select another). Only the selected channel is ever pulsed. Calibrate empirically:

1. Remove the servo horn / free the base so nothing can jam. Disconnect the HC-05 from D0/D1.
2. Power the external supply, upload `code/test-code/mg995_base_calibration`, open Serial Monitor at 9600 baud (no line ending).
3. No pulse is sent until you press a move key or `c` (centre = 1500 us). Press `d`/`a` (±10 us) or `w`/`s` (±1 us) to creep toward each end; the sketch clamps to 1000-2000 us (`CAL_MIN_US`/`CAL_MAX_US`; widen only deliberately). Stop and back off ~20 us when the servo hums or reaches its stop. `x` makes the channel limp.
4. Note the **min**, **max** and the value that points the base straight ahead.
5. Edit these in `arduino_robot_arm_code.ino`:

| Setting | Default (uncalibrated) | Meaning |
|---|---|---|
| `servo_base_parking_pos` | 1708 us | startup position |
| `servo_base_min_pos` / `servo_base_max_pos` | 1220 / 2000 us | software limits |
| `servo_base_pos_increment` | 49 us | per command |
| `servo_base_step_us` | 5 us | ramp step size |
| `servo_base_step_delay` | 5 | ms per ramp step (higher = slower) |
| `servo_base_left_direction` | +1 | set −1 if `T`/`S` turns the wrong way |

The firmware clamps every base move to min/max, and the startup position to the limits too.

## 12. Upload instructions

1. In the Arduino IDE open Library Manager and install **Adafruit PWM Servo Driver Library** (accept Adafruit BusIO).
2. **Disconnect the HC-05 from D0/D1** (it shares the USB serial lines) before uploading.
3. Select *Arduino Uno*, open `code/arduino_robot_arm_code/arduino_robot_arm_code.ino`, upload.
4. Reconnect the HC-05, power the servo supply, pair the HC-05 and send commands.

At power-up the base servo moves to `servo_base_parking_pos`; with no position feedback it can jump from wherever it was. Keep clear of the arm and power the servo supply with the arm in a safe pose. The other five joints are **disabled** (no pulse, commands ignored; at startup every disabled channel is forced to full-OFF, so a stale pulse from before an Arduino-only reset is cancelled) until you set their `joint_enabled` entry to `true` after calibrating them.

## 13. Safety

- **Startup is open-loop.** On power-up the base jumps from wherever the horn physically is to `servo_base_parking_pos` (default 1708 us, uncalibrated) at full speed; there is no position feedback, and the step-by-step ramp only applies to later moves. Keep the horn off / the arm clear on first power-up.
- **Every servo write is clamped** to that joint's configured min/max (no overshoot, no negative-to-unsigned wrap). If any limit pair is invalid (needs `1000 <= min < max <= 2000`, and the base parking inside its limits) the sketch sends no servo pulse and halts with an error on Serial.

- Keep fingers and cables out of the arm's reach when powering on.
- Use a supply with current limiting or a fuse; a stalled MG995 can draw 1 A or more.
- Do not exceed 6 V on V+ without verifying your board and servo ratings.
- Do not force servo horns against end stops; stop calibrating when the servo buzzes.
- Always join all grounds; never connect the supply with reversed polarity.
- The HC-05 RXD pin is not 5 V tolerant; keep the divider in place.
