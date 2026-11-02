# Arduino Robotic Arm - Controlled by Hand Gestures

This video is a tutorial on how to build a DIY Robotic Arm controlled by hand gestures. The Robotic arm is able to be controlled wirelessly from a distance as well as perform basic functions on its own.

Find the Full Instruction Tutorial on smartbuilds.io
For more tutorials subscribe to my [YouTube channel](https://www.youtube.com/channel/UCGxwyXJWEarxh2XWqvygiIg)

Robot Arm: Thingiverse 3D Printed Robotic Arm - [credits: Wonder Tiger](https://www.thingiverse.com/thing:1748596)

- Part 1: Gripper Parts
- Part 2: Base and Turntable
- Part 3: Robotic Core Arm

Robotic Glove: Thingyverse Robotic Glove/ Gauntlet - [(credits: Roman 13)](https://www.thingiverse.com/thing:1982745)

- Robotic Glove

## Fork modification: MG995 base servo

This fork replaces the NEMA 17 stepper (base rotation) with an **MG995 positional servo on PCA9685 CH0**, powered from a separate regulated 5-6 V supply. Bluetooth (HC-05, 4800 baud) and the other joints keep their original commands but use a new PCA9685 channel map.

| Channel | Servo | Channel | Servo |
|---|---|---|---|
| CH0 | MG995 base | CH3 | Wrist 1 |
| CH1 | Shoulder | CH4 | Wrist 2 |
| CH2 | Elbow (single servo) | CH5 | Gripper |

Six servos, one per joint. The original mirrored dual-servo elbow is gone.

Glove commands: `T`/`t` base (`S`/`O` kept as phone aliases), `P`/`p` shoulder, `C`/`c` elbow, `U`/`D` wrist 1, `L`/`R` wrist 2, `F`/`f` gripper. No CH6, one servo per joint.

- Wiring, power, voltage divider, commands, calibration and safety: [HARDWARE_WIRING.md](HARDWARE_WIRING.md)
- Wiring overview: [block-diagrams/mg995-base-servo-wiring.svg](block-diagrams/mg995-base-servo-wiring.svg)
- Calibration sketch: `code/test-code/mg995_base_calibration`
- Requires the **Adafruit PWM Servo Driver Library** (Arduino Library Manager; also installs Adafruit BusIO). All servo positions are pulse widths in microseconds; see "Units / migration" in HARDWARE_WIRING.md. Only the base joint is enabled by default until each joint is calibrated. The original Fritzing diagrams in `block-diagrams/` still show the old stepper build.

Original project and credits above are by Smartbuilds.io (EbenKouao/arduino-robot-arm).
