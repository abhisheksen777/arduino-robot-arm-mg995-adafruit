/******************************************************************************
  Author: Smartbuilds.io
  YouTube: https://www.youtube.com/channel/UCGxwyXJWEarxh2XWqvygiIg
  Fork your own version: https://github.com/EbenKouao/arduino-robot-arm
  Check out the full article: https://smartbuilds.io/diy-arduino-robot-arm-controlled-hand-gestures/
  Date: 06/01/2021
  Robot Arm
  Version 1.0
  Creator: smartbuilds.io
  Description: Robotic Arm Mark II - Servo Motor
  Modification: Base rotation is an MG995 positional servo on PCA9685 CH0
                (replaces the original NEMA 17 stepper + driver). See
                HARDWARE_WIRING.md for wiring, power and calibration.

  To use the module connect it to your Arduino as follows:

  PCA9685...........Uno/Nano
  GND...............GND (also tie to external servo supply GND)
  OE................N/A
  SCL...............A5
  SDA...............A4
  VCC...............5V  (logic only)
  V+................External regulated 5-6V servo supply (NOT the Arduino 5V)

  PCA9685 channel map (this build):
  CH0 Base (MG995)  CH1 Shoulder  CH2 Elbow  CH3 Wrist 1  CH4 Wrist 2  CH5 Gripper
  Commands: T/t (also S/O) Base  P/p Shoulder  C/c Elbow  U/D Wrist 1  L/R Wrist 2  F/f Gripper

******************************************************************************/

/* Adafruit PCA9685 library: Library Manager -> "Adafruit PWM Servo Driver Library" */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* I2C slave address of the PCA9685 (default 0x40) */
#define  I2CAdd 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2CAdd);

/* ---- PCA9685 / servo timing (configurable) ------------------------------
   ALL positions in this sketch are PULSE WIDTHS IN MICROSECONDS (us), not the
   the old 0-480 library units and not degrees.
   OSC_FREQ_HZ: nominal 25 MHz internal oscillator. Real chips differ by a few
   percent, which scales every pulse by the same factor. Getting it exact needs
   a measurement (scope/logic analyser on a channel: measure the period, then
   OSC_FREQ_HZ = 25000000 * 0.020 / measured_period_seconds for 50 Hz). */
const float    SERVO_FREQ_HZ = 50;
const uint32_t OSC_FREQ_HZ   = 25000000UL;
/* Global test envelope. Every pulse is clamped to this, whatever the
   per-joint limits say. Widen deliberately, only after calibration. */
const int SERVO_MIN_US = 1000;
const int SERVO_MAX_US = 2000;

//PCA9685 channel assignment - change here if you rewire a servo
const int CH_BASE = 0;        // MG995 base rotation
const int CH_SHOULDER = 1;
const int CH_ELBOW = 2;
const int CH_WRIST_1 = 3;     // wrist up / down (U / D commands)
const int CH_WRIST_2 = 4;     // wrist rotation (R / L commands)
const int CH_GRIPPER = 5;

/* Per-joint enable. A disabled joint gets NO pulse at all (it stays limp) and
   ignores its commands. Enable one joint at a time, only after calibrating it
   with code/test-code/mg995_base_calibration. Only the base is on for the
   initial MG995 test. */
const bool joint_enabled[6] = { true,    // CH0 base (MG995)
                                false,   // CH1 shoulder
                                false,   // CH2 elbow
                                false,   // CH3 wrist 1
                                false,   // CH4 wrist 2
                                false }; // CH5 gripper

/* Joint values below were converted from the old 0-480 library positions with the
   verified formula  us = (old_pos + 110) * 1e6 / (4096 * 50.029)  (~4.88 us per
   old unit) and then clipped to the 1000-2000 us envelope. They are still
   UNCALIBRATED. See HARDWARE_WIRING.md "Units / migration". Old parking for
   elbow / wrist 1 / wrist 2 / gripper converted to 830 / 766 / 844 / 844 us,
   which is BELOW the envelope, so parking is clamped up to 1000 us here. It is
   not driven while the joint is disabled. */
//initial parking position of the motor
const int servo_joint_elbow_parking_pos = 1000;
const int servo_joint_1_parking_pos = 1000;
const int servo_joint_2_parking_pos = 1000;
const int servo_joint_3_parking_pos = 1000;
const int servo_joint_4_parking_pos = 1000;

//Degree of robot servo sensitivity - Intervals (us per command)
int servo_joint_elbow_pos_increment = 98;
int servo_joint_1_pos_increment = 98;
int servo_joint_2_pos_increment = 244;
int servo_joint_3_pos_increment = 293;
int servo_joint_4_pos_increment = 195;

//Keep track of the current value of the motor positions
int servo_joint_elbow_parking_pos_i = servo_joint_elbow_parking_pos;
int servo_joint_1_parking_pos_i = servo_joint_1_parking_pos;
int servo_joint_2_parking_pos_i = servo_joint_2_parking_pos;
int servo_joint_3_parking_pos_i = servo_joint_3_parking_pos;
int servo_joint_4_parking_pos_i = servo_joint_4_parking_pos;


//Minimum and maximum pulse (us) per joint
int servo_joint_elbow_min_pos = 1000;
int servo_joint_elbow_max_pos = 1415;


int servo_joint_1_min_pos = 1000;
int servo_joint_1_max_pos = 2000;

int servo_joint_2_min_pos = 1000;
int servo_joint_2_max_pos = 2000;

int servo_joint_3_min_pos = 1000;
int servo_joint_3_max_pos = 2000;

int servo_joint_4_min_pos = 1000;
int servo_joint_4_max_pos = 1122;

int servo_joint_1_pos = 0;
int servo_joint_2_pos = 0;
int servo_joint_3_pos = 0;
int servo_joint_4_pos = 0;

char state = 0; // Changes value from ASCII to char
int response_time = 5;
int response_time_4 = 2;
int loop_check = 0;
int response_time_fast = 20;
int action_delay = 600;

//Posiion of motor for example demos
unsigned int Pos;

/* MG995 base rotation (PCA9685 CH0), all values in microseconds.
   CONSERVATIVE, UNCALIBRATED defaults. The old window 140-340 converted to
   1220-2196 us; the top is clipped to the 2000 us envelope. Parking 240 old
   units = 1708 us (NOT mid-range: the middle of the window is ~1610 us).
   Widen step by step using code/test-code/mg995_base_calibration, then copy
   the results here. */
const int servo_base_parking_pos = 1708; // startup position (must be within min/max)
int servo_base_min_pos = 1220;           // software limit, "left" or "right" end
int servo_base_max_pos = 2000;           // software limit, opposite end
int servo_base_pos_increment = 49;       // us moved per T / t / S / O command
int servo_base_step_us = 5;              // us per ramp step (~1 old unit)
int servo_base_step_delay = 5;           // ms per ramp step (bigger = slower)
// +1: baseRotateLeft() increases the pulse, -1: it decreases it.
// Flip this if the base turns the wrong way for 'T' / 'S' (no rewiring needed).
const int servo_base_left_direction = 1;
int servo_base_pos_i = servo_base_parking_pos; // current base pulse (us)

//Old 0-480 library position -> us, for the commented-out demos only.
int legacyPosToUs(long oldPos) {
  return (int)((oldPos + 110) * 1000000L / 204900L); // 4096 * 50.029 Hz ~= 204900
}

//Clamp to [mn, mx] using long math, so a stepped value can never overshoot a limit
//or go negative (a negative int would wrap to a huge unsigned servo position).
int clampJoint(long v, int mn, int mx) {
  return (int)constrain(v, (long)mn, (long)mx);
}

//The ONLY way joints reach the PCA9685: every write is clamped to the joint's limits
//and the global envelope, and skipped for disabled joints.
void writeJoint(int ch, long pos, int mn, int mx) {
  if (!joint_enabled[ch]) return;
  int us = clampJoint(pos, mn, mx);
  pwm.writeMicroseconds(ch, (uint16_t)clampJoint(us, SERVO_MIN_US, SERVO_MAX_US));
}

//Every limit pair must satisfy envelope <= min < max <= envelope, or the sketch refuses to move anything.
bool limitsValid() {
  return servo_joint_elbow_min_pos >= SERVO_MIN_US && servo_joint_elbow_min_pos < servo_joint_elbow_max_pos && servo_joint_elbow_max_pos <= SERVO_MAX_US
      && servo_joint_1_min_pos >= SERVO_MIN_US && servo_joint_1_min_pos < servo_joint_1_max_pos && servo_joint_1_max_pos <= SERVO_MAX_US
      && servo_joint_2_min_pos >= SERVO_MIN_US && servo_joint_2_min_pos < servo_joint_2_max_pos && servo_joint_2_max_pos <= SERVO_MAX_US
      && servo_joint_3_min_pos >= SERVO_MIN_US && servo_joint_3_min_pos < servo_joint_3_max_pos && servo_joint_3_max_pos <= SERVO_MAX_US
      && servo_joint_4_min_pos >= SERVO_MIN_US && servo_joint_4_min_pos < servo_joint_4_max_pos && servo_joint_4_max_pos <= SERVO_MAX_US
      && servo_base_parking_pos >= servo_base_min_pos && servo_base_parking_pos <= servo_base_max_pos
      && servo_base_min_pos >= SERVO_MIN_US && servo_base_min_pos < servo_base_max_pos && servo_base_max_pos <= SERVO_MAX_US;
}

void setup()
{
  /* Invalid limits: send NO servo pulse at all, report, and halt. */
  if (!limitsValid()) {
    Serial.begin(4800);
    Serial.println(F("ERROR: invalid servo limits (need envelope <= min < max <= envelope). Halted."));
    while (true) {}
  }

  /* Initialise the library and set it to 'servo mode' */
  if (!pwm.begin()) {                   // I2C device not found
    Serial.begin(4800);
    Serial.println(F("ERROR: PCA9685 not found on I2C. Halted."));
    while (true) {}
  }
  /* A MODE1 restart does NOT clear channel registers: if only the Arduino reset,
     an old pulse could still be running. Force every channel except the enabled
     joints to full-OFF (no signal) before anything else. */
  for (uint8_t c = 0; c < 16; c++) {
    if (c >= 6 || !joint_enabled[c]) pwm.setPWM(c, 0, 4096);
  }
  pwm.setOscillatorFrequency(OSC_FREQ_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);        // wakes the chip; still no pulse until a write

  /* Move the base to its parking position (clamped to the base limits).
     Until now the PCA9685 outputs no pulse on this channel, so the MG995 is idle.
     NOTE: this first move is OPEN-LOOP. There is no position feedback, so the
     firmware cannot know where the horn really is; the servo jumps from its
     physical position to the parking value at full speed (the ramp in
     baseMoveTo() only applies to later moves). Keep the horn off / the arm
     clear on first power-up, and set servo_base_parking_pos from calibration. */
  servo_base_pos_i = constrain(servo_base_parking_pos, servo_base_min_pos, servo_base_max_pos);
  writeJoint(CH_BASE, servo_base_pos_i, servo_base_min_pos, servo_base_max_pos);

  Serial.begin(4800); // Initialise default communication rate of the Bluetooth module


  delay(3000);
  //wakeUp(); -- Uncomment for Example Demo 1
  //flexMotors(); -- Uncomment for Example Demo 1

}


void loop() {


  if (Serial.available() > 0) { // Checks whether data is coming from the serial port

    state = Serial.read(); // Reads the data from the serial port
    Serial.print(state); // Prints out the value sent


    //For the naming of the motors, refer to the article / tutorial
    //Move (Base Rotation) MG995 Servo Left  ('T' = glove thumb flex above threshold, 'S' = original phone command)
    if (state == 'T' || state == 'S') {
      baseRotateLeft();
      delay(response_time);
    }

    //Move (Base Rotation) MG995 Servo Right ('t' = glove thumb flex below threshold)
    if (state == 't' || state == 'O') { // 'O' = original phone command
      baseRotateRight();
      delay(response_time);
    }


    //Move Shoulder Down
    if (state == 'p') {

      shoulderServoForward();
      delay(response_time);

    }

    //Move Shoulder Up
    if (state == 'P') {

      shoulderServoBackward();
      delay(response_time);

    }

    //Move Elbow Down
    if (state == 'c') {

      elbowServoForward();
      delay(response_time);

    }

    //Move Elbow Up
    if (state == 'C') {

      elbowServoBackward();
      delay(response_time);

    }


    //Move Wrist 1 UP
    if (state == 'U') {

      wristServo1Backward();
      delay(response_time);
    }

    //Move Move Wrist 1 Down
    if (state == 'D' || state == 'G') { // 'G' = legacy glove code, same as 'D'

      wristServo1Forward();
      delay(response_time);

    }


    //Move Wrist 2 Clockwise
    if (state == 'R') {

      wristServoCW();
      delay(response_time);

    }

    //Move Wrist 2 Counter-CW
    if (state == 'L') {

      wristServoCCW();
      delay(response_time);

    }


    //Open Claw Grip
    if (state == 'F') {
      gripperServoBackward();
      delay(response_time);

    }

    //Close Claw Grip
    if (state == 'f') {
      gripperServoForward();
      delay(response_time);
    }


  }
}

//Boiler plate function - These functions move the servo motors in a specific direction for a duration.

void gripperServoForward() {
  if (!joint_enabled[CH_GRIPPER]) return;

  if (servo_joint_4_parking_pos_i > servo_joint_4_min_pos) {
    writeJoint(CH_GRIPPER, servo_joint_4_parking_pos_i, servo_joint_4_min_pos, servo_joint_4_max_pos);
    delay(response_time); //Delay the time takee to turn the servo by the given increment
    Serial.println(servo_joint_4_parking_pos_i);
    servo_joint_4_parking_pos_i = clampJoint((long)servo_joint_4_parking_pos_i - servo_joint_4_pos_increment, servo_joint_4_min_pos, servo_joint_4_max_pos);

  }
}

void gripperServoBackward() {
  if (!joint_enabled[CH_GRIPPER]) return;

  if (servo_joint_4_parking_pos_i < servo_joint_4_max_pos) {
    writeJoint(CH_GRIPPER, servo_joint_4_parking_pos_i, servo_joint_4_min_pos, servo_joint_4_max_pos);
    delay(response_time);
    Serial.println(servo_joint_4_parking_pos_i);
    servo_joint_4_parking_pos_i = clampJoint((long)servo_joint_4_parking_pos_i + servo_joint_4_pos_increment, servo_joint_4_min_pos, servo_joint_4_max_pos);

  }

}

void wristServoCW() {
  if (!joint_enabled[CH_WRIST_2]) return;

  if (servo_joint_3_parking_pos_i > servo_joint_3_min_pos) {
    writeJoint(CH_WRIST_2, servo_joint_3_parking_pos_i, servo_joint_3_min_pos, servo_joint_3_max_pos);
    delay(response_time_4);
    Serial.println(servo_joint_3_parking_pos_i);
    servo_joint_3_parking_pos_i = clampJoint((long)servo_joint_3_parking_pos_i - servo_joint_3_pos_increment, servo_joint_3_min_pos, servo_joint_3_max_pos);

  }

}

void wristServoCCW() {
  if (!joint_enabled[CH_WRIST_2]) return;

  if (servo_joint_3_parking_pos_i < servo_joint_3_max_pos) {
    writeJoint(CH_WRIST_2, servo_joint_3_parking_pos_i, servo_joint_3_min_pos, servo_joint_3_max_pos);
    delay(response_time_4);
    Serial.println(servo_joint_3_parking_pos_i);
    servo_joint_3_parking_pos_i = clampJoint((long)servo_joint_3_parking_pos_i + servo_joint_3_pos_increment, servo_joint_3_min_pos, servo_joint_3_max_pos);

  }

}

void wristServo1Forward() {
  if (!joint_enabled[CH_WRIST_1]) return;

  if (servo_joint_2_parking_pos_i < servo_joint_2_max_pos) {
    writeJoint(CH_WRIST_1, servo_joint_2_parking_pos_i, servo_joint_2_min_pos, servo_joint_2_max_pos);
    delay(response_time);
    Serial.println(servo_joint_2_parking_pos_i);

    servo_joint_2_parking_pos_i = clampJoint((long)servo_joint_2_parking_pos_i + servo_joint_2_pos_increment, servo_joint_2_min_pos, servo_joint_2_max_pos);

  }


}

void wristServo1Backward() {
  if (!joint_enabled[CH_WRIST_1]) return;

  if (servo_joint_2_parking_pos_i > servo_joint_2_min_pos) {
    writeJoint(CH_WRIST_1, servo_joint_2_parking_pos_i, servo_joint_2_min_pos, servo_joint_2_max_pos);
    delay(response_time);
    Serial.println(servo_joint_2_parking_pos_i);

    servo_joint_2_parking_pos_i = clampJoint((long)servo_joint_2_parking_pos_i - servo_joint_2_pos_increment, servo_joint_2_min_pos, servo_joint_2_max_pos);

  }

}


void elbowServoForward() {
  if (!joint_enabled[CH_ELBOW]) return;

  if (servo_joint_elbow_parking_pos_i < servo_joint_elbow_max_pos) {
    writeJoint(CH_ELBOW, servo_joint_elbow_parking_pos_i, servo_joint_elbow_min_pos, servo_joint_elbow_max_pos);

    delay(response_time);
    Serial.println(servo_joint_elbow_parking_pos_i);

    servo_joint_elbow_parking_pos_i = clampJoint((long)servo_joint_elbow_parking_pos_i + servo_joint_elbow_pos_increment, servo_joint_elbow_min_pos, servo_joint_elbow_max_pos);

  }
}

void elbowServoBackward() {
  if (!joint_enabled[CH_ELBOW]) return;
  if (servo_joint_elbow_parking_pos_i > servo_joint_elbow_min_pos) {
    writeJoint(CH_ELBOW, servo_joint_elbow_parking_pos_i, servo_joint_elbow_min_pos, servo_joint_elbow_max_pos);

    delay(response_time);
    Serial.println(servo_joint_elbow_parking_pos_i);


    servo_joint_elbow_parking_pos_i = clampJoint((long)servo_joint_elbow_parking_pos_i - servo_joint_elbow_pos_increment, servo_joint_elbow_min_pos, servo_joint_elbow_max_pos);

  }

}

void shoulderServoForward() {
  if (!joint_enabled[CH_SHOULDER]) return;

  if (servo_joint_1_parking_pos_i < servo_joint_1_max_pos) {
    writeJoint(CH_SHOULDER, servo_joint_1_parking_pos_i, servo_joint_1_min_pos, servo_joint_1_max_pos);
    delay(response_time);
    Serial.println(servo_joint_1_parking_pos_i);

    servo_joint_1_parking_pos_i = clampJoint((long)servo_joint_1_parking_pos_i + servo_joint_1_pos_increment, servo_joint_1_min_pos, servo_joint_1_max_pos);

  }

}

void shoulderServoBackward() {
  if (!joint_enabled[CH_SHOULDER]) return;


  if (servo_joint_1_parking_pos_i > servo_joint_1_min_pos) {
    writeJoint(CH_SHOULDER, servo_joint_1_parking_pos_i, servo_joint_1_min_pos, servo_joint_1_max_pos);
    delay(response_time);
    Serial.println(servo_joint_1_parking_pos_i);

    servo_joint_1_parking_pos_i = clampJoint((long)servo_joint_1_parking_pos_i - servo_joint_1_pos_increment, servo_joint_1_min_pos, servo_joint_1_max_pos);

  }
}

//Move the MG995 to a target position servo_base_step_us at a time (smooth, speed set by
//servo_base_step_delay). The target is always clamped to the min/max limits.
void baseMoveTo(long requested) {
  if (!joint_enabled[CH_BASE]) return;
  int target = clampJoint(requested, servo_base_min_pos, servo_base_max_pos);
  while (servo_base_pos_i != target) {
    int d = target - servo_base_pos_i;
    servo_base_pos_i += (abs(d) > servo_base_step_us) ? ((d > 0) ? servo_base_step_us : -servo_base_step_us) : d;
    writeJoint(CH_BASE, servo_base_pos_i, servo_base_min_pos, servo_base_max_pos);
    delay(servo_base_step_delay);
  }
}

void baseRotateLeft() {
  baseMoveTo((long)servo_base_pos_i + (long)servo_base_left_direction * servo_base_pos_increment);
  Serial.println(servo_base_pos_i);
}


void baseRotateRight() {
  baseMoveTo((long)servo_base_pos_i - (long)servo_base_left_direction * servo_base_pos_increment);
  Serial.println(servo_base_pos_i);
}

void wakeUp() {

  //Pre-Program Function - Wake Up Robot on Start

  if (loop_check == 0) {

    //    //Shoulder Raise
    for (Pos = 0; Pos < 10; Pos++)
    {

      writeJoint(CH_ELBOW, legacyPosToUs(Pos), servo_joint_elbow_min_pos, servo_joint_elbow_max_pos);
      delay(response_time_fast);
    }

    //  //Move Elbow Backwards
    for (Pos = 400; Pos > 390; Pos--)
    {

      writeJoint(CH_SHOULDER, legacyPosToUs(Pos), servo_joint_1_min_pos, servo_joint_1_max_pos);

      delay(response_time_fast);
    }

    //Move Wrist 1 Forward
    for (Pos = 10; Pos < 20; Pos++)
    {
      writeJoint(CH_WRIST_1, legacyPosToUs(Pos), servo_joint_2_min_pos, servo_joint_2_max_pos);
      delay(response_time);
    }

    //Move Wrist 2 Backwards
    for (Pos = 380; Pos > 50; Pos--)
    {
      writeJoint(CH_WRIST_2, legacyPosToUs(Pos), servo_joint_3_min_pos, servo_joint_3_max_pos);
      delay(response_time);
    }

    //Move Wrist 2 Backwards
    for (Pos = 50; Pos < 150; Pos++)
    {
      writeJoint(CH_WRIST_2, legacyPosToUs(Pos), servo_joint_3_min_pos, servo_joint_3_max_pos);
      delay(response_time);
    }

    //Move Wrist 1 Forward
    for (Pos = 19; Pos < 100; Pos++)
    {
      writeJoint(CH_WRIST_1, legacyPosToUs(Pos), servo_joint_2_min_pos, servo_joint_2_max_pos);
      delay(response_time);
    }
    loop_check = 0;

  }
}

void flexMotors() {

  //Example Demo Pre-program Function to Make Robot Wake Up (Motor by Motor)

  if (loop_check == 0) {

    delay(action_delay);

    //Move Wrist 1 Forward
    for (Pos = 100; Pos > 10; Pos--)
    {
      writeJoint(CH_WRIST_1, legacyPosToUs(Pos), servo_joint_2_min_pos, servo_joint_2_max_pos);
      delay(10);
    }

    delay(action_delay);

    //Move Wrist 1 Forward
    for (Pos = 10; Pos < 70; Pos++)
    {
      writeJoint(CH_WRIST_1, legacyPosToUs(Pos), servo_joint_2_min_pos, servo_joint_2_max_pos);
      delay(10);
    }

    delay(action_delay);

    baseRotateLeft();
    delay(action_delay);


    //Move Wrist 2 Backwards
    for (Pos = 200; Pos < 380; Pos++)
    {
      writeJoint(CH_WRIST_2, legacyPosToUs(Pos), servo_joint_3_min_pos, servo_joint_3_max_pos);
      delay(10);
    }

    delay(action_delay);

    loop_check = 1;


  }
}
