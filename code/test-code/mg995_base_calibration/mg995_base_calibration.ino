/******************************************************************************
  Description: Single-servo pulse calibration (PCA9685, Adafruit library)
  Default channel is CH0 = the MG995 base. Any one channel can be selected.

  Finds the safe min / max / centre PULSE WIDTH IN MICROSECONDS for ONE servo.
  Values are microseconds, NOT degrees and NOT the old 0-480 library units.

  ONE SERVO AT A TIME: only the selected channel ever receives a pulse. All
  others stay silent (limp). Selecting a channel sends NO pulse until you press
  a move key or 'c'.

  SAFETY: do this with the servo horn REMOVED or the mechanism free to move and
  no linkage that can jam. Power servos from the external 5-6V supply.
  Disconnect the HC-05 from D0/D1 (it shares the USB serial port).

  Open Serial Monitor at 9600 baud, line ending "No line ending".
    0-5   : select channel (CH0 base ... CH5 gripper); no pulse sent
    d / a : +10 / -10 us     w / s : +1 / -1 us
    c     : go to centre (START_US) and start pulsing
    p     : print channel + current pulse
    x     : stop pulsing this channel (servo goes limp)
  Start at 1500 and creep outward. The pulse is clamped to the
  CAL_MIN_US..CAL_MAX_US envelope (1000-2000 by default). Widen it ONLY
  deliberately, in small steps, with the horn off. Stop when the servo
  hums/buzzes against its end stop or the mechanics bind, then back off ~20 us.
  Write down min, max, and the value where the joint points straight ahead
  (parking). Copy them into the matching limits in arduino_robot_arm_code.ino
  (servo_base_* for CH0, servo_joint_*_min/max_pos for the others).
******************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2CAdd 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2CAdd);

/* Keep identical to arduino_robot_arm_code.ino. */
const float    SERVO_FREQ_HZ = 50;
const uint32_t OSC_FREQ_HZ   = 25000000UL;  // nominal; measure to calibrate
const int CAL_MIN_US = 1000;                // test envelope
const int CAL_MAX_US = 2000;
const int START_US   = 1500;

int ch = 0;          // selected channel (default CH0 = MG995 base)
int pos = START_US;
bool active = false; // true once a pulse has been sent on `ch`

void setup() {
  Serial.begin(9600);
  if (!pwm.begin()) {
    Serial.println(F("ERROR: PCA9685 not found on I2C. Halted."));
    while (true) {}
  }
  for (uint8_t c = 0; c < 16; c++) pwm.setPWM(c, 0, 4096);  // all channels full-OFF: no pulse at startup
  pwm.setOscillatorFrequency(OSC_FREQ_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  Serial.println(F("Calibration. 0-5=channel d/a=+-10 w/s=+-1 c=centre p=print x=stop"));
  Serial.println(F("CH0 selected, no pulse yet."));
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= '0' && c <= '5') {
      if (active) pwm.setPWM(ch, 0, 4096);  // previous channel: full OFF
      active = false;
      ch = c - '0';
      pos = START_US;
      Serial.print(F("CH")); Serial.print(ch); Serial.println(F(" selected, no pulse yet."));
      return;
    }
    if (c == 'x') {
      pwm.setPWM(ch, 0, 4096);              // full OFF = no pulse
      active = false;
      Serial.println(F("stopped"));
      return;
    }
    if (c == 'd') pos += 10;
    else if (c == 'a') pos -= 10;
    else if (c == 'w') pos += 1;
    else if (c == 's') pos -= 1;
    else if (c == 'c') pos = START_US;
    else if (c != 'p') return;

    pos = constrain(pos, CAL_MIN_US, CAL_MAX_US);
    if (c != 'p' || active) {
      pwm.writeMicroseconds(ch, pos);
      active = true;
    }
    Serial.print(F("CH")); Serial.print(ch); Serial.print(F(" "));
    Serial.print(pos); Serial.println(F(" us"));
  }
}
