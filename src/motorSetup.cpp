#include "motorSetup.h"

#define SERVOMIN 150
#define SERVOMAX 600

const uint8_t motorChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};

static void setServoAngle(Adafruit_PWMServoDriver &pwm, uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void setAllMotorsTo90(Adafruit_PWMServoDriver &pwm) {
  for (uint8_t i = 0; i < 8; i++) {
    setServoAngle(pwm, motorChannels[i], 90);
  }
}
