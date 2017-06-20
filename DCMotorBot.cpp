#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "DCMotorBot.h"

void DCMotorBot::setSpeeds(int left_pwm, int right_pwm) {

  if (left_pwm > 0) {
    digitalWrite(_dir1_A, LOW);
    digitalWrite(_dir1_B, HIGH);
  } else {
    digitalWrite(_dir1_A, HIGH);
    digitalWrite(_dir1_B, LOW);
  }

  if (right_pwm > 0) {
    digitalWrite(_dir2_A, LOW);
    digitalWrite(_dir2_B, HIGH);
  } else {
    digitalWrite(_dir2_A, HIGH);
    digitalWrite(_dir2_B, LOW);
  }
  if (left_pwm < 0) left_pwm = -left_pwm;
  if (right_pwm < 0) right_pwm = -right_pwm;

  if (left_pwm > _limit_pwm) left_pwm = _limit_pwm;
  if (right_pwm > _limit_pwm) right_pwm = _limit_pwm;
  analogWrite(_en1, left_pwm);
  analogWrite(_en2, right_pwm);
}
void DCMotorBot::setEnablePins(char en1, char en2) {
  _en1 = en1;
  _en2 = en2;
}
void DCMotorBot::setControlPins(char dir1_A, char dir2_A, char dir1_B, char dir2_B) {
  if (_reverse1 == 0) {
    _dir1_A = dir1_A;
    _dir1_B = dir1_B;
  } else {
    _dir1_A = dir1_B;
    _dir1_B = dir1_A;
  }
  if (_reverse2 == 0) {
    _dir2_A = dir2_A;
    _dir2_B = dir2_B;
  } else {
    _dir2_A = dir2_B;
    _dir2_B = dir2_A;
  }
}
void DCMotorBot::setReverseDir(bool reverse1, bool reverse2) {
  _reverse1 = reverse1;
  _reverse2 = reverse2;
}
void DCMotorBot::setLimit(unsigned char limit_pwm/*=255*/) {
  _limit_pwm = limit_pwm;
}


