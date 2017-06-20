#ifndef DCMotorBot_H
#define DCMotorBot_H

class DCMotorBot {
  private:
    unsigned char _limit_pwm = 255;
    char _en1, _en2;
    char _dir1_A, _dir2_A, _dir1_B, _dir2_B;
    bool _reverse1 = 0, _reverse2 = 0;
  public:
    void setSpeeds(int, int);
    void setEnablePins(char, char);
    void setControlPins(char dir1_A = -1, char dir2_A = -1, char dir1_B = -1, char dir2_B = -1);
    void setReverseDir(bool reverse1 = 0, bool reverse2 = 0);
    void setLimit(unsigned char limit_pwm = 255);
    
    
};
#endif

