#ifndef SensorArrayAnalog_H
#define SensorArrayAnalog_H

class SensorArrayAnalog {

  private:
    bool _calDone = 0;
    unsigned char _calSpeed = 40;
    unsigned int _cvSteps = 250, _ccvSteps = 450;
    unsigned int _calSteps = 0;
    unsigned char i = 0;
    unsigned int tmp_value;
    unsigned int sensors_sum;
    unsigned int last_pos;
    unsigned char _emitterPin, _sensorTreshold, _lineTreshold, _numSensors;
    //--------- Common ------------
    char tmp_str[32];
    unsigned int sensor_values[8];
    unsigned int sensor_calmin[8];
    unsigned int sensor_calmax[8];
    unsigned int sensor_denom[8];
  
  public:
    int leftSpeed;
    int rightSpeed;
    unsigned char sensors[8];

    //SensorArrayAnalog();
    SensorArrayAnalog(const unsigned char* pins, unsigned char numSensors = 8, unsigned char emitterPin = 255);
    bool calibrationDone();
    void calibrationReset();
    void setMotion(unsigned int cvSteps, unsigned int ccvSteps, unsigned char calSpeed);
    unsigned int read_position();
    void saveData();
    void loadData();
    void lineTreshold(unsigned char lineThreshold=40);
    void sensorTreshold(unsigned char sensorThreshold=5);
};





#endif
