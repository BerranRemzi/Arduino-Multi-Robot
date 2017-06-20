#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SensorArrayAnalog.h"

SensorArrayAnalog::SensorArrayAnalog(const unsigned char* pins, unsigned char numSensors, unsigned char emitterPin) {
  unsigned char sens = 0;
  _emitterPin = emitterPin;
  _numSensors = numSensors;
  _calSteps = 0;
  for (sens = 0; sens < _numSensors; sens++) {
    sensors[sens] = pins[sens];
  }
  //--- Preset the arrays ---
  for (i = 0; i < _numSensors; i++)
    sensor_calmin[i] = 4095;    // Maximum ADC value

  for (i = 0; i < _numSensors; i++) {
    sensor_calmax[i] = 0;     // Minimum ADC value
    //tmp_value = analogRead(pins[i]);
  }
  i = 0;
  //SensorArrayAnalog::init(pins, numSensors, emitterPin);
}


bool SensorArrayAnalog::calibrationDone() {
  unsigned char sens = 0;
  int center_pos;
  unsigned int tmp_value;
  if (this->_calSteps == 0) {

    digitalWrite(_emitterPin, HIGH);
    delayMicroseconds(400);
    leftSpeed = this->_calSpeed;
    rightSpeed = -this->_calSpeed;
  }
  if (_calSteps == this->_cvSteps) {
    leftSpeed = -this->_calSpeed;
    rightSpeed = this->_calSpeed;
  }




  if (this->_calSteps < this->_cvSteps + this->_ccvSteps)
  {
    //digitalWrite(_emitterPin, HIGH);
    //----- Find minimum and maximum values for all sensors -----
    for (sens = 0; sens < _numSensors; sens++) {
      delayMicroseconds(100);
      tmp_value = analogRead(sensors[sens]) / 2;
      if (tmp_value < sensor_calmin[sens])
        sensor_calmin[sens] = tmp_value;
      if (tmp_value > sensor_calmax[sens])
        sensor_calmax[sens] = tmp_value;
    }
    this->_calSteps++;
  }
  else if (this->_calSteps == this->_cvSteps + this->_ccvSteps)
  {
    //-------   Calculate calibration  denom --------
    for (sens = 0; sens < _numSensors; sens++) {
      sensor_denom[sens] = (sensor_calmax[sens] - sensor_calmin[sens]) / 10;
      sprintf(tmp_str, " min %d - max %d", sensor_calmin[sens], sensor_calmax[sens]);
      Serial.println(tmp_str);

    }
    this->_calSteps++;
  }
  else if (this->_calSteps > this->_cvSteps + this->_ccvSteps) {
    //---------- Go back to the line ----------
    leftSpeed = this->_calSpeed;
    rightSpeed = -this->_calSpeed;
    tmp_value = analogRead(sensors[3]) / 2;
    center_pos = ((tmp_value - sensor_calmin[3]) * 10) / sensor_denom[3];// Center sonsor position in the array = 2

    if (this->_calSteps > this->_cvSteps + this->_ccvSteps + 1) {
      return true;
    }

    if (center_pos > 80) {
      // --- Stop the motors ---
      leftSpeed = 0;
      rightSpeed = 0;
      digitalWrite(_emitterPin, LOW);
      this->_calSteps++;
    } else {
      //---------- Go back to the line ----------
      leftSpeed = this->_calSpeed;
      rightSpeed = -this->_calSpeed;
    }
    delay(10);
  }
  return false;
}
void SensorArrayAnalog::calibrationReset() {
  _calSteps = 0;
}

void SensorArrayAnalog::setMotion(unsigned int cvSteps, unsigned int ccvSteps, unsigned char calSpeed) {
  this->_cvSteps = cvSteps;
  this->_ccvSteps = ccvSteps;
  this->_calSpeed = calSpeed;
}

unsigned int SensorArrayAnalog::read_position() {
  unsigned char on_line = 0;
  long pos = 0;
  sensors_sum = 0;
  digitalWrite(_emitterPin, HIGH);
  delayMicroseconds(730);    // Wait for lighting
  //-------------- Read sensors ------------
  for (i = 0; i < this->_numSensors; i++) {
    delayMicroseconds(45);
    tmp_value = analogRead(sensors[i]) / 2;

    //--------- Validate ----------
    if (tmp_value < sensor_calmin[i])
      tmp_value = sensor_calmin[i];
    if (tmp_value > sensor_calmax[i])
      tmp_value = sensor_calmax[i];

    //-------- Calibrate ----------
    sensor_values[i] = ((tmp_value - sensor_calmin[i]) * 10)
                       / sensor_denom[i];

    //----------- Noise filtering ----------
    if (sensor_values[i]  < _sensorTreshold)
      sensor_values[i] = 0;


    // The estimate position is made using a weighted average of the sensor indices
    // multiplied by 100,  The formula is:
    //
    //    100*value0 + 200*value1 + 300*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

    pos += sensor_values[i] * ((i + 1) * 100);
    sensors_sum += sensor_values[i];

    //--- line presens check and count ---
    if (sensor_values[i] > _lineTreshold)
      on_line += 1;
    //--- line presens check and count ---

  }

  digitalWrite(_emitterPin, LOW);

  if (!on_line) {
    // If it last read to the left of center, return 0.
    //if (last_pos < (_numSensors - 1) * 100 / 2) {
    if (last_pos < 250) {
      last_pos = 90;
      // If it last read to the right of center, return the max.
    }
    else if (last_pos > (_numSensors * 100)-150 ) {
      last_pos = (_numSensors * 100) + 10;
    } else {
      last_pos = 350;   // center pos
    }
  }
  else {
    if (on_line > 2) {
      tmp_value = sensor_values[0] + sensor_values[1];
      if (  tmp_value > 90 )
        last_pos = 100;
      tmp_value = sensor_values[_numSensors - 2] + sensor_values[_numSensors - 1];
      if  ( tmp_value > 90 )
        last_pos = (_numSensors * 100);

    } else
      last_pos = pos / sensors_sum;
  }

  return last_pos;
}

void SensorArrayAnalog::saveData() {

}
void SensorArrayAnalog::loadData() {

}

void SensorArrayAnalog::lineTreshold(unsigned char lineTreshold){
  _lineTreshold = lineTreshold;
}

void SensorArrayAnalog::sensorTreshold(unsigned char sensorTreshold){
  _sensorTreshold = sensorTreshold;
}

