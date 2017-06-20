#include <Arduino.h>

#define LEFT_PWM    9
#define RIGHT_PWM   10
#define LEFT_DIR    6
#define RIGHT_DIR   7

//--------------- LEDS -----------
#define GREEN_LED  13

//-------------- Optrons ---------
#define EMITTER_PIN       5

//--------------  Buttons -----------
#define BUTT1     8
#define BUTT2     12

/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
signed long position;
int dir;
unsigned int sensors_sum;
signed int error;
signed int lastError;
signed int left_pwm;
signed int right_pwm;
signed int motorSpeed;
//int stojnost;
int line_position = 0;

//Maze variables
unsigned char found_left = 0;
unsigned char found_straight = 0;
unsigned char found_right = 0;

//================= SENSOR
#define LINE_TRESHOLD   30    // Ниво под което се смята че няма линия под сензора
#define SENSOR_TRESHOLD 5    // Филтриране на шумове от сензорите


//=================  CALIBRATE ==========================
unsigned int sensor_values[SENSORS_NR];
unsigned int sensor_calmin[SENSORS_NR];
unsigned int sensor_calmax[SENSORS_NR];
unsigned int sensor_denom[SENSORS_NR];

//--------- Common ------------
char tmp_str[64];

// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your
// maze design does not exceed the bounds of the array.
char path[100] = "";
char last_path = "";
unsigned char path_length = 0; // the length of the path

void printSensor(void);

void init_robot(void) {
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // --- Изходи  ---
  pinMode(GREEN_LED, OUTPUT);
  pinMode(EMITTER_PIN, OUTPUT);

  // --- Входове ---
  pinMode(BUTT1, INPUT_PULLUP);
  pinMode(BUTT2, INPUT_PULLUP);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  digitalWrite(EMITTER_PIN, LOW);

}

/* -----------------------------------------------------------------------------------
    Two way PWM speed control. If speed is >0 then motor runs forward.
    Else if speed is < 0 then motor runs backward
  ------------------------------------------------------------------------------------ */
void left_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;

  if (motor_speed < 0) {
    digitalWrite(LEFT_DIR, LOW);
  }
  else {
    digitalWrite(LEFT_DIR, HIGH);
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  analogWrite(LEFT_PWM, motor_speed);
}

//---------------------------------------
void right_motor_speed(signed int motor_speed) {
  if (motor_speed == 0)
    motor_speed = 1;
  if (motor_speed > 0) {
    digitalWrite(RIGHT_DIR, LOW);
  }
  else {
    digitalWrite(RIGHT_DIR, HIGH);
  }
  motor_speed = abs(motor_speed);
  if (motor_speed > 255)
    motor_speed = 255;

  analogWrite(RIGHT_PWM, motor_speed);
}

//===============================  CALIBRATE ======================================
//-------------------

void calibrate(void) {
  int i, sens;
  int center_pos;
  unsigned int tmp_value;


  //--- Preset the arrays ---
  for (i = 0; i < SENSORS_NR; i++)
    (sensor_calmin)[i] = 1024;    // Maximum ADC value

  for (i = 0; i < SENSORS_NR; i++) {
    (sensor_calmax)[i] = 0;     // Minimum ADC value
    tmp_value = analogRead(sensors[i]);
  }

  //--- Turn right ---
  left_motor_speed(ACQUIRE_SPEED);    // forward
  right_motor_speed(-ACQUIRE_SPEED);    // backward
  digitalWrite(EMITTER_PIN, HIGH);
  delayMicroseconds(400);
  for (i = 0; i < 700; i++) {
    //----- Find minimum and maximum values for all sensors -----
    for (sens = 0; sens < SENSORS_NR; sens++) {

      delayMicroseconds(100);
      tmp_value = analogRead(sensors[sens]) / 2;
      if (tmp_value < sensor_calmin[sens])
        sensor_calmin[sens] = tmp_value;
      if (tmp_value > sensor_calmax[sens])
        sensor_calmax[sens] = tmp_value;
    }

    if (i == 250) {   // --- turn left  ---
      left_motor_speed(-ACQUIRE_SPEED);
      right_motor_speed(ACQUIRE_SPEED);
    }
  }
  //-------   Calculate calibration  denom --------
  for (sens = 0; sens < SENSORS_NR; sens++) {
    sensor_denom[sens] = (sensor_calmax[sens] - sensor_calmin[sens]) / 10;

    sprintf(tmp_str, " min %d - max %d", sensor_calmin[sens], sensor_calmax[sens]);
    Serial.println(tmp_str);
  }

  //---------- Go back to the line ----------
  left_motor_speed(ACQUIRE_SPEED);
  right_motor_speed(-ACQUIRE_SPEED);
  do {
    tmp_value = analogRead(sensors[3]) / 2;
    center_pos = ((tmp_value - sensor_calmin[3]) * 10) / sensor_denom[3];// Center sonsor position in the array = 2
    delay(10);
  } while (center_pos < 80);

  // --- Stop the motors ---
  left_motor_speed(0);
  right_motor_speed(0);
  digitalWrite(EMITTER_PIN, LOW);
}


//============================= Read sensors  and scale =====================
unsigned int read_position(void) {
  unsigned char on_line;
  static unsigned int last_pos;
  unsigned char sens;
  unsigned int tmp_value;
  signed long pos;

  pos = 0;
  sensors_sum = 0;
  on_line = 0;
  digitalWrite(EMITTER_PIN, HIGH);
  delayMicroseconds(730);    // Wait for lighting
  //-------------- Read sensors ------------
  for (sens = 0; sens < 6; sens++) {
    delayMicroseconds(45);
    tmp_value = analogRead(sensors[sens]) / 2;

    //--------- Validate ----------
    if (tmp_value < sensor_calmin[sens])
      tmp_value = sensor_calmin[sens];
    if (tmp_value > sensor_calmax[sens])
      tmp_value = sensor_calmax[sens];

    //-------- Calibrate ----------
    sensor_values[sens] = ((tmp_value - sensor_calmin[sens]) * 10)
                          / sensor_denom[sens];

    //----------- Noise filtering ----------
    if (sensor_values[sens]  < SENSOR_TRESHOLD)
      sensor_values[sens] = 0;


    // The estimate position is made using a weighted average of the sensor indices
    // multiplied by 100,  The formula is:
    //
    //    100*value0 + 200*value1 + 300*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

    pos += sensor_values[sens] * ((sens + 1) * 100);
    sensors_sum += sensor_values[sens];

    //--- line presens check and count ---
    if (sensor_values[sens] > LINE_TRESHOLD)
      on_line += 1;
    //--- line presens check and count ---

  }

  digitalWrite(EMITTER_PIN, LOW);

  /*Serial.print(last_pos);
    Serial.print(", ");
    Serial.print(on_line);
    printSensor();
  */
  if (!on_line) {
    // If it last read to the left of center, return 0.
    //if (last_pos < (SENSORS_NR - 1) * 100 / 2) {
    if (last_pos < 250) {
      last_pos = 90;
      // If it last read to the right of center, return the max.
    }
    else if (last_pos > 450 ) {
      last_pos = (SENSORS_NR * 100) + 10;
    } else {
      last_pos = 350;   // center pos
    }
  }
  else {
    if (on_line > 2) {
      tmp_value = sensor_values[0] + sensor_values[1];
      if (  tmp_value > 90 )
        last_pos = 100;
      tmp_value = sensor_values[SENSORS_NR - 2] + sensor_values[SENSORS_NR - 1];
      if  ( tmp_value > 90 )
        last_pos = (SENSORS_NR * 100);

    } else
      last_pos = pos / sensors_sum;
  }

  return last_pos;
}
void printSensor(void) {
  for (int j = 0; j < SENSORS_NR; j++) {
    Serial.print("[");
    Serial.print(sensor_values[j]);
    Serial.print("]");
  }
  Serial.println();

}

