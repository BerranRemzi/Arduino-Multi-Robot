#include <DCMotorBot.h>
#include <SensorArrayAnalog.h>

//-------------- Inputs -----------
#define BUTTON_L    8
#define BUTTON_R    12
#define RECV_PIN    11

//-------------- Outputs -----------
#define LEFT_PWM    9
#define LEFT_DIRA   6
#define LEFT_DIRB   -1

#define RIGHT_PWM   10
#define RIGHT_DIRA  7
#define RIGHT_DIRB  -1

#define GREEN_LED 13

//-------------------------- Sensors and position -------------------------
#define EMITTER_PIN 5  // emitter is controlled by digital pin 5
#define SENSORS_NR  6
const unsigned char sensors[SENSORS_NR] = {A5, A4, A3, A2, A1, A0 }; //left-right

//---------------- Състояние ---------
#define FORWARD    1
#define LEFT       2
#define RIGHT      3
#define BACK       4

///////////////////////////////////////////////////////////////////////////
//----------------------------------- PID ---------------------------------
///////////////////////////////////////////////////////////////////////////
#define BREAKING

/*
  //----------------  35mm Wheels / 0.36 / 6 / 115 / ---------------
  #define LINE_TRESHOLD   45    // Ниво под което се смята че няма линия под сензора
  #define SENSOR_TRESHOLD 5    // Филтриране на шумове от сензорите

  //------------------  1000 rpm -------------
  #define KP             0.80
  #define KD             20
  #define MAX_SPEED      70
  #define ACQUIRE_SPEED  35//35

  //--------
  #define SLOW_SPEED     MAX_SPEED - MAX_SPEED/4  // Стабилизирне след завръщане на линията
  #define BREAK_LEVEL  MAX_SPEED * 0.3
  //#define BREAK_LEVEL     25
  #define BREAK_SPEED      -35

  //  Скорости на моторите при изпуснат завой - търсене на линията в ляво или дясно
  // Two speed for turn - for making right turn radius

  #define TURN_SPEED_HIGH     MAX_SPEED + MAX_SPEED/8
  #define TURN_SPEED_LOW      -80

  // Лява и дясна позиция, след които се изпуска линията
  #define TURN_ERROR_LEFT    100    //  100 minimum
  #define TURN_ERROR_RIGHT   600    // 600 maximum
*/
//----------------  40mm Wheels ---------------
#define LINE_TRESHOLD   45    // Ниво под което се смята че няма линия под сензора
#define SENSOR_TRESHOLD 5    // Филтриране на шумове от сензорите

//------------------  1000 rpm -------------
#define KP             0.4
#define KD             10
#define MAX_SPEED      70
#define ACQUIRE_SPEED  35

//---------------
#define SLOW_SPEED     MAX_SPEED - MAX_SPEED/10  // Стабилизирне след завръщане на линията
#define BREAK_LEVEL    MAX_SPEED * 0.4
#define BREAK_SPEED    -35

//  Скорости на моторите при изпуснат завой - търсене на линията в ляво или дясно
// Two speed for turn - for making right turn radius

#define TURN_SPEED_HIGH     MAX_SPEED + MAX_SPEED/10
#define TURN_SPEED_LOW      -30

// Лява и дясна позиция, след които се изпуска линията
#define TURN_ERROR_LEFT    100    // 100 minimum
#define TURN_ERROR_RIGHT   600    // 600 maximum

/////////////////////////////////////////////////////////////////////////
//------- Global Variables -----
int position;
int dir;
unsigned int sensors_sum;
signed int error;
signed int lastError;
signed int left_pwm;
signed int right_pwm;
signed int motorSpeed;
int derivate;

SensorArrayAnalog sensor(sensors, SENSORS_NR, EMITTER_PIN);
DCMotorBot motors;

void init_ports() {
  // --- Входове ---
  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(BUTTON_R, INPUT_PULLUP);
  pinMode(RECV_PIN, INPUT);

  // --- Изходи  ---
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIRA, OUTPUT);
  pinMode(LEFT_DIRB, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIRA, OUTPUT);
  pinMode(RIGHT_DIRB, OUTPUT);
  pinMode(EMITTER_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(EMITTER_PIN, LOW);
};

void setup() {
  init_ports();
  Serial.begin(9600);
  motors.setReverseDir(1, 0);
  motors.setEnablePins(9, 10);
  motors.setControlPins(6, 7);
  motors.setLimit(230);

  sensor.setMotion(250, 450, ACQUIRE_SPEED);
  sensor.lineTreshold(LINE_TRESHOLD);
  sensor.sensorTreshold(SENSOR_TRESHOLD);
  while (digitalRead (BUTTON_L) == HIGH);
  do {
    motors.setSpeeds(sensor.leftSpeed, sensor.rightSpeed);
  } while (sensor.calibrationDone() == false);

  Serial.println("Calibration DONE...");


  while (digitalRead (BUTTON_L) == HIGH);
  delay(500);

  dir = FORWARD;
  lastError = 0;
}

void loop() {
  //---- Прочитане на текущата позиция и пресмятане на грешката ----
  position = sensor.read_position();
  error = position - 350;

  switch (dir) {
    case FORWARD:    //Движение направо
    default:

      if (position < TURN_ERROR_LEFT) {  //Ако отклонението е много голямо преминаване в състояние "Завой наляво"
#ifdef BREAKING
        //--- Break ---
        motors.setSpeeds(BREAK_SPEED, BREAK_SPEED);
        delay(15);
#endif
        // --- Turn ---
        motors.setSpeeds(TURN_SPEED_LOW, TURN_SPEED_HIGH);
        //Serial.println(">>>>>>>>>>>>>>>>>>>>>>>");
        dir = LEFT;
        break;
      }

      if (position > TURN_ERROR_RIGHT) {  //Ако отклонението е много голямо преминаване в състояние "Завой надясно"
#ifdef BREAKING
        //--- Break ---
        motors.setSpeeds(BREAK_SPEED, BREAK_SPEED);
        delay(15);
#endif
        // --- Turn ---
        motors.setSpeeds(TURN_SPEED_HIGH, TURN_SPEED_LOW);
        //Serial.println("<<<<<<<<<<<<<<<<<<<<<");
        dir = RIGHT;
        break;
      }


      //-------------PD закон за управление---------------
      derivate =  (error - lastError) * KD;
      motorSpeed = KP * error + derivate;
      lastError = error;

      left_pwm = MAX_SPEED + motorSpeed;
      right_pwm = MAX_SPEED - motorSpeed;

      // ако MAX_SPEED > 140 трябва да се спира единия мотор.
      if (left_pwm < BREAK_LEVEL)
        left_pwm = BREAK_SPEED;
      if (right_pwm < BREAK_LEVEL )
        right_pwm = BREAK_SPEED;


      //---- Задаване на пресметнатата скорост на моторите ----
      motors.setSpeeds(left_pwm, right_pwm);
      break;

    case LEFT:     // Завой наляво при голямо отклонение - търсене на линия
      if (position > TURN_ERROR_LEFT + 50) {    //Maximum is 100%
        // Завръщане обратно на линията
        // Спирачен момент в обратна посока (за предотвратяване на заклащането)
        motors.setSpeeds(MAX_SPEED, SLOW_SPEED);
        dir = FORWARD;
        lastError = 0;
        delay(10);
      }
      break;

    case RIGHT:     // Завой надясно при голямо отклонение - търсене на линия
      if (position < TURN_ERROR_RIGHT - 50) {    //Maximum is 100%
        // Завръщане обратно на линията
        // Спирачен момент в обратна посока (за предотвратяване на заклащането)
        motors.setSpeeds(SLOW_SPEED, MAX_SPEED);
        dir = FORWARD;
        lastError = 0;
        delay(10);
      }
      break;

  }
  delayMicroseconds(330);
}
