//by Berran Remzi
//Modified: 12.03.2017 13:10

#define leftPWM   9
#define leftDIR   6
#define rightPWM  10
#define rightDIR  7
#define button1   8
#define button2   12
#define LED       13

//#define pinForward 8
//#define pinBack 7
#define pinSpeedForwardBack 6
#define pinFrontLights 13
#define pinBackLights 3
#define pinFrontSteering 10

//Дефиниция за инвертиране на даден двигател
#define invertLeft
//#define invertRight

void set_motors(int left_pwm, int right_pwm) {
#ifdef invertLeft
  if (left_pwm < 0)
#else
  if (left_pwm > 0)
#endif
    digitalWrite(6, LOW);
  else
    digitalWrite(6, HIGH);

#ifdef invertRight
  if (right_pwm < 0)
#else
  if (right_pwm > 0)
#endif
    digitalWrite(7, LOW);
  else
    digitalWrite(7, HIGH);
  if (right_pwm > 255) right_pwm = 255;
  if (left_pwm > 255) left_pwm = 255;
  analogWrite(9, abs(left_pwm));
  analogWrite(10, abs(right_pwm));
}
//#include <Servo.h>
//#include <L293.h>

//L293(pinForward, pinBack, pinFwdBakVel);
//L293 redCar(pinForward,pinBack,pinSpeedForwardBack);
//Servo leftRight;
byte commands[4] = {0x00, 0x00, 0x00, 0x00};
//Variables will be used to determine the frequency at which the sensor readings are sent
//to the phone, and when the last command was received.
unsigned long timer0 = 2000;  //Stores the time (in millis since execution started)
unsigned long timer1 = 0;  //Stores the time when the last sensor reading was sent to the phone
//14 byte payload that stores the sensor readings
byte sensors[14] = {0xee, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc};
//Constant used to caculate the 9V battery voltage (9.04 mV per step)
float stepSize = 9.04;
//The union allows you to declare a customized data type, in this case it can be either
//a float or a byte array of size 4. What we need is to store a float which is 4
//bytes long and retrieve each of the 4 bytes separately.
union u_sensor0 {
  byte a[4];
  float b;
}
sensor0;
union u_sensor1 {
  byte c[4];
  float d;
}
sensor1;
int i = 0;
int steer = 0;

void turbo_initialize() {
  pinMode(5,OUTPUT);
  digitalWrite(5,LOW);
  pinMode(9, OUTPUT);       //D9  leftPWM
  pinMode(6, OUTPUT);       //D6  leftDIR
  pinMode(10, OUTPUT);      //D10 rightPWM
  pinMode(7, OUTPUT);       //D7  rightDIR
  pinMode(8, INPUT_PULLUP); //D8  button1
  pinMode(12, INPUT_PULLUP);//D12 button2
  pinMode(13, OUTPUT);      //D13 LED
}

void setup()
{
  Serial.begin(9600);
  turbo_initialize();
  pinMode(pinFrontLights, OUTPUT);
  pinMode(pinBackLights, OUTPUT);
  //leftRight.attach(pinFrontSteering);

  
}

void loop()
{
  if (Serial.available() == 4) {
    commands[0] = Serial.read();  //Direction
    commands[1] = Serial.read();  //Speed
    commands[2] = Serial.read();  //Angle
    commands[3] = Serial.read();  //Lights and buttons states
    steer = commands[2] - 90;
    /*
      Since the last byte yields the servo's angle (between 0-180), it can never be 255. At times, the two
      previous commands pick up incorrect values for the speed and angle. Meaning that they get the direction
      correct 100% of the time but sometimes get 255 for the speed and 255 for the angle.
    */
    if ((commands[2] <= 0xb4) && ((commands[0] <= 0xf5) && (commands[0] >= 0xf1))) {
      //Make sure that the command received involves controlling the car's motors (0xf1,0xf2,0xf3)
      if (commands[0] <= 0xf3) {
        if (commands[0] == 0xf1) { //Check if the move forward command was received
          set_motors(commands[1] + steer, commands[1] - steer);
        }
        else if (commands[0] == 0xf2) { //Check if the move back command was received
          set_motors(-commands[1] - steer, -commands[1] + steer);
        }
        else { //Check if the stop command was received
          set_motors(steer, -steer);
        }
        //Steer front wheels only if the new angle is not equal to the previous angle
      }
      else if (commands[0] == 0xf5) {
        //Stop everything
        set_motors(0, 0);
        digitalWrite(pinFrontLights, LOW);
        digitalWrite(pinBackLights, LOW);
      }
      else {
        //Here you put the code that will control the tilt pan (commands[0] == 0xf4)
      }
      //Check the front/back lights and other toggles
      //               _______________________________________________
      //command[3] =  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  binary
      //              |_____|_____|_____|_____|_____|_____|_____|_____|
      //Buttons ---->  Front  Back  Horn   A     B     C     D     E
      //Front lights
      if (bitRead(commands[3], 7)) {
        digitalWrite(pinFrontLights, HIGH);
      }
      else {
        digitalWrite(pinFrontLights, LOW);
      }
      //Back lights
      if (bitRead(commands[3], 6)) {
        digitalWrite(pinBackLights, HIGH);
      }
      else {
        digitalWrite(pinBackLights, LOW);
      }
      //Horn (using front lights to test)
      if (bitRead(commands[3], 5)) {
        //digitalWrite(pinFrontLights, HIGH);
      }
      else {
        //digitalWrite(pinFrontLights, LOW);
      }
    }
    else {
      //Resetting the Serial port (clearing the buffer) in case the bytes are not being read in correct order.
      Serial.end();
      Serial.begin(9600);
    }
  }
  else {
    timer0 = millis();  //Get the current time (millis since execution started)
    if ((timer0 - timer1) >= 477) { //Check if it has been 477ms since sensor reading were sent
      Serial.print(commands[0]);
      Serial.print(commands[1]);
      Serial.print(commands[2]);
      Serial.println(commands[3]);

      //Calculate the 9V's voltage by multiplying the step size by the step number (analogRead(0))
      //This value will be in mV, which is why it's multiplied by 0.001 to convert into Volts.

      sensor0.b = (analogRead(7) * stepSize) * 0.001;
      //Break the sensor0 float into four bytes for transmission
      sensors[1] = sensor0.a[0];
      sensors[2] = sensor0.a[1];
      sensors[3] = sensor0.a[2];
      sensors[4] = sensor0.a[3];
      //Get sensor 2's reading
      /*
        sensor1.d = analogRead(1);
        //Break the sensor1 float into four bytes for transmission
        sensors[5] = sensor1.c[0];
        sensors[6] = sensor1.c[1];
        sensors[7] = sensor1.c[2];
        sensors[8] = sensor1.c[3];
        //Get the remaining reading from the analog inputs
        sensors[9] = map(analogRead(2),0,1023,0,255);
        sensors[10] = map(analogRead(3),0,1023,0,255);
        sensors[11] = map(analogRead(4),0,1023,0,255);
        sensors[12] = map(analogRead(5),0,1023,0,255);
      */
      //Send the six sensor readings
      Serial.write(sensors, 14);

      //Store the time when the sensor readings were sent
      timer1 = millis();
    }
  }
}









