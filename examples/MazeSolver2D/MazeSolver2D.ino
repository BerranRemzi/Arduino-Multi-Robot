
//-------------------------- Sensors and position -------------------------
#define SENSORS_NR  6
const unsigned int sensors[SENSORS_NR] = {A5, A4, A3, A2, A1, A0 }; //left-right

#define KP_stop         0.25
#define KD_stop         3
#define KP              0.40
#define KD              10
#define MAX_SPEED       60
#define ACQUIRE_SPEED   35//35
#define STRAIGH_SPEED   40
#define SLOW_SPEED      MAX_SPEED - MAX_SPEED/5  // Стабилизирне след завръщане на линията
#define BREAK_LEVEL     MAX_SPEED * 0.3
#define BREAK_SPEED     -40
#define turnSpeed       45

#define DETECT_TRESHOLD 45

#include "maze.h"

void setup() {
  // put your setup code here, to run once:
  init_robot();
  Serial.begin(9600);

  // Изчакване натискането на бутон
  while (digitalRead (BUTT1) == HIGH);
  for (int j = 0; j < 3; j++) {
    delay(250);
    digitalWrite(GREEN_LED, HIGH);
    delay(250);
    digitalWrite(GREEN_LED, LOW);
  }
  calibrate();
  stop_over_line();
  Serial.println();
  Serial.println("Calibration Complete");
  digitalWrite(GREEN_LED, HIGH);


}

void loop() {
  // put your main code here, to run repeatedly:
  MazeSolve();

  // Store the intersection in the path variable.
  path[path_length] = dir;
  path_length ++;

  // Simplify the learned path.
  simplify_path();
}

// Path simplification.  The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end.  For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length - 2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch (total_angle)
  {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }

  // The path is now two steps shorter.
  path_length -= 2;

} // end simplify_path


void MazeSolve(void) {
  while (1) {
    follow_line();
    //-----------------------------------
    // Check for left and right exits.
    if (sensor_values[5] > DETECT_TRESHOLD) {
      found_right = 1;
#ifdef DEBUG
      Serial.println("Found right >>>>>");
#endif
    }
    if (sensor_values[0] > DETECT_TRESHOLD) {
      found_left = 1;
#ifdef DEBUG
      Serial.println("Found left <<<<<");
#endif
    }
    //-----------------------------------
    left_motor_speed(SLOW_SPEED);
    right_motor_speed(SLOW_SPEED);
    if(last_path=='L'||last_path=='R'||last_path=='B')
      delay(50);
    else delay(25);
    
    line_position = read_position();
    if (sensor_values[1] > DETECT_TRESHOLD || sensor_values[2] > DETECT_TRESHOLD || sensor_values[3] > DETECT_TRESHOLD || sensor_values[4] > DETECT_TRESHOLD) {
      found_straight = 1;
#ifdef DEBUG
      Serial.println("Found straight |||||");
#endif
    }
    // Check for the ending spot.
    // If all six middle sensors are on dark black, we have
    // solved the maze.
    if (sensor_values[0] > DETECT_TRESHOLD && sensor_values[1] > DETECT_TRESHOLD && sensor_values[2] > DETECT_TRESHOLD && sensor_values[3] > DETECT_TRESHOLD && sensor_values[4] > DETECT_TRESHOLD && sensor_values[5] > DETECT_TRESHOLD) {
      left_motor_speed(BREAK_SPEED);
      right_motor_speed(BREAK_SPEED);
      delay(100);
      left_motor_speed(0);
      right_motor_speed(0);
      break;
    }
    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);
#ifdef DEBUG
    Serial.print("Turn : ");
    Serial.println((char)dir);
#endif
    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length ++;
    last_path = dir;
    Serial.print((char)dir);

    // Simplify the learned path.
    simplify_path();
  }

  while (1) {
    left_motor_speed(0);
    right_motor_speed(0);
    // Check for the ending spot.
    // If all six middle sensors are on dark black, we have
    // solved the maze.
    while (sensor_values[0] > DETECT_TRESHOLD && sensor_values[1] > DETECT_TRESHOLD && sensor_values[2] > DETECT_TRESHOLD && sensor_values[3] > DETECT_TRESHOLD && sensor_values[4] > DETECT_TRESHOLD && sensor_values[5] > DETECT_TRESHOLD) {
      line_position = read_position();
      delay(50);
    }
    bool led = LOW;
    // delay to give you time to let go of the robot
    while (digitalRead(BUTT1) == HIGH) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(GREEN_LED, led);
      led = !led;
      delay(75);
    }

    for (int j = 0; j < 3; j++) {
      delay(250);
      digitalWrite(GREEN_LED, HIGH);
      delay(250);
      digitalWrite(GREEN_LED, LOW);
    }

    // Re-run the now solved maze.  It's not necessary to identify the
    // intersections, so this loop is really simple.
    int i;
    for (i = 0; i < path_length; i++) {
      Serial.print((char)path[i]);
    }
    Serial.println();
    for (i = 0; i < path_length; i++)
    {
      // SECOND MAIN LOOP BODY
      follow_line();
      left_motor_speed(SLOW_SPEED);
      right_motor_speed(SLOW_SPEED);
      delay(30);

      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i]);
    }
    // Follow the last segment up to the finish.
    follow_line();
    left_motor_speed(BREAK_SPEED);
    right_motor_speed(BREAK_SPEED);
    delay(100);
    left_motor_speed(0);
    right_motor_speed(0);
    digitalWrite(GREEN_LED, LOW);
    // Now we should be at the finish!  Now move the robot again and it will re-run this loop with the solution again.
  } // end running solved
}   //end MazeSolve

void follow_line() {

  while (1)
  {

    int  derivate;

    found_left = 0;
    found_straight = 0;
    found_right = 0;
    position = read_position();
    error = position - 350;


    //-------------PD закон за управление---------------
    derivate =  (error - lastError) * KD;
    motorSpeed = KP * error + derivate;
    lastError = error;

    left_pwm = MAX_SPEED + motorSpeed;
    right_pwm = MAX_SPEED - motorSpeed;

    if (sensor_values[0] < DETECT_TRESHOLD && sensor_values[1] < DETECT_TRESHOLD && sensor_values[2] < DETECT_TRESHOLD && sensor_values[3] < DETECT_TRESHOLD && sensor_values[4] < DETECT_TRESHOLD && sensor_values[5] < DETECT_TRESHOLD) {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if (sensor_values[0] > DETECT_TRESHOLD || sensor_values[5] > DETECT_TRESHOLD)
    {
      // Found an intersection.
      return;
    }

    //---- Задаване на пресметнатата скорост на моторите ----
    left_motor_speed(left_pwm);
    right_motor_speed(right_pwm);

    delayMicroseconds(330);
  }
}

char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right) {

  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
} // end select_turn

// Turns to the sent variable of
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back)
// Tune 'turnSpeed' at declaration
void turn(char dir) {

  switch (dir)
  {
    // Turn left 90deg
    case 'L':
      left_motor_speed(BREAK_SPEED);
      right_motor_speed(BREAK_SPEED);
      delay(120);
      left_motor_speed(-turnSpeed);
      right_motor_speed(turnSpeed);

      line_position = read_position();
      while (sensor_values[0] < DETECT_TRESHOLD) // wait for outer most sensor to find the line
      {
        line_position = read_position();
      }
      stop_over_line();
      // stop both motors
      // stop right motor first to better avoid over run
      left_motor_speed(0);
      right_motor_speed(0);
      break;

    // Turn right 90deg
    case 'R':
      left_motor_speed(BREAK_SPEED);
      right_motor_speed(BREAK_SPEED);
      delay(120);
      left_motor_speed(turnSpeed);
      right_motor_speed(-turnSpeed);

      line_position = read_position();

      while (sensor_values[5] < DETECT_TRESHOLD) // wait for outer most sensor to find the line
      {
        line_position = read_position();
      }

      stop_over_line();

      // stop both motors
      left_motor_speed(0);
      right_motor_speed(0);
      break;

    // Turn right 180deg to go back
    case 'B':
      left_motor_speed(BREAK_SPEED);
      right_motor_speed(BREAK_SPEED);
      delay(120);
      left_motor_speed(turnSpeed);
      right_motor_speed(-turnSpeed);

      line_position = read_position();

      while (sensor_values[0] < DETECT_TRESHOLD && sensor_values[5] < DETECT_TRESHOLD) // wait for outer most sensor to find the line
      {
        line_position = read_position();
      }

      stop_over_line();
      // stop both motors
      left_motor_speed(0);
      right_motor_speed(0);
      break;

    // Straight ahead
    case 'S':
      // do nothing
      break;
  }
} // end turn

void stop_over_line(void) {
  int  derivate;
  lastError = 0;

  for (int i = 0; i < 100; i++) {

    position = read_position();
    error = position - 350;

    //-------------PD закон за управление---------------
    derivate =  (error - lastError) * KD_stop;
    motorSpeed = KP_stop * error + derivate;
    lastError = error;

    left_pwm = motorSpeed;
    right_pwm = - motorSpeed;
    //---- Задаване на пресметнатата скорост на моторите ----
    left_motor_speed(left_pwm);
    right_motor_speed(right_pwm);

    delayMicroseconds(330);
  }
  left_motor_speed(0);
  right_motor_speed(0);
}


