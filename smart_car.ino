
/*
   Smart car

  Ramon A. Delgado
*/


#include <Servo.h>
#include "NewPing.h"
#include <IRremote.h>



/* Pins on the Arduino Sensor Shield v5.0 */
#define LED_RED_PIN       13
#define LED_YELLOW_PIN    12

#define SONAR_SERVO_PIN    4
#define TRIGGER_PIN     A0
#define ECHO_PIN        A1

#define IR_RECEIVER_PIN    3

// Motor A - right side
#define MOTOR_A_ENABLE_PIN 10
#define IN_1_PIN           9
#define IN_2_PIN           8

// Motor B - left side
#define MOTOR_B_ENABLE_PIN 5
#define IN_3_PIN           7
#define IN_4_PIN           6


#define MAX_DISTANCE    200
#define FRONT_ANGLE    80
#define WHEELS_CORRECTION 0

enum OperationMode {
  self_driving,
  ir_remote,
  bluetooth_remote
} currentMode;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myServo;
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results irResults;


const int triggerDistance = 15;
const int triggerDistanceFar = 30;


/* Variables  */

//int angleList[] = {40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140}; // Angles to be scanned, 90 is the front
int angleList[] = {70, 90, 110};
//int angleList[] = {90};
//int nAngle = 11;                    // number of elements in angleList
//int nAngle = 1;
int nAngle = 3;
double forIncrement = -1;          // moves the servo forwards or barckwrds depending on the sign.
unsigned long irLastCheck;
int backwardsCounter = 0;

double speedEstimate = 0;
double previousFrontDistance = 0;
unsigned long previousFrontTime;
bool isMovingStraight = false;
bool wasMovingStraight = false;
bool isMoving = false;
bool wasMoving = false;
int stallCounter = 0;
int missingMeasurements = 0;



void setup()
{

  currentMode = self_driving;
  //Distance sensor
  pinMode (TRIGGER_PIN, OUTPUT);
  pinMode (ECHO_PIN, INPUT);

  // Servo Motor for sonar
  myServo.attach(SONAR_SERVO_PIN);  // Attaches the Servo to the Servo Object
  myServo.write(FRONT_ANGLE); // Set servo to mid point

  //Serial
  Serial.begin(9600);
  Serial.println("==Ultrasonic rotating sonar ==");

  //Infrared sensor
  irrecv.enableIRIn();
  irLastCheck = millis();

  // set all the motor control pins to outputs
  pinMode(MOTOR_A_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_B_ENABLE_PIN, OUTPUT);
  pinMode(IN_1_PIN, OUTPUT);
  pinMode(IN_2_PIN, OUTPUT);
  pinMode(IN_3_PIN, OUTPUT);
  pinMode(IN_4_PIN, OUTPUT);

  // Setup LED
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_YELLOW_PIN, 0);
}

void loop()
{
  int minDistance = MAX_DISTANCE;
  double avgDistance = 0;
  int minIndex = 0;
  int frontDistance = 0;
  int distanceList[nAngle];


  //Get Infrared data
  processInfraredInstructions();

  /* Handle Self driving Mode with obstacle avoidance*/
  if ( currentMode == self_driving ) {

    frontDistance = scan(FRONT_ANGLE); // measure front distance

    updateSpeedEstimates(frontDistance);

    handleStall(); // check and manage when the robot stalls

    if ((frontDistance <= triggerDistanceFar)) {
      /* if an object is close take furthe measurements to determinate
        which direction to go*/
      moveStop();
      takeDistanceMeasurements(distanceList , minDistance, minIndex, avgDistance, frontDistance);


      /*  handle the case when the robot has two walls that intersect
        with an angle that is less than 90 degrees.   */
      if (backwardsCounter > 0) {
        switch (random(0, 1)) {
          case 0:
            moveBackwardsRight(255);
            break;
          case 1:
            moveBackwardsLeft(255);
            break;
//          default:
//            moveBackwards(255);
//            break;
        }
        delay(random(5, 20) * 10);
      }

      double rightAngle = wallAngle(distanceList[0], frontDistance, abs(angleList[0] - 90));
      double leftAngle = wallAngle(distanceList[nAngle - 1], frontDistance, abs(angleList[nAngle - 1] - 90));
      double measurementAngle = max(abs(angleList[0] - 90), abs(angleList[nAngle - 1] - 90));

      /* If the Robot is too close to a wall, take appropriate actions*/
      if (minDistance <= triggerDistance) {

        if (max(rightAngle, leftAngle) < 80) {
          if (backwardsCounter <= 100) {
            backwardsCounter = backwardsCounter + 10;
          } else {
            backwardsCounter = 0;
            switch (random(1, 5)) {
              case 1:
                moveBackwardsLeft(255);
                break;
              case 2:
                moveBackwardsRight(255);
                break;
              case 3:
                moveRight(255);
                break;
              case 4:
                moveLeft(255);
                break;
              default:
                moveForward(255);
            }
            delay(random(3, 8) * 100);
          }
          moveBackwards(255);
          switch (random(0, 3)) {
          case 0:
            moveBackwardsRight(255);
            break;
          case 1:
            moveBackwardsLeft(255);
            break;
          default:
            moveBackwards(255);
            break;
        }
        }
        else {
          backwardsCounter = backwardsCounter + 1;
          if (leftAngle < rightAngle) {
            moveBackwardsLeft(255);
          } else {
            moveBackwardsRight(255);
          }
          delay(random(1, 6) * 100);
        }

      }
      else {
        backwardsCounter = max(backwardsCounter - 1, 0);
        if (backwardsCounter == 0) {
          if (rightAngle > leftAngle) {
            moveForwardRight(255);
          }
          else {
            moveForwardLeft(255);
          }
          delay(random(1, 4) * 50);
        } else
          delay(random(1, 4) * 50);
      }
    }
    else {
      backwardsCounter = max(backwardsCounter - 1, 0);
      if (backwardsCounter == 0)
        moveForward(255);
    }

    /* From time to time do a random movement*/
    if (random(1, 300) <= 1) {
      switch (random(1, 6)) {
        case 1:
          moveBackwardsLeft(255);
          break;
        case 2:
          moveBackwardsRight(255);
          break;
        case 3:
          moveRight(255);
          break;
        case 4:
          moveLeft(255);
          break;
        case 5:
          moveForward(255);
          break;
        default:
          moveBackwards(255);
          break;
      }
      delay(random(2, 10) * 100);

    }
  }

  /* Update Light Colour*/
  if (backwardsCounter == 0) {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_YELLOW_PIN, 0);
  } else {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_YELLOW_PIN, 255);
  }
}

void takeDistanceMeasurements(int distanceList[] , int& minDistance, int & minIndex, double& avgDistance, int &frontDistance) {
  int forInitialization = 0;
  int iAngle = 0;
  int distance;                 // to store the distance calculated from the sensor

  avgDistance = 0;
  minDistance = MAX_DISTANCE;

  // Determinate scan direction (left to right or viceversa) for the measurements
  forIncrement = -forIncrement;
  if (forIncrement > 0) {
    forInitialization = 0;
  } else {
    forInitialization = nAngle - 1;
  }

  for (iAngle = forInitialization; (iAngle < nAngle && iAngle > -1); iAngle = iAngle + forIncrement) {

    // Read measurements and move servo
    distance = scan(angleList[iAngle] - 90 + FRONT_ANGLE);   // Move Servo and get distance measurement

    if (angleList[iAngle] == 90) {
      frontDistance = distance;
    }
    // Analyse Measurements
    distanceList[iAngle] = distance;
    avgDistance = avgDistance + distance / nAngle;
    if (distance < minDistance) {
      minDistance = distance;
      minIndex = iAngle;
    }
  }


}





void handleStall() {

  if (stallCounter > 10)
  {
    moveBackwards(255);
    delay(random(4, 10) * 100);
    if (random(1, 10) > 5) {
      moveBackwardsLeft(255);
      delay(random(3, 5) * 100);
      moveRight(255);
      delay(random(3, 10) * 100);
    } else{
      moveBackwardsRight(255);
      delay(random(3, 5) * 100);
      moveLeft(255);
      delay(random(3, 10) * 100);
    }
    stallCounter = 0;
    
  }

  if (missingMeasurements > 30)
  {
    moveBackwards(255);
    delay(random(2, 6) * 100);
    switch (random(1, 4)) {
      case 1:
        moveBackwardsLeft(255);
        break;
      case 2:
        moveBackwardsRight(255);
        break;
      case 3:
        moveRight(255);
        break;
      default:
        moveLeft(255);
        break;
    }
    delay(random(0, 4) * 100);
    missingMeasurements = 0;
  }

}

void updateSpeedEstimates(int frontDistance) {
  if ( isMoving ) {
    unsigned long currentFrontTime = millis();
    if ( wasMoving ) {

      double currentFrontSpeed = -1000 * (frontDistance - previousFrontDistance) / (currentFrontTime - previousFrontTime); // speed in cm/s

      if ( currentFrontSpeed > 15 ) {
        if (isMovingStraight && wasMovingStraight)
          speedEstimate = (0.99) * speedEstimate + 0.01 * currentFrontSpeed;
        stallCounter = 0;
      } else
        stallCounter++;
    }
    previousFrontDistance = frontDistance;
    previousFrontTime = currentFrontTime;
  }
  wasMovingStraight = isMovingStraight;
  wasMoving = isMoving;
}

void processInfraredInstructions() {
  if (irrecv.decode(&irResults)) {
    unsigned long irData = irResults.value;
    Serial.println(irData, HEX);
    irLastCheck = millis();
    if (currentMode == ir_remote) {
      switch (irData) {
        case 0xFF629D : // UP
          moveForward(255);
          break;
        case 0xFFA857 : // Down
          moveBackwards(255);
          break;
        case 0xFF22DD : //Left
          moveLeft(255);
          break;
        case 0xFFC23D : //Right
          moveRight(255);
          break;
        case 0xFF02FD : //OK button
          moveStop();
          break;
        case 0xFF6897 : // #1 button
          currentMode = self_driving;
          break;
      }
    } else if (irData == 0xFF9867) { //#2 button
      currentMode = ir_remote;
    }
    irrecv.resume();
  } else {
    if (currentMode == ir_remote && (millis() - irLastCheck) > 200 ) {
      // Turn Off both motors
      digitalWrite(IN_1_PIN, LOW);
      digitalWrite(IN_2_PIN, LOW);
      digitalWrite(IN_3_PIN, LOW);
      digitalWrite(IN_4_PIN, LOW);
    }

  }
}

/**********************************************
    Define Auxiliary functions
 * ********************************************
*/
int scan(int deg)
{
  int distance = 0;
  myServo.write(deg);
  delay(160);
  distance = sonar.ping_cm();
  if (distance <= 0) {
    distance = MAX_DISTANCE ;
  }
  if (distance == MAX_DISTANCE)
    missingMeasurements++;
  return distance;
}


double wallAngle(double b, double c, double alphaDeg )
{
  double pi = 3.14159265;
  double a = sqrt(b * b + c * c - 2 * b * c * cos(pi * alphaDeg / 180));
  double beta = acos((c * c + a * a - b * b) / (2 * a * c));
  return beta / pi * 180;
}

/* ********************************************
    Define Car Movements
 *  *******************************************
*/

void moveForward(int speed)
{
  isMovingStraight = true;
  isMoving = true;
  // turn on motor A
  digitalWrite(IN_1_PIN, HIGH);
  digitalWrite(IN_2_PIN, LOW);

  // turn on motor B
  digitalWrite(IN_3_PIN, HIGH);
  digitalWrite(IN_4_PIN, LOW);

  analogWrite(MOTOR_A_ENABLE_PIN, speed - max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed - max(0, -WHEELS_CORRECTION));
}

void moveBackwards(int speed)
{
  isMovingStraight = false;
  isMoving = false;
  // turn on motor A
  digitalWrite(IN_1_PIN, LOW);
  digitalWrite(IN_2_PIN, HIGH);

  // turn on motor B
  digitalWrite(IN_3_PIN, LOW);
  digitalWrite(IN_4_PIN, HIGH);

  analogWrite(MOTOR_A_ENABLE_PIN, speed - max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed - max(0, -WHEELS_CORRECTION));
}


void moveRight(int speed)
{
  isMovingStraight = false;
  isMoving = true;
  // turn off motor A
  digitalWrite(IN_1_PIN, LOW); // low
  digitalWrite(IN_2_PIN, HIGH); // low

  // turn on motor B
  digitalWrite(IN_3_PIN, HIGH);
  digitalWrite(IN_4_PIN, LOW);

  analogWrite(MOTOR_A_ENABLE_PIN, speed -  max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed -  max(0, -WHEELS_CORRECTION));
}
void moveForwardRight(int speed)
{
  isMovingStraight = false;
  isMoving = true;
  // turn off motor A
  digitalWrite(IN_1_PIN, LOW); // low
  digitalWrite(IN_2_PIN, LOW); // low

  // turn on motor B
  digitalWrite(IN_3_PIN, HIGH);
  digitalWrite(IN_4_PIN, LOW);

  analogWrite(MOTOR_A_ENABLE_PIN, speed -  max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed -  max(0, -WHEELS_CORRECTION));
}
void moveBackwardsRight(int speed)
{
  isMovingStraight = false;
  isMoving = false;
  // turn on motor A
  digitalWrite(IN_1_PIN, HIGH); // low
  digitalWrite(IN_2_PIN, LOW); //low

  // turn on motor B
  digitalWrite(IN_3_PIN, LOW);
  digitalWrite(IN_4_PIN, HIGH);

  analogWrite(MOTOR_A_ENABLE_PIN, floor(speed * 1 / 2) - max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed - max(0, -WHEELS_CORRECTION));
}

void moveLeft(int speed)
{
  isMovingStraight = false;
  isMoving = true;
  // turn on motor A
  digitalWrite(IN_1_PIN, HIGH);
  digitalWrite(IN_2_PIN, LOW);

  // turn off motor B
  digitalWrite(IN_3_PIN, LOW); // low
  digitalWrite(IN_4_PIN, HIGH); // low

  analogWrite(MOTOR_A_ENABLE_PIN, speed - max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed - max(0, -WHEELS_CORRECTION));
}

void moveForwardLeft(int speed)
{
  isMovingStraight = false;
  isMoving = true;
  // turn on motor A
  digitalWrite(IN_1_PIN, HIGH);
  digitalWrite(IN_2_PIN, LOW);

  // turn off motor B
  digitalWrite(IN_3_PIN, LOW); // low
  digitalWrite(IN_4_PIN, LOW); // low

  analogWrite(MOTOR_A_ENABLE_PIN, speed - max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, speed - max(0, -WHEELS_CORRECTION));
}

void moveBackwardsLeft(int speed)
{
  isMovingStraight = false;
  isMoving = false;
  // turn on motor A
  digitalWrite(IN_1_PIN, LOW);
  digitalWrite(IN_2_PIN, HIGH);

  // turn on motor B
  digitalWrite(IN_3_PIN, HIGH); // low
  digitalWrite(IN_4_PIN, LOW); // low

  analogWrite(MOTOR_A_ENABLE_PIN, speed - max(0, WHEELS_CORRECTION));
  analogWrite(MOTOR_B_ENABLE_PIN, floor(speed * 1 / 2) - max(0, -WHEELS_CORRECTION));
}

void moveStop()
{
  isMovingStraight = false;
  isMoving = false;

  digitalWrite(IN_1_PIN, LOW);
  digitalWrite(IN_2_PIN, LOW);
  digitalWrite(IN_3_PIN, LOW);
  digitalWrite(IN_4_PIN, LOW);

  delay(200);
}

