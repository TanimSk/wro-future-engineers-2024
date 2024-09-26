#include <Servo.h>

#define ROTATION_THRESHOLD 3
#define MAX_SONARS 5

#define RPI_PWM_MAX 168
#define RPI_PWM_MIN 0
#define RPI_COMM_PIN A4

#define RPI_CAMERA_FLASH 11

int previousSteerAngle = 90;
int degreeToSteer = 0;

// Constants
const int minimumDistance = 20;
const int minimumFrontDistance = 30;

const int maxSteerDegreeRight = 22;
const int maxSteerDegreeLeft = 26;
// const int hardCornerDegree = 30;

// Servo
const int servoPin = 10;
Servo servo;

// Motor driver
const int enable = 9;
const int motorIn1 = 8;
const int motorIn2 = 7;

// Sonar pins
const int sonarPins[MAX_SONARS][2] = {
    {A1, 2},  // left
    {4, 3},   // front-left
    {A2, A5}, // front-center
    {13, 12}, // front-right
    {A3, 6}   // right
};

enum Sonar
{
  LEFT,
  FRONT_LEFT,
  FRONT_CENTER,
  FRONT_RIGHT,
  RIGHT
};

// Function to get sonar distance in cm
float getDistanceCM(Sonar sonar)
{
  int triggerPin = sonarPins[sonar][0];
  int echoPin = sonarPins[sonar][1];

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

// Function to steer the vehicle
void steering(String direction, float currentDistance)
{
  int changeInRotation = 0;

  if (direction == "LEFT")
  {
    changeInRotation = map(currentDistance, 0, minimumDistance, maxSteerDegreeLeft, 0);
    degreeToSteer = 90 + changeInRotation;
    Serial.println("Steering left" + String(degreeToSteer));
  }
  else if (direction == "RIGHT")
  {
    changeInRotation = map(currentDistance, 0, minimumDistance, maxSteerDegreeRight, 0);
    degreeToSteer = 90 - changeInRotation;
    Serial.println("Steering right" + String(degreeToSteer));
  }
  else
  {
    degreeToSteer = 90;
  }

  if (abs(degreeToSteer - previousSteerAngle) > ROTATION_THRESHOLD)
  {
    Serial.println("Steering to " + String(degreeToSteer));
    servo.write(degreeToSteer);
    previousSteerAngle = degreeToSteer;
  }
}

// Motor control
void goForward(int speed)
{
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  analogWrite(enable, speed);
}

void goBackward(int speed)
{
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  analogWrite(enable, speed);
}

void stopMotor()
{
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  analogWrite(enable, 0);
}

byte GetPWM(byte pin)
{
  //  static byte lastPWM = 0;  // Store the previous PWM value
  //  const float alpha = 0.05;  // Smoothing factor, between 0 and 1 (smaller values smooth more)

  unsigned long highTime = pulseIn(pin, HIGH, 50000UL); // 50 ms timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);   // 50 ms timeout

  // pulseIn() returns zero on timeout
  byte pwm;
  if (highTime == 0 || lowTime == 0)
  {
    pwm = digitalRead(pin) ? 100 : 0; // HIGH == 100%, LOW == 0%
  }
  else
  {
    pwm = (100 * highTime) / (highTime + lowTime); // highTime as percentage of total cycle time
  }

  // Apply exponential moving average to filter noise
  //  pwm = (alpha * pwm) + ((1 - alpha) * lastPWM);
  //  lastPWM = pwm;

  return pwm;
}

// Function that converts angle from the pwm signal sent by RPI

String GetCameraDecision()
{
  const int value = GetPWM(RPI_COMM_PIN_A);
  const int motionVal = GetPWM(A0);
  Serial.println("PWM: " + String(value));
  if (value == 0)
  {
    return "NORMAL";
  }
  else if (60 <= value && value <= 75 && motionVal < 95)
  {
    return "RIGHT";
  }
  else if (10 <= value && value <= 25 && motionVal < 95)
  {
    return "LEFT";
  }
  else if (motionVal > 95)
  {
    return "STOP";
  }
  return "NORMAL";
}

void setup()
{
  Serial.begin(9600);
  // Servo configuration
  servo.attach(servoPin);
  servo.write(90);
  delay(1000);

  // Sonar pins configuration
  for (int i = 0; i < MAX_SONARS; ++i)
  {
    pinMode(sonarPins[i][1], INPUT);
    pinMode(sonarPins[i][0], OUTPUT);
  }

  // Motor driver configuration
  pinMode(enable, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  // Wait for camera to be detected
  Serial.print("Waiting for Camera..");
//  while (true)
//  {
//    if (GetPWM(A0) > 95)
//    {
//      Serial.println("\nCamera detected");
//      break;
//    }
//    else
//    {
//      Serial.print(".");
//    }
//    delay(1000);
//  }
  analogWrite(RPI_CAMERA_FLASH, 180);

  delay(2500);
  goForward(255);
}

void loop()
{
  goForward(255);
//  delay(100);

  // Read sonar distances
  int frontCenterDistance = min(getDistanceCM(FRONT_CENTER), min(getDistanceCM(FRONT_RIGHT), getDistanceCM(FRONT_LEFT)));
  //   int frontLeftDistance = getDistanceCM(FRONT_LEFT);
  //   int frontRightDistance = getDistanceCM(FRONT_RIGHT);
  int leftDistance = getDistanceCM(LEFT);
  int rightDistance = getDistanceCM(RIGHT);
  String CamDecision = GetCameraDecision();
  Serial.println(CamDecision);

  String distances = String(leftDistance) + ", " + String(frontCenterDistance) + ", " + String(rightDistance) + "\n";
  Serial.print(distances);

  // Check for obstacles and steer accordingly
  if (CamDecision == "LEFT" || CamDecision == "RIGHT")
  {
    steering(CamDecision, 0);
  }
  else if (CamDecision == "STOP")
  {
    Serial.println("Stop Signal Detected");
    stopMotor();
    delay(1500);
    while (true)
    {
      if (GetPWM(A0) > 95)
      {
        Serial.println("Start Signal Detected...");
        delay(2000);
        break;
      }
      delay(50);
    }
  }
  else
  {
    if (leftDistance > rightDistance)
    {
      // Left side has more place, thats why moving towards it
      steering("LEFT", 0);
    }
    else if (rightDistance > leftDistance)
    {
      // Right side has more place, thats why moving towards it
      steering("RIGHT", 0); // Steer hard right if left side is blocked
    }
    else
    {
      goForward(255);
      servo.write(90);
      previousSteerAngle = 90;
    }
  }
}
