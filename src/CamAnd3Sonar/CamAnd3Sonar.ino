#include <Servo.h>

// Minimum Change in rotation to be applied on the steering servo (in degrees)
#define ROTATION_THRESHOLD 3
// Number of Sonar sensors connected to the vehicle
#define MAX_SONARS 3

/*
  Raspberry Pi handles only the image processing so
  it needs some way to communicate back and forth to
  the arduino.
  Thats why we chose to use 2 Digital Pins to use it
  as a way of communication
*/
#define RPI_COMM_PIN_A A4
#define RPI_COMM_PIN_B A0

/* 
  LED Flash for the camera
  We have chose a pwm pin so that we can control the light's
  intensity
*/
#define RPI_CAMERA_FLASH 11

/*
  Variables for holding the steering data
*/
int previousSteerAngle = 90;
int degreeToSteer = 0;

/*
  Safety distances for the sonar sensors
*/
const int minimumDistance = 20;
const int minimumFrontDistance = 30;

/*
  Variables for our ackerman steering system's
  configuration
*/
const int maxSteerDegreeRight = 22;
const int maxSteerDegreeLeft = 26;

/*
  Servo for the steering control
*/
const int servoPin = 10;
Servo servo;

/*
  Motor driver pins for controlling the rear wheel
*/
const int enable = 9;
const int motorIn1 = 8;
const int motorIn2 = 7;

// Sonar pins
const int sonarPins[MAX_SONARS][2] = {
    {A1, 2},  // left
    {A2, A5}, // front-center
    {A3, 6}   // right
};

// Enum for the sonar pins
// Easy to implement and understand
// Efficient to use
enum Sonar
{
  LEFT,
  FRONT_CENTER,
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
  // Calculating the change in rotation based on the distances 
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

// Simple void functions to control the motors

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

// Function that returns the PWM value of the pin that the camera is connected to
// Part of the communication between the Arduino and the Raspberry Pi
byte GetPWM(byte pin) {
//  static byte lastPWM = 0;  // Store the previous PWM value
//  const float alpha = 0.05;  // Smoothing factor, between 0 and 1 (smaller values smooth more)

  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 ms timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 ms timeout

  // pulseIn() returns zero on timeout
  byte pwm;
  if (highTime == 0 || lowTime == 0) {
    pwm = digitalRead(pin) ? 100 : 0;  // HIGH == 100%, LOW == 0%
  } else {
    pwm = (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
  }

  // Apply exponential moving average to filter noise
//  pwm = (alpha * pwm) + ((1 - alpha) * lastPWM);
//  lastPWM = pwm;

  return pwm;
}


// Decoding the PWM signal sent by the Raspberry Pi
String GetCameraDecision(){
  const int value = GetPWM(RPI_COMM_PIN_A);
  const int motionVal = GetPWM(RPI_COMM_PIN_B);
  Serial.println("PWM: "+String(value));
  if(value == 0){
    return "NORMAL";
  }else if(60<=value && value<=75 && motionVal<95){
    return "RIGHT";
  }else if(10<=value && value<=25 && motionVal<95){
    return "LEFT";
  }else if(motionVal>95){
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
  while (true)
  {
    if(GetPWM(RPI_COMM_PIN_B) > 95)
    {
      Serial.println("\nCamera detected");
      break;
    }else{
      Serial.print(".");
    }
    delay(1000);
  }
  analogWrite(RPI_CAMERA_FLASH, 180);
  
  delay(2500);
  goForward(255);
  
}

void loop()
{
  goForward(255);
  delay(100);

  // Read sonar distances
  int frontCenterDistance = getDistanceCM(FRONT_CENTER);
  int leftDistance = getDistanceCM(LEFT);
  int rightDistance = getDistanceCM(RIGHT);
  String CamDecision = GetCameraDecision();
  Serial.println(CamDecision);

   String distances = String(leftDistance) + ", " + String(frontCenterDistance) + ", " + String(rightDistance) + "\n";
   Serial.print(distances);

  // Check for obstacles and steer accordingly
  if(CamDecision == "LEFT" || CamDecision == "RIGHT"){
    steering(CamDecision, 0);
  }
  else if(CamDecision == "STOP"){
    Serial.println("Stop Signal Detected");
    stopMotor();
    delay(1500);
    while(true){
      if(GetPWM(RPI_COMM_PIN_B)>95){
        Serial.println("Start Signal Detected...");
        delay(2000);
        break;
      }
      delay(50);
    }
  }
  else if (frontCenterDistance < minimumFrontDistance)
  {
    if ((leftDistance < minimumDistance) && (rightDistance < minimumDistance))
    {
      Serial.println("Going back");
      goBackward(255);
      delay(2000);
    }
    else if (leftDistance < 30)
    {
      // Left e obstacle
      steering("RIGHT", 0); // Steer hard right if left side is blocked
      delay(1800);
    }
    else if (rightDistance < 30)
    {
      // Right e obstacle
      steering("LEFT", 0); // Steer hard left otherwise
      delay(1800);
      // reset angle
      previousSteerAngle = 90;
    }
    else
    {
      steering("LEFT", 0);
      delay(1800);
      // reset angle
      previousSteerAngle = 90;
    }
  }
  else
  {
    if (leftDistance < minimumDistance) 
    {
      steering("RIGHT", leftDistance);
    }
    // else if (frontLeftDistance < minimumDistance)
    // {
      // steering("RIGHT", frontLeftDistance);
    // }
    else if (rightDistance < minimumDistance)
    {
      steering("LEFT", rightDistance);
    }
    // else if (frontRightDistance < minimumDistance)
    // {
      // steering("LEFT", frontRightDistance);
    // }
    else
    {
      servo.write(90);
      // reset the angle
      previousSteerAngle = 90;
    }
  }
}
