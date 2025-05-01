#include "utils.h"
#include "DiffKinimatics.h"
#include "PIDController.h"
#include "sensor.h"

#include <cmath>
#include <Servo.h>
#include "MathUtil.h"
#include "utils.h"


const int MS_PER_TICK = 1;

const int SAMPLES_PER_READING = 5;
const int ARBRITRARY_NUMBER = 50;
const int PINS = 6; // Changed from 7 to 6 to match the actual number of sensors used

const int pwmSpeed = 255;

const int SENSOR_PINS[PINS] = {A1, A2, A3, A4, A5, A6}; // Updated array size to match PINS

const int SENSOR_WEIGHTS[PINS] = {-12, -8, -1, 1, 8, 12}; // Updated array size to match PINS

int sensorThresholds[PINS] = {0, 0, 0, 0, 0, 0}; // Updated array size to match PINS

const int L_PIN = 2;
const int R_PIN = 3;

const int BTN_PIN = 4;

int previousDetection = 0;

int noDetectionsCycles = 0;

int loopCount = 0;

bool run = false;

PIDController pid(0.03, 0, 0.03, MS_PER_TICK / 1000.0f);

Servo lServo;
Servo rServo;

// Update global variables to use Sensor objects
Sensor sensors[PINS] = {Sensor(A1), Sensor(A2), Sensor(A3), Sensor(A4), Sensor(A5), Sensor(A6)};

void RobotForward(int lServoSpeed, int rServoSpeed){

  // println("RobotForward");

  printVar("lServoSpeed", lServoSpeed);
  printVar("rServoSpeed", rServoSpeed);

  // // Update LEFT Motor PWM Control Inputs to "Forward" state at the requested speed
  // lServo.write(90-lServoSpeed);
  // // Update RIGHT Motor PWM Control Inputs to "Forward" state at the requested speed
  // rServo.write(90-rServoSpeed);
}

void calibrate(){
  delay(2000);
  println("Calibrating");

  bool state = false;
  for (int i = 0; i < PINS; i++)
  {
    state = !state;
    digitalWrite(LED_BUILTIN, state);
    sensors[i].calibrate(SAMPLES_PER_READING, ARBRITRARY_NUMBER);
  }
  digitalWrite(LED_BUILTIN, LOW);
  println("Calibrated");
}

void setup()
{
  Serial.begin(9600);
  for (int i = 0; i < PINS; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  pinMode(L_PIN, OUTPUT);
  pinMode(R_PIN, OUTPUT);

  pinMode(BTN_PIN, INPUT_PULLDOWN);

  pinMode(LED_BUILTIN, OUTPUT);

  lServo.attach(L_PIN);
  rServo.attach(R_PIN);

  calibrate();
}

void loop()
{
  if (!run){
    //println("Waiting for button press");
    if (digitalRead(BTN_PIN) == HIGH){
      run = true;
    }
    return;
  }
  int weight = 0;
  int detections = 0;
  // Initialize a character array to hold the sensor status string
  char sensorStatus[PINS + 1]; // +1 for the null terminator

  for (int i = 0; i < PINS; i++)
  {
    int blackThreshold = sensors[i].getThreshold();
    // Determine if the sensor sees black based on the threshold
    bool black = sensors[i].isBlack();
    int cWeight = SENSOR_WEIGHTS[i];

    if (black)
    {
      // If black, add the weight, increment detections, and mark '#'
      weight += cWeight;
      detections++;
      sensorStatus[i] = '#';
    }
    else
    {
      // If white, mark '_'
      sensorStatus[i] = '_';
    }
  }
  // Null-terminate the character array to make it a valid C-string
  sensorStatus[PINS] = '\0';

  // Print the sensor status string (assuming printVar can handle char*)
  printVar("Sensors", sensorStatus);
  float throttle = 0.3; // detections == 0 ? 0 : 1.0f;
  float lr = detections == 0 ? previousDetection*10 : weight / detections;
  if (detections != 0)
  {
    noDetectionsCycles = 0;
    previousDetection = lr;
    //throttle ramp 0 slowly to avoid jerking
    throttle = 1.5;
  } else {
    noDetectionsCycles += 1;
    if (noDetectionsCycles > 100) {
      // lr = 15;
      throttle = 0; 
    }
  }
  printVar("d", detections);
  printVar("w", weight);
  printVar("LR", lr);

  bool onStartEnd = detections == PINS;
  if (onStartEnd || loopCount < 100) {
    throttle = 1;
    lr = 0;
    loopCount++;
  }
  printVar("ons", onStartEnd);

  float theta = pid.calculate(lr);
  printVar("theta", theta);


  float lPower = clamp(throttle - theta, -1.0f, 1.0f);
  float rPower = clamp(throttle + theta, -1.0f, 1.0f);

  RobotForward(lPower * 90, rPower * 90);
  
  println("--------------------");

  delay(MS_PER_TICK);
}