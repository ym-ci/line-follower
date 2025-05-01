#include <AdafruitIO.h>

#include "utils.h"
#include "DiffKinimatics.h"
#include "PIDController.h"

#include <cmath>
#include <Servo.h>
#include "MathUtil.h"
#include "utils.h"


const int MS_PER_TICK = 1;

const int SAMPLES_PER_READING = 5;
const int ARBRITRARY_NUMBER = 50;
const int PINS = 7;

const int pwmSpeed = 255;

const int SENSOR_PINS[7] = {A0, A1, A2, A3, A4, A5, A6};

const int SENSOR_WEIGHTS[7] = {-12, -8, -1, 0, 1, 8, 12};

int sensorThresholds[7] = {0, 0, 0, 0, 0, 0, 0};

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

void RobotForward(int lServoSpeed, int rServoSpeed){

  println("RobotForward");

  printVar("lServoSpeed", lServoSpeed);
  printVar("rServoSpeed", rServoSpeed);

  // Update LEFT Motor PWM Control Inputs to "Forward" state at the requested speed
  lServo.write(90-lServoSpeed);
  // Update RIGHT Motor PWM Control Inputs to "Forward" state at the requested speed
  rServo.write(90-rServoSpeed);
}

void calibrate(){
  delay(2000);
  println("Calibrating");

  bool state = false;
  for (int i = 0; i < PINS; i++)
  {
    int value = 0;
    for (int j = 0; j < SAMPLES_PER_READING; j++)
    {
      state = !state;
      digitalWrite(LED_BUILTIN, state);
      value += analogRead(i);
    }
    sensorThresholds[i] = (value / SAMPLES_PER_READING) - ARBRITRARY_NUMBER;
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
  for (int i = 0; i < PINS; i++)
  {
    int blackThreshold = sensorThresholds[i];
    int value = analogRead(i);
    bool black = value >= blackThreshold;
    int cWeight = SENSOR_WEIGHTS[i];
    if (black)
    {
      weight += cWeight;
      detections++;
    }
  }
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