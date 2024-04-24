#include "utils.h"
#include "DiffKinimatics.h"
#include "PIDController.h"

#include <cmath>
#include "MathUtil.h"
#include "utils.h"

const int MS_PER_TICK = 1;

const int SAMPLES_PER_READING = 5;
const int ARBRITRARY_NUMBER = 100;
const int PINS = 7;

const int pwmSpeed = 255;

int blackThreshold = 500;

const int SENSOR_PINS[7] = {A0, A1, A2, A3, A4, A5, A6};

const int SENSOR_WEIGHTS[7] = {-12, -8, -1, 0, 1, 8, 12};

const int lPin = 2;
const int rPin = 3;

int previousDetection = 0;

PIDController pid(0.15, 0.1, 0.01, MS_PER_TICK / 1000.0f);

void setup()
{
  Serial.begin(57600);
  for (int i = 0; i < PINS; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  pinMode(lPin, OUTPUT);
  pinMode(rPin, OUTPUT);
}

void loop()
{
  // println("Calib1:1");
  int weight = 0;
  int detections = 0;
  for (int i = 0; i < PINS; i++)
  {
    // print(i);
    // print(":");
    int value = analogRead(i) - ARBRITRARY_NUMBER;
    bool black = value >= blackThreshold;
    // println(black);

    // int cWeight = i - 3;
    int cWeight = SENSOR_WEIGHTS[i];
    if (black)
    {
      weight += cWeight;
      detections++;
    }
  }
  float lr = detections == 0 ? previousDetection : weight / detections;
  if (detections != 0)
  {
    previousDetection = lr;
  }
  printVar("d", detections);
  printVar("w", weight);
  printVar("LR", lr);

  bool onStartEnd = detections == PINS;
  // printVar("ons", onStartEnd);

  float theta = pid.calculate(-lr);
  printVar("theta", theta);

  float throttle = 1.0f;

  float lPower = clamp(throttle - theta, -1.0f, 1.0f);
  float rPower = clamp(throttle + theta, -1.0f, 1.0f);

  printVar("lPower", lPower);
  printVar("rPower", rPower);

  analogWrite(lPin, lPower * pwmSpeed);
  analogWrite(rPin, rPower * pwmSpeed);
  println("--------------------");

  delay(MS_PER_TICK);
}