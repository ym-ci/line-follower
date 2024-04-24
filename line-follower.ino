#include "utils.h"
#include "DiffKinimatics.h"
#include "PIDController.h"

#include <cmath>
#include "MathUtil.h"
#include "utils.h"

const int MS_PER_TICK = 250;

const int SAMPLES_PER_READING = 5;
const int ARBRITRARY_NUMBER = 100;
const int PINS = 7;

int blackThreshold = 500;

const int SENSOR_PINS[7] = {A0, A1, A2, A3, A4, A5, A6};

const int lPin = 2;
const int rPin = 3;

// PIDController pid(0.2f, 0, 0);
// PIDController pid = PIDController(0.2f, 0, 0);

void setup()
{
  Serial.begin(9600);
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

    int cWeight = i - 3;
    if (black)
    {
      weight += cWeight;
      detections++;
    }
  }
  float lr = detections == 0 ? 0 : weight / detections;
  printVar("d", detections);
  printVar("w", weight);
  printVar("LR", lr);

  bool onStartEnd = detections == PINS;
  printVar("ons", onStartEnd);

  delay(MS_PER_TICK);

  float theta = pid.calculate(1.0f);
  printVar("theta", theta);
  LRPower lrPower = inverse(1.0f, theta);

  float lPower = lrPower.lPower;
  float rPower = lrPower.rPower;

  printVar("lPower", lPower);
  printVar("rPower", rPower);

  analogWrite(lPin, lPower * 0);
  analogWrite(rPin, rPower * 0);
  println("--------------------");
}