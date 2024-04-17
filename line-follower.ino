#include <kalmanFilter.h>
#include <ReefwingAHRS.h>

#include "utils.h"

const int MS_PER_TICK = 250;

const int SAMPLES_PER_READING = 5;
const int ARBRITRARY_NUMBER = 100;
const int PINS = 7;

int blackThreshold = 500;

const int SENSOR_PINS[7] = {A0,A1,A2,A3,A4,A5,A6}; 
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < PINS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

void loop() {
  // println("Calib1:1");
  int weight = 0;
  int detections = 0;
  for (int i = 0; i < PINS; i++) {
    // print(i);
    // print(":");
    int value = analogRead(i) - ARBRITRARY_NUMBER;
    bool black = value >= blackThreshold;
    // println(black);

    int cWeight = i-3;
    if (black) {
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
}