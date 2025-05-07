#include "utils.h"
#include "DiffKinimatics.h"
#include "PIDController.h"
#include "sensor.h"
#include "LEDSystem.h"

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

const int SENSOR_WEIGHTS[PINS] = {-40, -20, -10, 10, 20, 40}; // Updated array size to match PINS

int sensorThresholds[PINS] = {0, 0, 0, 0, 0, 0}; // Updated array size to match PINS

const int L_PIN = 2;
const int R_PIN = 3;

const int BTN_PIN = 4;

int previousDetection = 0;

int noDetectionsCycles = 0;

int loopCount = 0;

bool run = false;

PIDController pid(0.0075, 0, 0.01, MS_PER_TICK / 1000.0f);

Servo lServo;
Servo rServo;

// Create LED system for the onboard LED
LEDSystem ledSystem;

// Update global variables to use Sensor objects
Sensor sensors[PINS] = {Sensor(A1), Sensor(A2), Sensor(A3), Sensor(A4), Sensor(A5), Sensor(A6)};

void RobotForward(int lServoSpeed, int rServoSpeed){

  // println("RobotForward");

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

  // Set LED animation to pulse while waiting for calibration
  ledSystem.setPulse(1000);

  while (digitalRead(BTN_PIN) == LOW){
    println("Waiting for button press");
    ledSystem.update(); // Update LED animation
    delay(100);
  }

  // Switch LED animation to blink during calibration
  ledSystem.setBlink(200);
  
  for (int i = 0; i < PINS; i++)
  {
    // Use array elements directly instead of local copies
    sensors[i].calibrateWhite(SAMPLES_PER_READING);
    // Use separate Serial.print statements instead of string concatenation
    Serial.print("Calibrated white sensor ");
    Serial.print(i);
    Serial.print(" to ");
    Serial.println(sensors[i].getWhiteValue());
    ledSystem.update(); // Update LED animation
    delay(100);
  }

  // wait for button to be released
  while (digitalRead(BTN_PIN) == HIGH){
    println("Waiting for button release");
    ledSystem.update(); // Update LED animation
    delay(100);
  }

  // Set LED animation to alternate while waiting for button press
  ledSystem.setAlternate(300);

  // wait for button to be pressed again
  while (digitalRead(BTN_PIN) == LOW){
    println("Waiting for button press");
    ledSystem.update(); // Update LED animation
    delay(100);
  }
  
  // Switch LED animation back to blink during second calibration
  ledSystem.setBlink(200);
  
  for (int i = 0; i < PINS; i++)
  {
    // Use array elements directly instead of local copies
    sensors[i].calibrateBlack(SAMPLES_PER_READING);
    // Use separate Serial.print statements instead of string concatenation
    Serial.print("Calibrated black sensor ");
    Serial.print(i);
    Serial.print(" to ");
    Serial.println(sensors[i].getBlackValue());
    ledSystem.update(); // Update LED animation
    delay(100);
  }
  
  // wait for button to be released
  while (digitalRead(BTN_PIN) == HIGH){
    println("Waiting for button release");
    ledSystem.update(); // Update LED animation
    delay(100);
  }
  
  // Set LED animation to pulse when calibration is complete and waiting to start
  ledSystem.setPulse(2000);
}

void loop()
{
  // Update LED animations
  ledSystem.update();
  
  if (!run){
    //println("Waiting for button press");
    if (digitalRead(BTN_PIN) == HIGH){
      run = true;
      // Set LED animation to blink slowly when robot starts running
      ledSystem.setBlink(500);
    }
    delay(MS_PER_TICK);
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
    if (noDetectionsCycles > 50) {
      // lr = 15;
      throttle = 0; 
      if (abs(previousDetection) < 50) {
        lr = (previousDetection >= 0) ? 400 : -400;
      }
    }
  }
  printVar("d", detections);
  printVar("w", weight);
  printVar("LR", lr);

  bool onStartEnd = detections == PINS;
  if (onStartEnd || loopCount < 100) {
    throttle = 1.0;
    lr = 0;
    loopCount++;
    
    // When on start/end line, use LED on
    if (onStartEnd) {
      ledSystem.setOn();
    }
  }
  
  // Update LED based on the robot's state
  if (detections == 0) {
    // No line detected - fast blink to indicate warning
    if (noDetectionsCycles > 50 && noDetectionsCycles <= 100) {
      ledSystem.setBlink(100);
    } else if (noDetectionsCycles > 100) {
      // Robot stopped - rapid blink to indicate error
      ledSystem.setBlink(50);
    }
  } else if (!onStartEnd && !ledSystem.isAnimationActive()) {
    // Normal line following - slow blink
    ledSystem.setBlink(500);
  }
  
  printVar("ons", onStartEnd);

  float theta = pid.calculate(lr);
  printVar("theta", theta);


  float lPower = clamp(throttle - theta, -1.0f, 1.0f);
  float rPower = clamp(throttle + theta, -1.0f, 1.0f);

  RobotForward(lPower * 60, rPower * 60);
  
  println("--------------------");

  delay(MS_PER_TICK);
}