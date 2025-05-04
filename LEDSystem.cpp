#include "LEDSystem.h"

LEDSystem::LEDSystem() {
  // Initialize member variables
  animationActive = false;
  currentAnimation = LED_OFF;
  currentState = false;
  
  // Set up the built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Initialize LED to off
}

void LEDSystem::update() {
  // Check if animation should end based on duration
  if (animationActive && animationDuration > 0) {
    if (millis() - animationStartTime >= animationDuration) {
      stopAnimation();
      return;
    }
  }
  
  // Update the current animation
  switch (currentAnimation) {
    case LED_OFF:
      // LED is already off, nothing to do
      break;
      
    case LED_ON:
      // LED is already on, nothing to do
      break;
      
    case LED_BLINK:
      updateBlink();
      break;
      
    case LED_PULSE:
      updatePulse();
      break;
      
    case LED_ALTERNATE:
      updateAlternate();
      break;
  }
}

void LEDSystem::stopAnimation() {
  animationActive = false;
  digitalWrite(LED_BUILTIN, LOW);
  currentState = false;
  currentAnimation = LED_OFF;
}

void LEDSystem::setOff() {
  stopAnimation();
}

void LEDSystem::setOn() {
  currentAnimation = LED_ON;
  animationActive = true;
  animationStartTime = millis();
  animationDuration = 0; // Stay on indefinitely
  currentState = true;
  
  digitalWrite(LED_BUILTIN, HIGH);
}

void LEDSystem::setBlink(unsigned long interval, unsigned long duration) {
  currentAnimation = LED_BLINK;
  animationActive = true;
  animationStartTime = millis();
  animationDuration = duration;
  blinkInterval = interval;
  currentState = true;
  
  // Start with LED on
  digitalWrite(LED_BUILTIN, HIGH);
}

void LEDSystem::setPulse(unsigned long cycleTime, unsigned long duration) {
  // Since we can't do true PWM with just the built-in LED,
  // we'll simulate it with fast blinking
  currentAnimation = LED_PULSE;
  animationActive = true;
  animationStartTime = millis();
  animationDuration = duration;
  blinkInterval = cycleTime;
  currentState = false;
  
  digitalWrite(LED_BUILTIN, LOW);
}

void LEDSystem::setAlternate(unsigned long interval, unsigned long duration) {
  // With just one LED, alternate is the same as blink
  setBlink(interval, duration);
}

bool LEDSystem::isAnimationActive() {
  return animationActive;
}

// Private update functions for different animation types

void LEDSystem::updateBlink() {
  unsigned long elapsedTime = millis() - animationStartTime;
  bool newState = (elapsedTime / blinkInterval) % 2 == 0;
  
  // Only update if state has changed
  if (newState != currentState) {
    currentState = newState;
    digitalWrite(LED_BUILTIN, currentState ? HIGH : LOW);
  }
}

void LEDSystem::updatePulse() {
  // For the built-in LED without PWM, we'll use a pseudo-pulse effect
  // by blinking at increasing/decreasing rates
  unsigned long currentTime = millis();
  float brightness = calculatePulseBrightness(currentTime);
  
  // Use brightness to determine blink speed - higher brightness = faster blinks
  unsigned long effectiveInterval = map(round(brightness * 100), 0, 100, 500, 50);
  unsigned long elapsedTime = currentTime - animationStartTime;
  
  bool newState = (elapsedTime % effectiveInterval) < (effectiveInterval / 2);
  
  // Only update if state has changed
  if (newState != currentState) {
    currentState = newState;
    digitalWrite(LED_BUILTIN, currentState ? HIGH : LOW);
  }
}

void LEDSystem::updateAlternate() {
  // With one LED, this is the same as blink
  updateBlink();
}

float LEDSystem::calculatePulseBrightness(unsigned long currentTime) {
  // Calculate phase of the sine wave (0 to 2*PI)
  float phase = ((currentTime - animationStartTime) % blinkInterval) / (float)blinkInterval;
  phase *= 2 * PI;
  
  // Apply sine wave function and scale from -1..1 to 0..1
  return (sin(phase) + 1.0f) / 2.0f;
}