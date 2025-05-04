#ifndef LEDSYSTEM_H
#define LEDSYSTEM_H

#include <Arduino.h>

// LED animation types
enum LEDAnimation {
  LED_OFF,
  LED_ON,
  LED_BLINK,
  LED_PULSE,
  LED_ALTERNATE
};

class LEDSystem {
private:
  // Animation parameters
  LEDAnimation currentAnimation;
  unsigned long animationStartTime;
  unsigned long animationDuration;
  unsigned long blinkInterval;
  bool animationActive;
  bool currentState;
  
  // Utility functions
  void updateBlink();
  void updatePulse();
  void updateAlternate();
  float calculatePulseBrightness(unsigned long currentTime);

public:
  // Constructor
  LEDSystem();
  
  // Core functions
  void update(); // Call this in every loop iteration
  void stopAnimation();
  
  // Animation setup functions
  void setOff();
  void setOn();
  void setBlink(unsigned long interval, unsigned long duration = 0);
  void setPulse(unsigned long cycleTime, unsigned long duration = 0);
  void setAlternate(unsigned long interval, unsigned long duration = 0);
  
  // Utility functions
  bool isAnimationActive();
};

#endif // LEDSYSTEM_H