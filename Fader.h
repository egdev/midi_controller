
#pragma once

#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <L298N.h>
#include <ResponsiveAnalogRead.h>
#include "Header.h"

typedef enum {
  MF_Standby = 0,
  MF_Up      = 1,
  MF_Down    = 2
} direction;

class Fader {
  public:
    Fader(uint16_t signalPin, uint8_t enablePin, uint8_t motorPin1, uint8_t motorPin2, uint8_t touchPin, uint8_t sendTouchPin, bool master = 0);
    byte faderPositionToMidiPosition(int faderPosition);
    int midiPositionToFaderPosition(byte midiPosition);
    uint16_t getSignalPin();
    uint8_t getEnablePin();
    uint8_t getMotorPin1();
    uint8_t getMotorPin2();
    bool isMoving();
    bool isTouched();
    bool isMaster();
    void checkTouched();
    void calibrate();
    int readCurrentPosition();
    int getCurrentPosition();
    int getTargetPosition();
    void setTargetPosition(int position);
    void setLastPosition(int position);
    int getLastPosition();
    void move();
    void setMotorSpeed(uint16_t speed);
    int getMinPosition();
    int getMaxPosition();
    uint16_t getMotorSpeed();
    CapacitiveSensor* _cs; 
    bool _suspended = false;
    volatile uint8_t *_motorPin1Port;
    volatile uint8_t *_motorPin2Port;
    
  private:
    volatile int _lastPosition     = 0;
    volatile int _currentPosition  = 0;
    volatile int _targetPosition   = 0;
    volatile int _minPosition      = 0;
    volatile int _maxPosition      = 1023;
    uint8_t _deadBand     = 15;
    uint8_t _signalPin;           // pin for wiper signal
    uint8_t _enablePin;           // pwm pin for motor
    uint8_t _motorPin1;           // pin IN1 for motor
    uint8_t _motorPin2;           // pin IN2 for motor
    uint8_t _receiveTouchPin;     // touch pin from fader            
    uint8_t _sendTouchPin;        // pin from 1M resistor
    uint16_t _motorSpeed = 255;   // PWM motor speed 
    bool _touched;
    bool _moving;                 // true if fader is moving
    bool _master;                 // true if master fader
    uint8_t _motorPin1Bit;
    uint8_t _motorPin2Bit;
};

