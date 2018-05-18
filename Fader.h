
#pragma once

#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <L298N.h>
#include <ResponsiveAnalogRead.h>
#include "Header.h"


class Fader {
  public:
    Fader(uint16_t signalPin, uint8_t enablePin, uint8_t motorPin1, uint8_t motorPin2, uint8_t touchPin, uint8_t sendTouchPin, bool master = 0);
    unsigned char faderPositionToMidiPosition(uint16_t faderPosition);
    uint16_t midiPositionToFaderPosition(uint16_t midiPosition);
    uint16_t getSignalPin();
    uint8_t getEnablePin();
    uint8_t getMotorPin1();
    uint8_t getMotorPin2();
    bool isMoving();
    bool isTouched();
    bool isMaster();
    void checkTouched();
    bool needMidiUpdate();
    uint16_t updateCurrentPosition();
    void setMidiUpdate(bool midiUpdate);
    void calibrate();
    L298N* getDCMotor();
    ResponsiveAnalogRead* getAnalog();
    uint16_t getCurrentPosition();
    void move();
    void setTargetPosition(uint16_t position);
    void setMotorSpeed(uint16_t speed);
    uint16_t getMinPosition();
    uint16_t getMaxPosition();
    CapacitiveSensor* _cs; 
    
  private:
    uint16_t _currentPosition;
    uint16_t _targetPosition;
    uint16_t _minPosition = 0;
    uint16_t _maxPosition = 1023;
    uint8_t _deadBand = 15;
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
    bool _lastTouched;
    bool _midiUpdate;
    L298N* _DCMotor;
    ResponsiveAnalogRead* _analog;
};

