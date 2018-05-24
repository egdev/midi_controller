
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
    uint16_t readCurrentPosition();
    void setMidiUpdate(bool midiUpdate);
    void calibrate();
    uint16_t getCurrentPosition();
    uint16_t getTargetPosition();
    void move();
    void setTargetPosition(uint16_t position);
    void setMotorSpeed(uint16_t speed);
    uint16_t getMinPosition();
    uint16_t getMaxPosition();
    uint16_t getMotorSpeed();
    void setDelta(uint16_t delta);
    uint16_t getDelta();
    void setPosition(uint16_t position);
    uint16_t getPosition();
    CapacitiveSensor* _cs; 
    direction _direction = MF_Standby;    
    bool _idle;
    uint8_t _pwm_ctr;
    uint8_t _manual_move_ctr;
    uint8_t _timeout_ctr;
    uint8_t _repeat_ctr;
    bool _suspended = false;
    
  private:
    uint16_t _delta;
    uint16_t _position = 0;
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
    bool _midiUpdate;
    uint8_t _motorPin1Port;
    uint8_t _motorPin1Bit;
    uint8_t _motorPin2Port;
    uint8_t _motorPin2Bit;
};

