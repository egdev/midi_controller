#pragma once

#include "Fader.h"

Fader::Fader(uint16_t signalPin, uint8_t enablePin, uint8_t motorPin1, uint8_t motorPin2, uint8_t receiveTouchPin, uint8_t sendTouchPin, bool master) {//, uint16_t enablePinPort, uint8_t enablePinBit, uint16_t motorPin1Port, uint8_t motorPin1Bit, uint16_t motorPin2Port, uint8_t motorPin2Bit) {
   _signalPin = signalPin;
   _master = master;
   _currentPosition = 0;
   _enablePin = enablePin;
   _motorPin1 = motorPin1;
   _motorPin2 = motorPin2;
   _receiveTouchPin = receiveTouchPin;
   _sendTouchPin = sendTouchPin;
   _cs      = new CapacitiveSensor(_sendTouchPin, _receiveTouchPin); 
   _cs->set_CS_AutocaL_Millis(0xFFFFFFFF);

    pinMode(_enablePin, OUTPUT);
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    analogWrite(_enablePin, 255);
   _motorPin1Bit  = digitalPinToBitMask(_motorPin1);
   _motorPin1Port = portOutputRegister(digitalPinToPort(_motorPin1));
   _motorPin2Bit  = digitalPinToBitMask(_motorPin2);
   _motorPin2Port = portOutputRegister(digitalPinToPort(_motorPin2));
}

int Fader::getLastPosition() {
  return _lastPosition;
}

void Fader::setLastPosition(int position) {
  _lastPosition = position;
}

int Fader::getMinPosition() {
  return _minPosition;
}

int Fader::getMaxPosition() {
  return _maxPosition;
}

uint8_t Fader::getEnablePin() {
  return _enablePin;
}

uint8_t Fader::getMotorPin1() {
  return _motorPin1;
}

uint8_t Fader::getMotorPin2() {
  return _motorPin2;
}

void Fader::calibrate() {
  analogWrite(_enablePin, 255);
  
  *_motorPin1Port |= _motorPin1Bit;
  *_motorPin2Port &= ~(_motorPin2Bit);
  delay(200);

  *_motorPin1Port &= ~(_motorPin1Bit);
  *_motorPin2Port &= ~(_motorPin2Bit);

  _maxPosition = analogRead(_signalPin);
  
  *_motorPin1Port &= ~(_motorPin1Bit);
  *_motorPin2Port |= _motorPin2Bit;
  delay(200);


  *_motorPin1Port &= ~(_motorPin1Bit);
  *_motorPin2Port &= ~(_motorPin2Bit);

  _minPosition = analogRead(_signalPin);
}

// map fader position (0-1023) to cc midi value (0-127)
byte Fader::faderPositionToMidiPosition(int faderPosition) {
  return (byte) map(faderPosition, _minPosition, _maxPosition, 0, 127);  
}

// map midi value (0-127) to fader position (_minPosition, _maxPosition)
int Fader::midiPositionToFaderPosition(byte midiPosition) {
  return (int) map(midiPosition, 0, 127, _minPosition, _maxPosition);
}

bool Fader::isMoving() {
  return _moving;
}

bool Fader::isTouched() {
  return _touched;
}

bool Fader::isMaster() {
  return _master;
}

uint16_t Fader::getSignalPin() {
  return _signalPin;
}

void Fader::checkTouched() {
  static long waitBeforeMove = 0;
  long cs = _cs->capacitiveSensor(5);
  
  if (!_touched && cs >= 150) { 
    _touched = true;
    _suspended = true;
  } else if (_touched && cs < 50) {
    _touched = false;
    waitBeforeMove = millis();
  }

  if (_suspended && !_touched && (millis() - waitBeforeMove) > 500) { // not touched for 500ms
    _suspended = false;
  }
}

int Fader::readCurrentPosition() {
  _currentPosition = analogRead(_signalPin);
  if (_currentPosition > _maxPosition) _currentPosition = _maxPosition;
  else if (_currentPosition < _minPosition) _currentPosition = _minPosition;
  return _currentPosition;
}

int Fader::getCurrentPosition() {
  return _currentPosition;
}

int Fader::getTargetPosition() {
  return _targetPosition;
}

void Fader::setTargetPosition(int position) {
  _targetPosition = position;
}

uint16_t Fader::getMotorSpeed() {
  return _motorSpeed;
}

void Fader::setMotorSpeed(uint16_t speed) {
  _motorSpeed = speed;
}

void Fader::move() {
  int delta = _currentPosition - _targetPosition; // delta between read value and wanted value
  int abs_delta = abs(delta);
  if (!_suspended && abs_delta > MF_DEADBAND) {  // delta greater than deadband we can move and fader not touched
    if (!_moving) {                   // fader was not moving, start at max speed
      if (abs_delta < 50)
        _motorSpeed = 200;
      else
        _motorSpeed = 255;
      _moving = true;
    } else {
      if (abs_delta < 100) // moving but very close to target so reduce speed via PWM
        _motorSpeed = map(abs_delta, 0, 100, 100, 200); 
      else if (abs_delta < 150)
        _motorSpeed = 200;
      else                 // go fast
        _motorSpeed = 255;
    }

    analogWrite(_enablePin, _motorSpeed);
    if (delta > 0) { // move down
      *_motorPin1Port &= ~(_motorPin1Bit);
      *_motorPin2Port |= _motorPin2Bit;
    } else {        // move up
      *_motorPin1Port |= _motorPin1Bit;
      *_motorPin2Port &= ~(_motorPin2Bit);
    }
  } else { // stop
    _motorSpeed = 255;
    *_motorPin1Port &= ~(_motorPin1Bit);
    *_motorPin2Port &= ~(_motorPin2Bit);
    _targetPosition = _currentPosition;
    _moving = false;
  }

}


