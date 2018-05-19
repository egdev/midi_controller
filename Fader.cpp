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
//   _analog  = new ResponsiveAnalogRead(_signalPin, false);

    pinMode(_enablePin, OUTPUT);
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    analogWrite(_enablePin, 255);
   _motorPin1Port = digitalPinToPort(_motorPin1);
   _motorPin1Bit  = digitalPinToBitMask(_motorPin1);
   _motorPin2Port = digitalPinToPort(_motorPin2);    
   _motorPin2Bit  = digitalPinToBitMask(_motorPin2);
}

uint16_t Fader::getMinPosition() {
  return _minPosition;
}

uint16_t Fader::getMaxPosition() {
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
  
  *portOutputRegister(_motorPin1Port) |= _motorPin1Bit;
  *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);
  delay(200);

  *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
  *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);

  _maxPosition = analogRead(_signalPin);
  
  *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
  *portOutputRegister(_motorPin2Port) |= _motorPin2Bit;
  delay(200);


  *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
  *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);

  _minPosition = analogRead(_signalPin);
}


// map fader position (0-1023) to cc midi value (0-127)
unsigned char Fader::faderPositionToMidiPosition(uint16_t faderPosition) {
  return (unsigned char) map(faderPosition, _minPosition, _maxPosition, 0, 127);  
}

// map midi value (0-127) to fader position (_minPosition, _maxPosition)
uint16_t Fader::midiPositionToFaderPosition(uint16_t midiPosition) {
  return (uint16_t) map(midiPosition, 0, 127, _minPosition, _maxPosition);
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
  _lastTouched = _touched;
  //long cs = _cs->capacitiveSensor(30);
  long cs = _cs->capacitiveSensor(5);
  if (!_touched && cs >= 200)    
    _touched = true;
  else if (_touched && cs < 50)
    _touched = false;

  //if (_lastTouched && !_touched) { // FALLING
  if (_touched) {
    _midiUpdate = true;
  }
}

bool Fader::needMidiUpdate() {
  return _midiUpdate;  
}

uint16_t Fader::updateCurrentPosition() {
  _currentPosition = analogRead(_signalPin);
  if (_currentPosition > _maxPosition) _currentPosition = _maxPosition;
  else if (_currentPosition < _minPosition) _currentPosition = _minPosition;
  return _currentPosition;
}

void Fader::setMidiUpdate(bool midiUpdate) {
  _midiUpdate = midiUpdate;
}

uint16_t Fader::getCurrentPosition() {
  return _currentPosition;
}

void Fader::setTargetPosition(uint16_t position) {
  _targetPosition = position;
}

void Fader::setMotorSpeed(uint16_t speed) {
  _motorSpeed = speed;
}

void Fader::move() {
  int16_t delta = _currentPosition - _targetPosition; // delta between read value and wanted value
  uint16_t abs_delta = abs(delta);
  if (!_touched && abs_delta > 15) {  // delta greater than deadband we can move and fader not touched
    if (!_moving) {                   // fader was not moving we set speed depending on delta
      if (abs_delta < 100)
        _motorSpeed = 200;
      else
        _motorSpeed = 255;
      _moving = true;
    } else {
      if (abs_delta < 100) // very close to target so reduce speed
        _motorSpeed--;
      else 
        _motorSpeed = map(abs_delta, 100, _maxPosition, 200, 255);
    }      
    analogWrite(_enablePin, _motorSpeed);
    if (delta > 0) {
      *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
      *portOutputRegister(_motorPin2Port) |= _motorPin2Bit;
    } else {
      *portOutputRegister(_motorPin1Port) |= _motorPin1Bit;
      *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);
    }
  } else {
    _motorSpeed = 255;
    *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
    *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);
    _targetPosition = _currentPosition;
    _moving = false;
  }  
}

