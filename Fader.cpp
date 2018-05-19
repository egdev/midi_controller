#pragma once

#include "Fader.h"

Fader::Fader(uint16_t signalPin, uint8_t enablePin, uint8_t motorPin1, uint8_t motorPin2, uint8_t receiveTouchPin, uint8_t sendTouchPin, bool master) {
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
   _DCMotor = new L298N(_enablePin, _motorPin1, _motorPin2);
   _analog  = new ResponsiveAnalogRead(_signalPin, false);
}

L298N* Fader::getDCMotor() {
  return _DCMotor;  
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
  //analogWrite(getEnablePin, 255);
  
  _DCMotor->setSpeed(255);
  _DCMotor->forward();
  delay(250);
  _DCMotor->stop();
  _analog->update();
  _maxPosition = _analog->getValue();
  //delay(100);
  _DCMotor->backward();
  delay(250);
  _DCMotor->stop();
  _analog->update();
  _minPosition = _analog->getValue();
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

ResponsiveAnalogRead* Fader::getAnalog() {
  return _analog;  
}

uint16_t Fader::updateCurrentPosition() {
  //_analog->update();
  //_currentPosition = constrain(_analog->getValue(), _minPosition, _maxPosition);
  //_currentPosition = constrain(analogRead(getSignalPin()), _minPosition, _maxPosition);
  _currentPosition = analogRead(getSignalPin());
  /*_currentPosition = _analog->getValue();*/
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
  //if (!_touched && abs_delta > 30) {  // delta greater than deadband we can move and fader not touched
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
    _DCMotor->setSpeed(_motorSpeed);
    if (delta > 0) {
      _DCMotor->backward();
    } else {
      _DCMotor->forward();
    }
  } else {
    _motorSpeed = 255;
    _DCMotor->stop();
    _targetPosition = _currentPosition;
    _moving = false;
  }  
}

/*void Fader::move() {
  int16_t delta = _currentPosition - _targetPosition; // delta between read value and wanted value
  uint16_t abs_delta = abs(delta);
  if (!_touched && abs_delta > 50) {  // delta greater than deadband we can move and fader not touched
    if (!_moving) {                   // fader was not moving we set speed depending on delta
      _motorSpeed = 255;
      _moving = true;
    }     
    _DCMotor->setSpeed(_motorSpeed);
    if (delta > 0) {
      _DCMotor->backward();
    } else {
      _DCMotor->forward();
    }
  } else {
    _motorSpeed = 255;
    _DCMotor->stop();
    _targetPosition = _currentPosition;
    _moving = false;
  }  
}*/

