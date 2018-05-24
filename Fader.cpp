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
  long cs = _cs->capacitiveSensor(5);
  if (!_touched && cs >= 150)    
    _touched = true;
  else if (_touched && cs < 50)
    _touched = false;

  if (_touched) {
    _midiUpdate = true;
  }
}

bool Fader::needMidiUpdate() {
  return _midiUpdate;  
}

uint16_t Fader::readCurrentPosition() {
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

uint16_t Fader::getTargetPosition() {
  return _targetPosition;
}

void Fader::setTargetPosition(uint16_t position) {
  _targetPosition = position;
}

uint16_t Fader::getMotorSpeed() {
  return _motorSpeed;
}

void Fader::setMotorSpeed(uint16_t speed) {
  _motorSpeed = speed;
}

void Fader::setDelta(uint16_t delta) {
  _delta = delta;
}

uint16_t Fader::getDelta() {
  return _delta;
}

uint16_t Fader::getPosition() {
  return _position;
}

void Fader::setPosition(uint16_t position) {
  _position = position;
}

void Fader::move() {
  static long waitBeforeMove;
  if (_suspended)
    return;
    
  bool lastMoving = _moving;
  int16_t delta = _currentPosition - _targetPosition; // delta between read value and wanted value
  uint16_t abs_delta = abs(delta);
  if (!_touched && abs_delta > MF_DEADBAND) {  // delta greater than deadband we can move and fader not touched
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
      *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
      *portOutputRegister(_motorPin2Port) |= _motorPin2Bit;
    } else {        // move up
      *portOutputRegister(_motorPin1Port) |= _motorPin1Bit;
      *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);
    }
  } else { // stop
    _motorSpeed = 255;
    *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
    *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);
    _targetPosition = _currentPosition;
    _moving = false;
  }

/*  if (!_suspended && lastMoving && !_moving) { // fader just stops wait before moving again
    _suspended = true;
    waitBeforeMove = millis();
  }

  if (_suspended && millis() - waitBeforeMove > 2000) {
    _suspended = false;
  }*/
}

/*void Fader::move() {
  // skip if fader directly controlled

  // counter handling
  if( _manual_move_ctr )
  --_manual_move_ctr;
  if( _timeout_ctr )
  --_timeout_ctr;
  
  // check touch detection, shutdown motor if active
  if( _touched ) {
    // no repeats
    _repeat_ctr = 0;
    // no timeout
    _timeout_ctr = 0;
  }

  // check motor state
  _idle = 0;

  // CASE: motor on target position?
  if( !_repeat_ctr ) {
    // motor should go into standby mode
    _direction = MF_Standby;
    _idle = 1;

    // timeout reached?
    if( _timeout_ctr ) {
      // no: copy current AIN value into target position (reassurance phase)
      _targetPosition = _currentPosition;
    } else {
      // AIN value was outside deadband?
      if( _delta > AIN_DEADBAND ) {
        // copy current AIN value into target position
        _targetPosition = _currentPosition;
        // set manual move counter, so that the motor won't be moved during this time
        _manual_move_ctr = MANUAL_MOVE_CTR_RELOAD;
        // change flag should not be cleared
      }
    }
  }
  // CASE: motor very slow or not moving
  else if( !_timeout_ctr && _delta <= AIN_DEADBAND ) {
    // if timeout reached, write 1 into repeat counter for proper shutdown
    if( _timeout_ctr == 0 ) {
      _repeat_ctr = 1;
      _idle = 1;
    }
  }
  // CASE: motor is moving fast
  else if( _delta > AIN_DEADBAND ) {
    // fine: reload timeout counter
    _timeout_ctr = TIMEOUT_CTR_RELOAD;
  }
  
  // continue if motor control hasn't reached idle state
  if( !_idle ) {
    // don't move motor if speed too fast
    if( _delta > 500 ) { // TODO: check with panasonic faders
      _direction = MF_Standby;
    } else {
      // determine into which direction the motor should be moved

      // special case: if target and current position <= 0x01f, stop motor
      // (workaround for ALPS faders which never reach the 0x000 value)
      if( _targetPosition < 0x1f && _currentPosition < 0x1f ) {
        _direction = MF_Standby;
        --_repeat_ctr;
      } else {
        // check distance between current and target position
        // if fader is in between the MF deadband, don't move it anymore
        int16_t mf_delta = _currentPosition - _targetPosition;
        uint16_t abs_mf_delta = abs(mf_delta);
  
        // dynamic deadband: depends on repeat counter
        uint8_t dyn_deadband;
        if( _repeat_ctr < 4 )
          dyn_deadband = 16;
        else if( _repeat_ctr < 8 )
          dyn_deadband = 32;
        else if( _repeat_ctr < 16 )
          dyn_deadband = 64;
        else
          dyn_deadband = MF_DEADBAND;
    
        if( abs_mf_delta <= dyn_deadband ) {
          _idle = 1;
          --_repeat_ctr;
        }

        // slow down motor via PWM if distance between current and target position < 0x180
        if( MF_PWM_PERIOD && abs_mf_delta < 0x180 ) {
          _motorSpeed = map(abs_mf_delta, 0, 0x180, 0, 200);
          if( ++_pwm_ctr > MF_PWM_PERIOD )
            _pwm_ctr = 0;
          
          if( _pwm_ctr > ((mf_delta > 0) ? MF_DUTY_CYCLE_DOWN : MF_DUTY_CYCLE_UP) ) {
            _idle = 1;
          }
        } else 
          _motorSpeed = 255;

        // check if motor should be moved up/down
        if( _idle )
          _direction = MF_Standby;
        else
          _direction = (mf_delta > 0) ? MF_Down : MF_Up;
      }
    }
  }

  analogWrite(_enablePin, _motorSpeed);
  if (_direction == MF_Up) {
    *portOutputRegister(_motorPin1Port) |= _motorPin1Bit;
    *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);   
  } else if (_direction == MF_Down) {
    *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
    *portOutputRegister(_motorPin2Port) |= _motorPin2Bit;    
  } else {
    *portOutputRegister(_motorPin1Port) &= ~(_motorPin1Bit);
    *portOutputRegister(_motorPin2Port) &= ~(_motorPin2Bit);    
  }
}*/

