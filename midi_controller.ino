#include <MIDI.h>
#include "InputDebounce.h"
#include "Fader.h"

#define BUTTON_DEBOUNCE_DELAY   20
static const int changeBankPin = 14;
static InputDebounce changeBankButton;

static const int led1Pin = 15;
static const int led2Pin = 16;
static const int led3Pin = 17;

const uint16_t MAX_BANKS = 2;//(NUM_CHANNELS / (NUM_FADERS - 1));

struct MySettings : public midi::DefaultSettings
{
  static const bool Use1ByteParsing = false;
};
 
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MySettings);

volatile uint8_t currentBank = 0;

// Fader(signalPin, enablePin, motorPin1, motorPin2, receiveTouchPin, sendTouchPin, master)
// GOOD ONES
Fader FADERS[NUM_FADERS] = { Fader(0,  10,  19, 18, 53, 52, true), // master fader
                             Fader(8,  2,   35, 34, 37, 36, false),                             
                             Fader(7,  3,   33, 32, 39, 38, false),
                             Fader(6,  11,  31, 30, 41, 40, false),
                             Fader(5,  5,   29, 28, 43, 42, false),
                             Fader(4,  6,   27, 26, 45, 44, false),
                             Fader(3,  7,   25, 24, 47, 46, false),
                             Fader(2,  8,   23, 22, 49, 48, false),
                             Fader(1,  9,   21, 20, 51, 50, false)
                            };

volatile uint8_t *_led1Port;
volatile uint8_t *_led2Port;
volatile uint8_t *_led3Port;
uint8_t _led1Bit;
uint8_t _led2Bit;
uint8_t _led3Bit;

struct Track {
  midi::Channel channel;
  int position;  // 10-bit resolution 0-1023
  midi::MidiControlChangeNumber number;  
};

Track tracks[NUM_CHANNELS] = { { 1, 0, 31}, // master track
                               { 1, 0, 0},
                               { 1, 0, 1},
                               { 1, 0, 2},
                               { 1, 0, 3},
                               { 1, 0, 4},
                               { 1, 0, 5},
                               { 1, 0, 6},
                               { 1, 0, 7},
                               { 1, 0, 8},
                               { 1, 0, 9},
                               { 1, 0, 10},
                               { 1, 0, 11},
                               { 1, 0, 12},
                               { 1, 0, 13},
                               { 1, 0, 14},
                               { 1, 0, 15}/*,
                               { 1, 0, 16}*/
                            }; // save all channels positions

int16_t getChannelIndex(byte channel, byte number) {
  int16_t index = -1;
  int i;
  for (i=0; i<NUM_CHANNELS && index == -1; i++) {
    if (tracks[i].channel == channel && tracks[i].number == number) // channel found
      index = i;
  }
  return index;  
}

void handleControlChange(byte channel, byte number, byte value) {
  int16_t channelIndex = getChannelIndex(channel, number);
  if (channelIndex == -1)
    return;

  bool needToMoveFader = false;
  uint16_t faderIndex = 0;
  
  if ( channelIndex == 0 ) { // master so need to move
    needToMoveFader = true;
    faderIndex = 0;
  } else {
    int16_t tBank = ((channelIndex-1) / 8);   // get corresponding bank for channelIndex
    needToMoveFader = (tBank == currentBank); // need to move fader only if channelIndex is inside active bank
    faderIndex = (1 + ((channelIndex-1) % 8));
  }
  
  if ( needToMoveFader ) 
  {
    int newPosition = FADERS[faderIndex].midiPositionToFaderPosition(value);
    FADERS[faderIndex].setTargetPosition(newPosition);
  }

  tracks[channelIndex].position = map(value, 0, 127, 0, 1023);
}

void calibrateFaders() {
  int i;
  for (i=0; i<NUM_FADERS; i++) 
  {
    FADERS[i].calibrate();
    if ((i % 3) == 0)
      *_led1Port |= _led1Bit;
    else
      *_led1Port &= ~(_led1Bit);
  
    if ((i % 3) == 1)
      *_led2Port |= _led2Bit;
    else
      *_led2Port &= ~(_led2Bit);
  
    if ((i % 3) == 2)
      *_led3Port |= _led3Bit;
    else
      *_led3Port &= ~(_led3Bit);
  }
  *_led1Port &= ~(_led1Bit);
  *_led2Port &= ~(_led2Bit);
  *_led3Port &= ~(_led3Bit);
}

void manageFaders() {
  int i;
  for (i=0; i<NUM_FADERS; i++) {
    FADERS[i].readCurrentPosition();

    FADERS[i].checkTouched();
    if (FADERS[i].needMidiUpdate()) { // we need to update corresponding fader channel position on remote
      uint16_t faderChannelIndex;
      if (i == 0)
        faderChannelIndex = 0;
      else
        faderChannelIndex = ((currentBank * 8) + i);
      
      if (faderChannelIndex < NUM_CHANNELS) {
        int pos = FADERS[i].readCurrentPosition();
        FADERS[i].setTargetPosition(pos);
        tracks[faderChannelIndex].position = pos; // save channel position        
        MIDI.sendControlChange(tracks[faderChannelIndex].number, FADERS[i].faderPositionToMidiPosition(pos), tracks[faderChannelIndex].channel); 
      }
      FADERS[i].setMidiUpdate(false);
    } 
    FADERS[i].move();    
  }
}

void changeBank() {
   if ( currentBank < (MAX_BANKS-1) ) 
    currentBank++;
   else 
    currentBank = 0;

  // move fader to channels positions (except master)
  int i;
  for (i=1; i<NUM_FADERS; i++) {
    uint16_t faderChannelIndex = ((currentBank * 8) + i);
    FADERS[i].setTargetPosition(tracks[faderChannelIndex].position);
  }
}

void setup() {
  bitSet(DDRJ, 0);
  bitSet(DDRH, 1);
  bitSet(DDRH, 0);

  _led1Bit  = digitalPinToBitMask(led1Pin);
  _led1Port = portOutputRegister(digitalPinToPort(led1Pin));
  _led2Bit  = digitalPinToBitMask(led2Pin);
  _led2Port = portOutputRegister(digitalPinToPort(led2Pin));
  _led3Bit  = digitalPinToBitMask(led3Pin);
  _led3Port = portOutputRegister(digitalPinToPort(led3Pin));

  
  changeBankButton.registerCallbacks(NULL, changeBank, NULL);
  changeBankButton.setup(changeBankPin, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.begin(1);
  MIDI.turnThruOff();
  //Serial.begin(9600);
  calibrateFaders();
}

void loop() {  
  changeBankButton.process(millis());
  MIDI.read();
  noInterrupts();
  manageFaders();  
  interrupts();

  if (currentBank == 0)
    *_led1Port |= _led1Bit;
  else
    *_led1Port &= ~(_led1Bit);

  if (currentBank == 1)
    *_led2Port |= _led2Bit;
  else
    *_led2Port &= ~(_led2Bit);

  if (currentBank == 2)
    *_led3Port |= _led3Bit;
  else
    *_led3Port &= ~(_led3Bit);
}

