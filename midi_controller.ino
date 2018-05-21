#include <MIDI.h>
#include "InputDebounce.h"
#include "Fader.h"

#define BUTTON_DEBOUNCE_DELAY   20
static const int changeBankPin = 14;
static InputDebounce changeBankButton;

const uint16_t MAX_BANKS = 2;//(NUM_CHANNELS / (NUM_FADERS - 1));

MIDI_CREATE_DEFAULT_INSTANCE();
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

/*Fader FADERS[NUM_FADERS] = { Fader(0,  10, 19, 18, 53, 52, true,  PORTB, 4, PORTD, 2, PORTD, 3), // master fader
                             Fader(8,  2,  35, 34, 37, 36, false, PORTE, 4, PORTC, 2, PORTC, 3),                             
                             Fader(7,  3,  33, 32, 39, 38, false, PORTE, 5, PORTC, 4, PORTC, 5),
                             Fader(6,  4,  31, 30, 41, 40, false, PORTG, 5, PORTC, 6, PORTC, 7),
                             Fader(5,  5,  29, 28, 43, 42, false, PORTE, 3, PORTA, 7, PORTA, 6),
                             Fader(4,  6,  27, 26, 45, 44, false, PORTH, 3, PORTA, 5, PORTA, 4),
                             Fader(3,  7,  25, 24, 47, 46, false, PORTH, 4, PORTA, 3, PORTA, 2),
                             Fader(2,  8,  23, 22, 49, 48, false, PORTH, 5, PORTA, 1, PORTA, 0),
                             Fader(1,  9,  21, 20, 51, 50, false, PORTH, 6, PORTD, 0, PORTD, 1)
                            };*/

struct Track {
  midi::Channel channel;
  uint16_t position;  // 10-bit resolution 0-1023
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
                               /*,
                               { 17,  0, 95},
                               { 1,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 2,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 3,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 4,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 5,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 6,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 7,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 8,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 9,   0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 10,  0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 11,  0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 12,  0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 13,  0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 14,  0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 15,  0, midi::MidiControlChangeNumber::ChannelVolume},
                               { 16,  0, midi::MidiControlChangeNumber::ChannelVolume}*/
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
    FADERS[faderIndex].setTargetPosition(FADERS[faderIndex].midiPositionToFaderPosition(value));

  tracks[channelIndex].position = map(value, 0, 127, 0, 1023);
}

void calibrateFaders() {
  int i;
  for (i=0; i<NUM_FADERS; i++) 
  {
    FADERS[i].calibrate();
    bitWrite(PORTJ, 0, (i % 3) == 0);
    bitWrite(PORTH, 1, (i % 3) == 1);
    bitWrite(PORTH, 0, (i % 4) == 2);
    //break;
  }
  bitClear(PORTJ, 0);
  bitClear(PORTH, 1);
  bitClear(PORTH, 0);
}

void manageFaders() {
  int i;
  for (i=0; i<NUM_FADERS; i++) {
    FADERS[i].checkTouched();
    if (FADERS[i].needMidiUpdate()) { // we need to update corresponding fader channel position on remote
      uint16_t faderChannelIndex;
      if (i == 0)
        faderChannelIndex = 0;
      else
        faderChannelIndex = ((currentBank * 8) + i);
      
      if (faderChannelIndex < NUM_CHANNELS) {
        uint16_t pos = FADERS[i].updateCurrentPosition();
        FADERS[i].setTargetPosition(pos);
        tracks[faderChannelIndex].position = pos; // save channel position        
        MIDI.sendControlChange(tracks[faderChannelIndex].number, FADERS[i].faderPositionToMidiPosition(pos), tracks[faderChannelIndex].channel); 
      }
      FADERS[i].setMidiUpdate(false);
    } 
    FADERS[i].updateCurrentPosition();
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
  
  changeBankButton.registerCallbacks(NULL, changeBank, NULL);
  changeBankButton.setup(changeBankPin, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  calibrateFaders();
}

void loop() {  
  changeBankButton.process(millis());
  MIDI.read();
  manageFaders();  

  bitWrite(PORTJ, 0, currentBank == 0); // pin 15
  bitWrite(PORTH, 1, currentBank == 1); // pin 16
  bitWrite(PORTH, 0, currentBank == 2); // pin 17
}

