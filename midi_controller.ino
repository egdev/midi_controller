#include <MIDI.h>
#include "InputDebounce.h"
#include "Fader.h"

#define BUTTON_DEBOUNCE_DELAY   20
static const int changeBankPin = 14;
static InputDebounce changeBankButton;

static const int ledBank1Pin = 15;
static const int ledBank2Pin = 16;
static const int ledBank3Pin = 17;

const uint16_t MAX_BANKS = 2;//(NUM_CHANNELS / (NUM_FADERS - 1));

MIDI_CREATE_DEFAULT_INSTANCE();
volatile uint8_t currentBank = 0;

// Fader(signalPin, enablePin, motorPin1, motorPin2, receiveTouchPin, sendTouchPin, master)
/*Fader FADERS[NUM_FADERS] = { Fader(0,  10, 19, 18, 53, 52, true), // master fader
                             Fader(8,  2,  35, 34, 51, 50, false),
                             Fader(7,  3,  33, 32, 49, 48, false),
                             Fader(6,  4,  31, 30, 47, 46, false),
                             Fader(5,  5,  29, 28, 45, 44, false),
                             Fader(4,  6,  27, 26, 43, 42, false),
                             Fader(3,  7,  25, 24, 41, 40, false),
                             Fader(2,  8,  23, 22, 39, 38, false),
                             Fader(1,  9,  21, 20, 37, 36, false)                             
                            };*/

Fader FADERS[NUM_FADERS] = { Fader(0,  10, 19, 18, 53, 52, true), // master fader
                             Fader(8,  2,  35, 34, 37, 36, false),                             
                             Fader(7,  3,  33, 32, 39, 38, false),
                             Fader(6,  4,  31, 30, 41, 40, false),
                             Fader(5,  5,  29, 28, 43, 42, false),
                             Fader(4,  6,  27, 26, 45, 44, false),
                             Fader(3,  7,  25, 24, 47, 46, false),
                             Fader(2,  8,  23, 22, 49, 48, false),
                             Fader(1,  9,  21, 20, 51, 50, false)
                            };

struct Track {
  midi::Channel channel;
  uint16_t position;  // 10-bit resolution 0-1023
  midi::MidiControlChangeNumber number;  
};

Track tracks[NUM_CHANNELS] = { /*{ 1, 0, 31}, // master track
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
                               { 1, 0, 15},
                               { 1, 0, 16}*/
                               /*,*/
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
                               { 16,  0, midi::MidiControlChangeNumber::ChannelVolume} }; // save all channels positions

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
    /*digitalWrite(ledBank1Pin, (i % 3 == 0) ? HIGH : LOW);
    digitalWrite(ledBank2Pin, (i % 3 == 1) ? HIGH : LOW);
    digitalWrite(ledBank3Pin, (i % 3 == 2) ? HIGH : LOW);*/
    bitWrite(PORTJ, 0, (i % 3) == 0);
    bitWrite(PORTH, 1, (i % 3) == 1);
    bitWrite(PORTH, 0, (i % 4) == 2);
  }
  /*digitalWrite(ledBank1Pin, LOW);
  digitalWrite(ledBank2Pin, LOW);
  digitalWrite(ledBank3Pin, LOW);*/
  bitClear(PORTJ, 0);
  bitClear(PORTH, 1);
  bitClear(PORTH, 0);
}

long t;

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
        /*Serial.print("MIDI.sendControlChange(");
        Serial.print(tracks[faderChannelIndex].number);
        Serial.print(", ");
        //Serial.print(FADERS[i].faderPositionToMidiPosition(pos));
        //Serial.print(pos);
        Serial.print(map(pos, FADERS[i].getMinPosition(), FADERS[i].getMaxPosition(), 0, 127));
        Serial.print(", ");
        Serial.print(tracks[faderChannelIndex].channel);
        Serial.println(")");*/
        //Serial.println("move fader ");
        /*if (!i)
          Serial.print("MASTER");
         else
          Serial.print(i);
         Serial.print(" to position ");
         Serial.println(FADERS[i].faderPositionToMidiPosition(pos));*/
      }
      FADERS[i].setMidiUpdate(false);
    }
    //analogRead(A8);
    //Serial.print("time for read = ");
    FADERS[i].updateCurrentPosition();
    FADERS[i].move();
    /*Serial.print(analogRead(FADERS[i].getSignalPin()));
    Serial.print("\t");*/
    //if (i == NUM_FADERS-1)
    //  Serial.println("");
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
    uint16_t faderChannelIndex = ((currentBank * 8) + i); //((currentBank-1)*(NUM_FADERS - 1)) + i;
    FADERS[i].setTargetPosition(tracks[faderChannelIndex].position);
  }
}

void setup() {
  /*pinMode(ledBank1Pin, OUTPUT); //15
  pinMode(ledBank2Pin, OUTPUT); //16
  pinMode(ledBank3Pin, OUTPUT); //17*/
  bitSet(DDRJ, 0);
  bitSet(DDRH, 1);
  bitSet(DDRH, 0);
  
  changeBankButton.registerCallbacks(NULL, changeBank, NULL);
  changeBankButton.setup(changeBankPin, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
  TCCR3B = TCCR3B & 0b11111000 | 0x01;
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  //Serial.begin(9600);
  
  //calibrateFaders();
}

/*long counter = 0;
long lastMicros = 0;
long currentMicros;
double totalMicros = 0;*/

void loop() {  
  // currentMicros = micros();
  changeBankButton.process(millis());
  MIDI.read();
  manageFaders();  

 bitWrite(PORTJ, 0, currentBank == 0);
 bitWrite(PORTH, 1, currentBank == 1);
 bitWrite(PORTH, 0, currentBank == 2);
 /* digitalWrite(ledBank1Pin, (currentBank == 0) ? HIGH : LOW);
  digitalWrite(ledBank2Pin, (currentBank == 1) ? HIGH : LOW);
  digitalWrite(ledBank3Pin, (currentBank == 2) ? HIGH : LOW);*/

  /*int i;
  for (i=0; i<NUM_FADERS; i++) 
  {
    Serial.print(i);
    Serial.print(" : min = ");
    Serial.print(FADERS[i].getMinPosition());
    Serial.print(", max = ");
    Serial.println(FADERS[i].getMaxPosition());
  }*/
  //Serial.println(currentBank);
/*  digitalWrite(ledBank4Pin, (currentBank == 4) ? HIGH : LOW);
  digitalWrite(ledBank5Pin, (currentBank == 5) ? HIGH : LOW);*/
  
  /*Serial.print("Channel 1  = ");
  Serial.println(analogRead(A8));

  Serial.print("Channel 2  = ");
  Serial.println(analogRead(A7));

  Serial.print("Channel 3  = ");
  Serial.println(analogRead(A6));

  Serial.print("Channel 4  = ");
  Serial.println(analogRead(A5));

  Serial.print("Channel 5  = ");
  Serial.println(analogRead(A4));

  Serial.print("Channel 6  = ");
  Serial.println(analogRead(A3));

  Serial.print("Channel 7  = ");
  Serial.println(analogRead(A2));

  Serial.print("Channel 8  = ");
  Serial.println(analogRead(A1));

  Serial.print("Master  = ");
  Serial.println(analogRead(A0));*/

  //Serial.println(FADERS[8]._cs->capacitiveSensor(30));

  //Serial.println(FADERS[1].updateCurrentPosition());
  //delay(1000);
  //Serial.print("loop = ");
    //Serial.println(FADERS[1]._cs->capacitiveSensor(5));
/*  if ((currentMicros - lastMicros) >= 1000000)
  {
    long t = micros();
    Serial.println(FADERS[1]._cs->capacitiveSensor(5));
    //Serial.println(micros() - t);
    //Serial.println(totalMicros / counter);
    //Serial.println(counter);
    counter = 0;
    totalMicros = 0;
    lastMicros = currentMicros;
  } else {
    totalMicros += (micros() - currentMicros);
    counter++;
  }*/
  //Serial.println(micros()-t);
  
}

