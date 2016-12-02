// This script is used as interface to a simulator
#define OSX 0
#define WINDOWS 1
#define UBUNTU 2

#define FOIL_RAKE_PLUS_PIN 0 
#define FOIL_RAKE_MINUS_PIN 1
#define ELEV_RAKE_PLUS_PIN 2 
#define ELEV_RAKE_MINUS_PIN 3
volatile unsigned long last_micros;
int debouncing_time=30;
int delay_read = 0;

#include "Keyboard.h"

// change this to match your platform:
int platform = WINDOWS;

// Variables will change:
int buttonState0;             // the current reading from the input pin
int lastButtonState0 = LOW;   // the previous reading from the input pin
int buttonState1;             // the current reading from the input pin
int lastButtonState1 = LOW;   // the previous reading from the input pin
int buttonState2;             // the current reading from the input pin
int lastButtonState2 = LOW;   // the previous reading from the input pin
int buttonState3;             // the current reading from the input pin
int lastButtonState3 = LOW;   // the previous reading from the input pin

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime0 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime2 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime3 = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


void setup() {
  // initialize serial communication at 9600 bits per second:
   // Serial.begin(9600);
  Keyboard.begin();
  pinMode(FOIL_RAKE_PLUS_PIN, INPUT_PULLUP);
  pinMode(FOIL_RAKE_MINUS_PIN, INPUT_PULLUP);
  pinMode(ELEV_RAKE_PLUS_PIN, INPUT_PULLUP);
  pinMode(ELEV_RAKE_MINUS_PIN, INPUT_PULLUP);
}

void loop() {
  int reading0= digitalRead(FOIL_RAKE_PLUS_PIN);
  int reading1= digitalRead(FOIL_RAKE_MINUS_PIN);
  int reading2= digitalRead(ELEV_RAKE_PLUS_PIN);
  int reading3= digitalRead(ELEV_RAKE_MINUS_PIN);
  if (reading0!=lastButtonState0){
    lastDebounceTime0 = millis();
  }
  if (reading1!=lastButtonState1){
    lastDebounceTime1 = millis();
  }
  if (reading2!=lastButtonState2){
    lastDebounceTime2 = millis();
  }
  if (reading3!=lastButtonState3){
    lastDebounceTime3 = millis();
  }
  if ((millis()-lastDebounceTime0)>debounceDelay){
    if (reading0!=buttonState0){
      buttonState0= reading0;

          // only toggle the LED if the new button state is HIGH
      if (buttonState0 == LOW) {
        increaseFoilRake();
      }
      }
  }
    if ((millis()-lastDebounceTime1)>debounceDelay){
    if (reading1!=buttonState1){
      buttonState1= reading1;

          // only toggle the LED if the new button state is HIGH
      if (buttonState1 == LOW) {
        decreaseFoilRake();
      }
      }
  }
    if ((millis()-lastDebounceTime2)>debounceDelay){
    if (reading2!=buttonState2){
      buttonState2= reading2;

          // only toggle the LED if the new button state is HIGH
      if (buttonState2 == LOW) {
        increaseElevatorRake();
      }
      }
  }
  if ((millis()-lastDebounceTime3)>debounceDelay){
    if (reading3!=buttonState3){
      buttonState3= reading3;

          // only toggle the LED if the new button state is HIGH
      if (buttonState3 == LOW) {
        decreaseElevatorRake();
      }
    }
  }
  lastButtonState0 = reading0;
  lastButtonState1 = reading1;
  lastButtonState2 = reading2;
  lastButtonState3 = reading3;
}
void increaseFoilRake()
{
    //Serial.println("KEY_UP_ARROW");
    Keyboard.write(KEY_UP_ARROW);
}
void decreaseFoilRake()
{
    //Serial.println("KEY_DOWN_ARROW");
    Keyboard.write(KEY_DOWN_ARROW);
}
void increaseElevatorRake()
{
    //Serial.println("KEY_RIGHT_ARROW");
    Keyboard.write(KEY_RIGHT_ARROW);
}
void decreaseElevatorRake()
{
    //Serial.println("KEY_LEFT_ARROW");
    Keyboard.write(KEY_LEFT_ARROW);
}


