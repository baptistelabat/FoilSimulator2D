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

void setup() {
  // initialize serial communication at 9600 bits per second:
   // Serial.begin(9600);
  Keyboard.begin();
  pinMode(FOIL_RAKE_PLUS_PIN, INPUT_PULLUP);
  pinMode(FOIL_RAKE_MINUS_PIN, INPUT_PULLUP);
  pinMode(ELEV_RAKE_PLUS_PIN, INPUT_PULLUP);
  pinMode(ELEV_RAKE_MINUS_PIN, INPUT_PULLUP);
  //digitalWrite(FOIL_RAKE_PLUS_PIN, HIGH);       // turn on pullup resistors
   //digitalWrite(FOIL_RAKE_MINUS_PIN, HIGH);       // turn on pullup resistors
   //digitalWrite(ELEV_RAKE_PLUS_PIN, HIGH);       // turn on pullup resistors
   //digitalWrite(ELEV_RAKE_MINUS_PIN, HIGH);       // turn on pullup resistors
  
  attachInterrupt(digitalPinToInterrupt(FOIL_RAKE_PLUS_PIN), debounceIncreaseFoilRake, FALLING);
  attachInterrupt(digitalPinToInterrupt(FOIL_RAKE_MINUS_PIN), debounceDecreaseFoilRake, FALLING);
  attachInterrupt(digitalPinToInterrupt(ELEV_RAKE_PLUS_PIN), debounceIncreaseElevatorRake, FALLING);
  attachInterrupt(digitalPinToInterrupt(ELEV_RAKE_MINUS_PIN), debounceDecreaseElevatorRake, FALLING);
}

void loop() {
  delay(10);
}
void debounceIncreaseFoilRake()
{
  if ((long)(micros()-last_micros)>=debouncing_time*1000){
    increaseFoilRake();
    last_micros=micros();
  }
}
void increaseFoilRake()
{
    //Serial.println("KEY_UP_ARROW");
    Keyboard.write(KEY_UP_ARROW);
}
void debounceDecreaseFoilRake()
{
  if ((long)(micros()-last_micros)>=debouncing_time*1000){
    decreaseFoilRake();
    last_micros=micros();
  }
}
void decreaseFoilRake()
{
    //Serial.println("KEY_DOWN_ARROW");
    Keyboard.write(KEY_DOWN_ARROW);
}
void debounceIncreaseElevatorRake()
{
  if ((long)(micros()-last_micros)>=debouncing_time*1000){
    increaseElevatorRake();
    last_micros=micros();
  }
}
void increaseElevatorRake()
{
    //Serial.println("KEY_RIGHT_ARROW");
    Keyboard.write(KEY_RIGHT_ARROW);
}
void debounceDecreaseElevatorRake()
{
  if ((long)(micros()-last_micros)>=debouncing_time*1000){
    decreaseElevatorRake();
    last_micros=micros();
  }
}
void decreaseElevatorRake()
{
    //Serial.println("KEY_LEFT_ARROW");
    Keyboard.write(KEY_LEFT_ARROW);
}


