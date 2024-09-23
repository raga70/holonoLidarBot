#include <Arduino.h>
#include "config.h"
#include <L298N.h>
void processSerialInput();
void getSerialInput();
void printSpeeds();
void printEncoders();

L298N motor = L298N(2, 35, 37);

void setup()
{
  // Used to display information
  Serial.begin(9600);
  Serial2.begin(9600);
  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    // do nothing
  }
  for (size_t i = 0; i < 4; i++)
  {
    motors[i].begin();
    Serial.println("Motor ");
    Serial.print(i);
    Serial.println(" started");
    // motors[i].setSpeed(255);
    // motors[i].forward();
  }
  
  // motor.forward();
}

void loop()
{
  processSerialInput();
  // printSpeeds();
  // printEncoders();
  delay(10);
}

void processSerialInput() {
  if (Serial.available()) {
    char input[32];
    Serial.readBytesUntil('\n', input, 31);
    input[31] = '\0'; // Ensure null-terminated string

    char* command = strtok(input, " "); //tokenise input on spaces
    if (command != nullptr && strcmp(command, "m") == 0) {
      char* motorStr = strtok(NULL, " ");
      char* speedStr = strtok(NULL, " ");
      if (motorStr != nullptr && speedStr != nullptr) {

        int motor = atoi(motorStr);
        if (motor < 0 || motor >= sizeof(motors) / sizeof(Motor)) {
          Serial.println("Invalid motor number");
          return;
        }

        int speed = atoi(speedStr);
        if(speed < -30 || speed > 30) {
          Serial.println("Invalid speed");
          return;
        }

        int targetSpeed = map(abs(speed), 0, 30, 0, 255);
          motors[motor].setSpeed(targetSpeed);
        if(speed > 0){
          motors[motor].forward();
        } else if(speed < 0) {
          motors[motor].backward();
        } else {
          motors[motor].stop();
        }
        
        Serial.print("Motor: ");
        Serial.print(motor);
        Serial.print(", Speed: ");
        Serial.println(targetSpeed);
        // Add code to control motor here
      } else {
        Serial.println("Invalid input format");
      }
    } else {
      Serial.println("Unknown command");
    }
  }
}

void printSpeeds(){
  for (size_t i = 0; i < 4; i++)
  {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" speed: ");
    Serial.println(motors[i].getSpeed());
  }
}

void printEncoders(){
  for (size_t i = 0; i < 4; i++)
  {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" encoder: ");
    Serial.println(motors[i].readEncoder());
  }
}

/// @brief protocol 1 byte per pair of wheels first bit is which pair, second direction, last 6 is speed
void getSerialInput(){
  if(Serial2.available()){
    uint8_t command;
    Serial2.readBytes(&command,1);
    uint8_t motor = command >> 7;
    uint8_t direction =  (command >> 6) & 0x01;
    uint8_t speed = command & 0x3F;
  }
}