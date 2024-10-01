#include <Arduino.h>
// #include <bluetoothSerial.h>
#include "config.h"

void processSerialInput(HardwareSerial &thisserial);
void getSerialInput();
void printSpeeds();
void printEncoders();
void sendEncoder();

long currentTime = 0;
//debug timing
long d_previousTime = 0;

//pi comm timing
long p_previousTime = 0;

long lastReceivedTime = 0;

// BluetoothSerial SerialBt;

void setup()
{
  // Used to display information
  Serial.begin(9600);
  // SerialBt.begin("ESP32");
  
  Serial2.begin(115200);

  // if ( gpio_install_isr_service(0) != ESP_OK) {
  //   Serial.println("Failed to install ISR service");
  // }

  // ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // enc0.attachHalfQuad(ENCODER0_PIN_A, ENCODER0_PIN_B);
  // enc0.clearCount();
  // enc1.attachHalfQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
  // enc1.clearCount();
  // enc2.attachHalfQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
  // enc2.clearCount();
  // enc3.attachHalfQuad(ENCODER3_PIN_A, ENCODER3_PIN_B);
  // enc3.clearCount();

  lastReceivedTime = millis();
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
  }

  Serial.println("Setup Complete");
}

void loop()
{ 
  currentTime = millis();

  if(currentTime - d_previousTime >= 1000){
      printSpeeds();
      printEncoders();
      d_previousTime = currentTime;
  }

  if(currentTime - lastReceivedTime >= 200){
    // Serial.println("No command received");
    lastReceivedTime = currentTime;
    for(int i = 0; i < 4; i++){
      motors[i].stop();
    }
  }

  if(currentTime - p_previousTime >= 100){
    
    p_previousTime = currentTime;
    sendEncoder();
  }

  processSerialInput(Serial);
  processSerialInput(Serial2);
}

void processSerialInput(HardwareSerial &thisserial) {
  if (thisserial.available()) {
    char input[32];
    thisserial.readBytesUntil('\n', input, 31);
    input[31] = '\0'; // Ensure null-terminated string

    char* command = strtok(input, " "); //tokenise input on spaces
    if (command != nullptr && strcmp(command, "m") == 0) {
      char* motorStr = strtok(NULL, " ");
      char* speedStr = strtok(NULL, " ");
      if (motorStr != nullptr && speedStr != nullptr) {

        unsigned long motor = atoi(motorStr);
        if (motor < 0 || motor >= sizeof(motors) / sizeof(Motor)) {
          Serial.println("Invalid motor number");
          return;
        }

        int speed = atoi(speedStr);
        if(speed < -30 || speed > 30) {
          Serial.println("Invalid speed");
          return;
        }

        int targetSpeed = map(abs(speed), 0, 11, 0, 255);
          motors[motor].setSpeed(targetSpeed);
        if(speed > 0){
          motors[motor].forward();
        } else if(speed < 0) {
          motors[motor].backward();
        } else {
          motors[motor].stop();
        }
        
        // Serial.print("Motor: ");
        // Serial.print(motor);
        // Serial.print(", Speed: ");
        // Serial.println(targetSpeed);
      } else {
        Serial.println("Invalid input format");
      }
    } else {
      Serial.print("Unknown command: ");
      Serial.print(input);
      Serial.println();
    }
  }
}

void printSpeeds(){
    Serial.println();
  for (size_t i = 0; i < 4; i++)
  {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" speed: ");
    Serial.println(motors[i].getSpeed());
  }
}

void printEncoders(){
    Serial.println();
  for (size_t i = 0; i < 4; i++)
  {
    double count = motors[i].readEncoder();
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" encoder: ");
    Serial.println(count);

  }
}

void sendEncoder(){
  long count = motors[0].readEncoder();
  long count1 = motors[1].readEncoder();
  long count2 = motors[2].readEncoder();
  long count3 = motors[3].readEncoder();

  char message[32];
  snprintf(message, sizeof(message), "%li,%li,%li,%li", count, count1, count2, count3);
  Serial2.println(message);
}

// /// @brief protocol 1 byte per pair of wheels first bit is which pair, second direction, last 6 is speed
// void getSerialInput(){
//   if(Serial2.available()){
//     uint8_t command;
//     Serial2.readBytes(&command,1);
//     uint8_t motor = command >> 7;
//     uint8_t direction =  (command >> 6) & 0x01;
//     uint8_t speed = command & 0x3F;
//   }
// }
