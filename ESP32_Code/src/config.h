#ifndef CONFIG_H
#define CONFIG_H
#include "motor.h"
// #define ENCODER_DO_NOT_USE_INTERRUPTS
// #define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
// #include <ESP32Encoder.h>
// int pulsesPerRevolution = 1440;

#if defined(__AVR_ATmega2560__)
//front left
// Motor 1 Pins
#define MOTOR0_PIN_EN 2 
#define MOTOR0_PIN_IN1 35
#define MOTOR0_PIN_IN2 37
// Encoder 1 Pins
#define ENCODER0_PIN_A 18   //blue 
#define ENCODER0_PIN_B 23   //yellow

//front right
// Motor 2 Pins
#define MOTOR1_PIN_EN 3
#define MOTOR1_PIN_IN1 41
#define MOTOR1_PIN_IN2 39
// Encoder 2 Pins
#define ENCODER1_PIN_A 19
#define ENCODER1_PIN_B 25

//back left
// Motor 3 Pins
#define MOTOR2_PIN_EN 4
#define MOTOR2_PIN_IN1 31
#define MOTOR2_PIN_IN2 33
// Encoder 3 Pins
#define ENCODER2_PIN_A 20
#define ENCODER2_PIN_B 27

//back right
// Motor 4 Pins
#define MOTOR3_PIN_EN 5
#define MOTOR3_PIN_IN1 45
#define MOTOR3_PIN_IN2 43
// Encoder 4 Pins
#define ENCODER3_PIN_A 21
#define ENCODER3_PIN_B 29
#elif defined(ESP32)
//front left
// Motor 0 Pins
#define MOTOR0_PIN_EN 5 
#define MOTOR0_PIN_IN1 16
#define MOTOR0_PIN_IN2 17
// Encoder 0 Pins
#define ENCODER0_PIN_A 13   //blue 
#define ENCODER0_PIN_B 33   //yellow

//front right
// Motor 1 Pins
#define MOTOR1_PIN_EN 18
#define MOTOR1_PIN_IN1 19
#define MOTOR1_PIN_IN2 21
// Encoder 1 Pins
#define ENCODER1_PIN_A 14
#define ENCODER1_PIN_B 32

//back left
// Motor 2 Pins
#define MOTOR2_PIN_EN 15
#define MOTOR2_PIN_IN1 2
#define MOTOR2_PIN_IN2 4
// Encoder 2 Pins
#define ENCODER2_PIN_A 27
#define ENCODER2_PIN_B 35

//back right
// Motor 3 Pins
#define MOTOR3_PIN_EN 25
#define MOTOR3_PIN_IN1 22
#define MOTOR3_PIN_IN2 23
// Encoder 3 Pins
#define ENCODER3_PIN_A 26
#define ENCODER3_PIN_B 34
#endif

Encoder enc0(ENCODER0_PIN_A, ENCODER0_PIN_B);
Encoder enc1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder enc2(ENCODER2_PIN_A, ENCODER2_PIN_B);
Encoder enc3(ENCODER3_PIN_A, ENCODER3_PIN_B);

// ESP32Encoder enc0(true);
// ESP32Encoder enc1(true);
// ESP32Encoder enc2(true);
// ESP32Encoder enc3(true);

Motor m0(MOTOR0_PIN_EN, MOTOR0_PIN_IN1, MOTOR0_PIN_IN2, enc0);
Motor m1(MOTOR1_PIN_EN, MOTOR1_PIN_IN1, MOTOR1_PIN_IN2, enc1);
Motor m2(MOTOR2_PIN_EN, MOTOR2_PIN_IN1, MOTOR2_PIN_IN2, enc2);
Motor m3(MOTOR3_PIN_EN, MOTOR3_PIN_IN1, MOTOR3_PIN_IN2, enc3);

Motor motors[] = {m0,m1,m2,m3};

// INT.0 on Pin 21
// INT.1 on pin 20
// INT.2 on pin 19
// INT.3 on Pin 18
// INT.4 on Pin 2
// INT.5 on Pin 3
#endif // CONFIG_H