#ifndef CONFIG_H
#define CONFIG_H
#include "motor.h"
// #include <L298N.h>

//front left
// Motor 1 Pins
#define MOTOR1_PIN_EN 2 
#define MOTOR1_PIN_IN1 35
#define MOTOR1_PIN_IN2 37
// Encoder 1 Pins
#define ENCODER1_PIN_A 18   //blue 
#define ENCODER1_PIN_B 23   //yellow

//front right
// Motor 2 Pins
#define MOTOR2_PIN_EN 3
#define MOTOR2_PIN_IN1 41
#define MOTOR2_PIN_IN2 39
// Encoder 2 Pins
#define ENCODER2_PIN_A 19
#define ENCODER2_PIN_B 25

//back left
// Motor 3 Pins
#define MOTOR3_PIN_EN 4
#define MOTOR3_PIN_IN1 31
#define MOTOR3_PIN_IN2 33
// Encoder 3 Pins
#define ENCODER3_PIN_A 20
#define ENCODER3_PIN_B 27

//back right
// Motor 4 Pins
#define MOTOR4_PIN_EN 5
#define MOTOR4_PIN_IN1 45
#define MOTOR4_PIN_IN2 43
// Encoder 4 Pins
#define ENCODER4_PIN_A 21
#define ENCODER4_PIN_B 29

// struct MotorConfig{
//     L298N Motor;
//     int EncA;
//     int EncB;
// };

// MotorConfig motor1 = {L298N(MOTOR1_PIN_EN, MOTOR1_PIN_IN1, MOTOR1_PIN_IN2), ENCODER1_PIN_A, ENCODER1_PIN_B};
// MotorConfig motor2 = {L298N(MOTOR2_PIN_EN, MOTOR2_PIN_IN1, MOTOR2_PIN_IN2), ENCODER2_PIN_A, ENCODER2_PIN_B};
// MotorConfig motor3 = {L298N(MOTOR3_PIN_EN, MOTOR3_PIN_IN1, MOTOR3_PIN_IN2), ENCODER3_PIN_A, ENCODER3_PIN_B};
// MotorConfig motor4 = {L298N(MOTOR4_PIN_EN, MOTOR4_PIN_IN1, MOTOR4_PIN_IN2), ENCODER4_PIN_A, ENCODER4_PIN_B};

Motor m1(MOTOR1_PIN_EN, MOTOR1_PIN_IN1, MOTOR1_PIN_IN2, ENCODER1_PIN_A, ENCODER1_PIN_B);
Motor m2(MOTOR2_PIN_EN, MOTOR2_PIN_IN1, MOTOR2_PIN_IN2, ENCODER2_PIN_A, ENCODER2_PIN_B);
Motor m3(MOTOR3_PIN_EN, MOTOR3_PIN_IN1, MOTOR3_PIN_IN2, ENCODER3_PIN_A, ENCODER3_PIN_B);
Motor m4(MOTOR4_PIN_EN, MOTOR4_PIN_IN1, MOTOR4_PIN_IN2, ENCODER4_PIN_A, ENCODER4_PIN_B);

Motor motors[] = {m1,m2,m3,m4};

// MotorConfig motors[] = {motor1, motor2, motor3, motor4};

// INT.0 on Pin 21
// INT.1 on pin 20
// INT.2 on pin 19
// INT.3 on Pin 18
// INT.4 on Pin 2
// INT.5 on Pin 3
#endif // CONFIG_H