#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int enA, int in1, int in2, int encoderA, int encoderB);
    void begin();
    void setSpeed(double output);
    void forward();
    void backward();
    void stop();
    long readEncoder();
    void resetEncoder();
    double getSpeed();
    void PID(double output);
    double calculateAngularVelocity(long _encoderCount, unsigned long currentTime);

private:
    int _enA;
    int _in1;
    int _in2;
    int _encoderA;
    int _encoderB;
    volatile long _encoderCount;
    long _lastTime;
    long _lastPosition;
    static void encoderISR();
    double output;
};

#endif // MOTOR_H
