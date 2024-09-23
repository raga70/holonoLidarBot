#include "motor.h"

Motor::Motor(int enA, int in1, int in2, int encoderA, int encoderB)
    : _enA(enA), _in1(in1), _in2(in2), _encoderA(encoderA), _encoderB(encoderB), _encoderCount(0) {}

static Motor* motorInstance = nullptr;

void Motor::begin() {
    motorInstance = this;
    pinMode(_enA, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_encoderA, INPUT);
    pinMode(_encoderB, INPUT);
    attachInterrupt(digitalPinToInterrupt(_encoderA), encoderISR, CHANGE);
}

void Motor::setSpeed(int speed) {
    analogWrite(_enA, speed);
}

void Motor::forward() {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
}

void Motor::backward() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
}

void Motor::stop() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
}

long Motor::readEncoder() {
    return _encoderCount;
}

void Motor::resetEncoder() {
    _encoderCount = 0;
}

double Motor::getSpeed()
{
    long currentTime = millis();
    long elapsedTime = currentTime - _lastTime;
    long currentPosition = _encoderCount;
    long deltaPosition = currentPosition - _lastPosition;
    double speed = (double)deltaPosition / elapsedTime;
    _lastTime = currentTime;
    _lastPosition = currentPosition;
    return speed;
}

void Motor::encoderISR() {
    if (motorInstance != nullptr) {
        motorInstance->_encoderCount++;
    }
}