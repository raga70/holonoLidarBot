#include "motor.h"

Motor::Motor(int enA, int in1, int in2, Encoder& encoder)
    : _enA(enA), _in1(in1), _in2(in2),_encoder(encoder), _encoderCount(0) {}

static Motor *motorInstance = nullptr;
void Motor::begin()
{
    motorInstance = this;
    pinMode(_enA, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    // attachInterrupt(digitalPinToInterrupt(_encoderA), encoderISR, CHANGE);
}

void Motor::setSpeed(int speed)
{
    // _targetSpeed = speed;
    analogWrite(_enA,speed);
}

void Motor::forward()
{
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
}

void Motor::backward()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    _currentOutput = -_currentOutput;
}

void Motor::stop()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
}

double Motor::calculateAngularVelocity(unsigned long currentTime)
{
    // Wheel and encoder specs

    currentTime = millis();
    // Function to calculate angular velocity from encoder position

    // Calculate the difference in encoder position since the last check
    long encoderDelta = _encoderCount - _lastPosition;

    // Calculate time difference in seconds
    double timeDelta = (currentTime - _lastTime) / 1000.0;

    // Calculate the angular displacement (radians) based on encoder ticks
    double angularDisplacement = (encoderDelta * 2.0 * PI) / pulsesPerRevolution;

    // Calculate angular velocity (rad/s)
    double angularVelocity = angularDisplacement / timeDelta;

    // Update last position for the next calculation
    _lastPosition = _encoderCount;

    return angularVelocity;
}

long Motor::readEncoder()
{
    return _encoder.read();
}

void Motor::resetEncoder()
{
    // _encoderCount = 0;
    _encoder.write(0);
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

// void Motor::encoderISR()
// {
//     if (motorInstance != nullptr)
//     {
//         if (digitalRead(motorInstance->_encoderB) == HIGH)
//         {
//             motorInstance->_encoderCount--;
//         }
//         else
//         {
//             motorInstance->_encoderCount++;
//         }
//     }
// }

void Motor::PID()
{
    long currentTime = millis();
    double currentVelocity = calculateAngularVelocity(currentTime); // Current angular velocity

    // PID control logic
    double error = _targetSpeed - currentVelocity;
    double deltaTime = (currentTime - _lastTime) / 1000.0; // Time difference in seconds

    // Proportional term
    double P = Kp * error;

    // Integral term
    integral += error * deltaTime; // change if needed
    double I = Ki * integral;

    // Derivative term
    double derivative = (error - prevError) / deltaTime;
    double D = Kd * derivative;

    // PID output
    double output = P + I + D;

    // Control motor speed and direction
    analogWrite(_enA, output);

    // Update previous values
    prevError = error;
    _lastTime = currentTime;
}
