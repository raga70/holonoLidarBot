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

void Motor::setSpeed(double output) {
    analogWrite(_enA, output);
}

void Motor::forward() {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
}

void Motor::backward() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    output = -output;
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

double Motor::calculateAngularVelocity(long _encoderCount, unsigned long currentTime) {

  long lastEncoderPosition = 0;  // Store the last encoder position
  unsigned long lastVelocityTime = 0;  // Time when we last checked velocity
  // Wheel and encoder specs
  const double wheelDiameter = 0.080;  // Wheel diameter in meters (80mm)
  const int pulsesPerRevolution = 1440;
  
  currentTime = millis();
  // Function to calculate angular velocity from encoder position

  // Calculate the difference in encoder position since the last check
  long encoderDelta = _encoderCount - lastEncoderPosition;

  // Calculate time difference in seconds
  double timeDelta = (currentTime - lastVelocityTime) / 1000.0;

  // Calculate the angular displacement (radians) based on encoder ticks
  double angularDisplacement = (encoderDelta * 2.0 * PI) / pulsesPerRevolution;

  // Calculate angular velocity (rad/s)
  double angularVelocity = angularDisplacement / timeDelta;

  // Update last position for the next calculation
  lastEncoderPosition = _encoderCount;
    
  return angularVelocity,angularDisplacement;
 }


void Motor::encoderISR() {
    if (motorInstance != nullptr) {
        motorInstance->_encoderCount++;
    }
}

void Motor::PID (double speed){
 
    long currentTime = millis();
    unsigned long lastTime = 0;    
    double currentVelocity = calculateAngularVelocity(_encoderCount, currentTime); // Current angular velocity
    double prevError = 0;
    double integral = 0;

    // PID constants
    double Kp = 3.0, Ki = 5.0, Kd = 1.0;  // Motor 1

    // PID control logic
    double targetVelocity = speed;
    double error = targetVelocity - currentVelocity;
    double deltaTime = (currentTime - lastTime) / 1000.0;  // Time difference in seconds
    
    // Proportional term
    double P = Kp * error;
    
    // Integral term
    integral += error * deltaTime; //change if needed
    double I = Ki * integral;
    
    // Derivative term
    double derivative = (error - prevError) / deltaTime;
    double D = Kd * derivative;
    
    // PID output
    double output = P + I + D;
    
    // Control motor speed and direction
    setSpeed(output);
    
    // Update previous values
    prevError = error;
    lastTime = currentTime;
    }
