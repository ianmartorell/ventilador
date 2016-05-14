#include "Arduino.h"

// Pins
int inputPin = A0;
int setPointPin = A1;
int outputPin = 9;

// Constants
int sampleTime = 100;

// Working variables
int input, output, outputPWM, setPoint;
double kp, ki, kd;
double errSum, lastErr;
unsigned long lastTime;

void setTunings(double a, double b, double c) {
  kp = a;
  ki = b * sampleTime;
  kd = c / sampleTime;
}

void setup()
{
  Serial.begin(9600);
  pinMode(inputPin, INPUT);
  pinMode(setPointPin, INPUT);
  pinMode(outputPin, OUTPUT);
  setTunings(16.75, 0, 0);
}

void loop()
{
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  if (timeChange >= sampleTime)
  {
    input = analogRead(inputPin);
    setPoint = analogRead(setPointPin);

    // P
    double error = (double)(setPoint - input);
    // I
    errSum += error;
    if (errSum > 1023) errSum = 1023;
    else if (errSum < 0) errSum = 0;
    // D
    double dErr = (error - lastErr);

    // compute output
    output = kp * error + ki * errSum + kd * dErr;
    if (output > 1023) output = 1023;
    else if (output < 0) output = 0;

    // map to PWM range
    outputPWM = map((int)(output+0.5), 0, 1023, 0, 255);

    analogWrite(outputPin, outputPWM);
    Serial.println(outputPWM);

    lastErr = error;
    lastTime = now;
  }
}
