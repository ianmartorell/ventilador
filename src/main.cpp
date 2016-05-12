#include "Arduino.h"

int inputPin = A0;
int setPointPin = A1;
int outputPin = 9;

unsigned long lastTime;
int input, output, outputPWM, setPoint;
double errSum, lastErr;
double kp, ki, kd;

double min = 0;
double max = 1023;

void setup()
{
  Serial.begin(9600);
  pinMode(inputPin, INPUT);
  pinMode(setPointPin, INPUT);
  pinMode(outputPin, OUTPUT);
}

void loop()
{
  input = analogRead(inputPin);
  setPoint = analogRead(setPointPin);

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  // P
  double error = (double)(setPoint - input);
  // I
  errSum += (error * timeChange);
  if (errSum > max) errSum = max;
  else if (errSum < min) errSum = min;
  // D
  double dErr = (error - lastErr) / timeChange;

  // compute output
  output = kp * error + ki * errSum + kd * dErr;
  if (output > max) output = max;
  else if (output < min) output = min;

  // map to PWM range
  outputPWM = map((int)(output+0.5), 0, 1023, 0, 255);

  analogWrite(outputPin, outputPWM);
  Serial.println(outputPWM);

  lastErr = error;
  lastTime = now;
}
