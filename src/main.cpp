#include <arduino.h>
#include <PID_v1.h>

// Pins
int inputPin = A0;
int setPointPin = A1;
int outputPin = 9;

// Working variables
double input, output, outputPWM, setPoint;

double kp = 16.75;
double ki = 0;
double kd = 0;

PID PID(&input, &output, &setPoint, kp, ki, kd, DIRECT);

void setup()
{
  input = analogRead(A0);
  setPoint = analogRead(A1);
  PID.SetOutputLimits(0, 1023);
  PID.SetMode(AUTOMATIC);
}

void loop()
{
  input = analogRead(A0);
  setPoint = analogRead(A1);
  PID.Compute();
  outputPWM = map((int)(output+0.5), 0, 1023, 0, 255);
  analogWrite(outputPin,outputPWM);
}
