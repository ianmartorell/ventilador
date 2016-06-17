#include <arduino.h>
#include <PID_v1.h>

int inputPin = A0;
int setPointPin = A1;
int outputPin = 9;

double setPoint, input, output;
double kp=0.8, ki=0.25, kd=0;

PID PID(&input, &output, &setPoint, kp, ki, kd, DIRECT);

void setup()
{
  PID.SetMode(AUTOMATIC);
}

void loop()
{
  input = analogRead(inputPin);
  setPoint = analogRead(setPointPin);
  PID.Compute();
  analogWrite(outputPin, output);
}
