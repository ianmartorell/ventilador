#include "Arduino.h"

// Pins
int inputPin = A0;
int setPointPin = A1;
int outputPin = 9;

// Constants
int sampleTime = 0.1; // in seconds

// Working variables
volatile int input, output, outputPWM, setPoint;
volatile double errSum, lastInput;
double kp, ki, kd;

void setTunings(double a, double b, double c) {
  kp = a;
  ki = b * sampleTime;
  kd = c / sampleTime;
}

ISR(TIMER1_COMPA_vect)
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
  double dInput = (input - lastInput);

  // compute output
  output = kp * error + ki * errSum - kd * dInput;
  if (output > 1023) output = 1023;
  else if (output < 0) output = 0;

  // map to PWM range
  outputPWM = map((int)(output+0.5), 0, 1023, 0, 255);

  analogWrite(outputPin, outputPWM);
  Serial.println(outputPWM);

  lastInput = input;
}

void setup()
{
  Serial.begin(9600);
  pinMode(inputPin, INPUT);
  pinMode(setPointPin, INPUT);
  pinMode(outputPin, OUTPUT);

  setTunings(16.75, 0, 0);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  // set compare match register to desired timer count
  OCR1A = (int) 16000000/(1024*(1/sampleTime))-1;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void loop() {}
