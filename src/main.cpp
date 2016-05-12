#include "Arduino.h"

int pin1=9;
int pote=A0;
int valorPote;
int pwm1;

void setup() {
  Serial.begin(9600);
  pinMode(pin1, OUTPUT);
}

void loop() {
  valorPote = analogRead(pote);
  pwm1 = map(valorPote, 0, 1023, 0, 255);
  analogWrite(pin1, pwm1);

  Serial.println(pin1);
}
