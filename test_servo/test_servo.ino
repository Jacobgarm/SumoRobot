#include <Servo.h>
#define SERVO_PIN 11

Servo servo;
int Trig = A1;
int Echo = A2;
int a = 0;
bool forward = true;

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop() {
  //Program to turn towards thing.
  int closestDistance = 300;
  int closestAngle = 80;
  for(int angle = 2; angle < 180; angle+=10)
  {
    servo.write(angle);
    int currentDistance = Distance_test();
    if(currentDistance <= closestDistance)
    {
      closestAngle = angle;
      closestDistance = currentDistance;
    }
    Serial.print("Current distance = ");
    Serial.print(currentDistance);
    Serial.print(" at angle = ");
    Serial.print(angle);
    Serial.print("       Closest distance = ");
    Serial.print(closestDistance);
    Serial.print(" at angle = ");
    Serial.println(closestAngle);
    delay(10);
  }
  servo.write(closestAngle);
  delay(5000);
}

// PROGRAM COPIED FROM https://maker.pro/arduino/tutorial/an-ultrasonic-object-following-robot
int Distance_test() {
  
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  
  float Fdistance = pulseIn(Echo, HIGH); 
  Serial.print("Pre: ");
  Serial.print(Fdistance);
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}
