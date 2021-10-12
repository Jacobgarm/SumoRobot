#include <Servo.h>
#define SERVO_PIN 11

Servo servo;
int Trig = 6;
int a = 0;
bool forward = true;

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  
}

void loop() {
  //Program to turn towards thing.
  int closestDistance = 300;
  int closestAngle = 80;
  for(int angle = 2; angle < 180; angle++)
  {
    servo.write(angle);
    int currentDistance = Distance_test();
    if(currentDistance <= closestDistance)
    {
      closestAngle = angle;
      closestDistance = currentDistance;
    }
    delay(10);
    Serial.print("Closest distance = ");
    Serial.print(closestDistance);
    Serial.print(" at angle = ");
    Serial.println(closestAngle);
    servo.write(closestAngle);
  }
  
  delay(5000);
}

// PROGRAM COPIED FROM https://maker.pro/arduino/tutorial/an-ultrasonic-object-following-robot
int Distance_test() {
  pinMode(Trig, OUTPUT);
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  pinMode(Trig, INPUT);
  float Fdistance = pulseIn(Trig, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}
