#include <Servo.h>
#define SERVO_PIN 11

Servo servo;

//int servo = 11;
int echo = 5;
int Trig = 6;
int a = 0;
bool forward = true;

void setup() {
//  pinMode(servo, OUTPUT);  
  pinMode(Trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  
}

void loop() {
  // put your main code here, to run repeatedly:
/*  PROGRAM TO TURN FROM SIDE TO SIDE:
  servo.write(80);
  delay(1000);
  servo.write(179);
  delay(1000);
  servo.write(80);
  delay(1000);
  servo.write(2);
  delay(1000);
  *//*
  //Program to turn towards thing.
  int closestDistance = 300;
  int closestAngle = 80;
  Serial.println(closestDistance);
  for(int angle = 2; angle < 180; angle++)
  {
    servo.write(angle);
    int currentDistance = Distance_test();
    if(currentDistance < closestDistance)
    {
      closestAngle = angle;
      closestDistance = currentDistance;
    }
    delay(10);
    Serial.println(closestDistance);
    servo.write(currentDistance);
  }
  
  delay(5000);
  */
  Serial.println(Distance_test());
  delay(100);
}

// PROGRAM COPIED FROM https://maker.pro/arduino/tutorial/an-ultrasonic-object-following-robot
int Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(echo, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}
