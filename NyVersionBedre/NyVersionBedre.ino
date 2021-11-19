#include <Servo.h>
#include <Arduino_FreeRTOS.h>

#define LW_PIN1 7
#define LW_PIN2 8
#define RW_PIN1 6
#define RW_PIN2 5

#define IR_CONTROL_PIN 2
#define LIR_PIN A0
#define RIR_PIN A1
#define BIR_PIN A2

#define SERVO_PIN 9
#define US_PIN 10

int lirTH = 100;
int rirTH = 100;
int birTH = 130;

void TaskIRSensor( void *pvParameters );
void TaskUSSensor( void *pvParameters );
void TaskDrive( void *pvParameters );
void TaskPrintInfo( void *pvParameters );

int L1, R1, P1;
int lir, rir, bir;
bool forward = true;
int enemyAngle = 0;

Servo servo;
int servoAngle = 0;
int servoStep = 3;
int pastDistance = 1000;

void setup() {
  // Set pins to modes
  pinMode(LW_PIN1, OUTPUT);
  pinMode(LW_PIN2, OUTPUT);
  pinMode(RW_PIN1, OUTPUT);
  pinMode(RW_PIN2, OUTPUT);
  pinMode(IR_CONTROL_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  servo.write(servoAngle);
  Serial.begin(9600);
  //          Taskname      "name"      Stacksize  Priority
  xTaskCreate(TaskIRSensor, "IRSensor", 128, NULL, 1, NULL);
  xTaskCreate(TaskUSSensor, "USSensor", 128, NULL, 1, NULL);
  xTaskCreate(TaskDrive,    "Drive",    128, NULL, 1, NULL);
  xTaskCreate(TaskPrintInfo,  "PrintInfo",  128, NULL, 1, NULL);

  // Start by driving forward
  //setWheelDir("left", 1);
  //setWheelDir("right", 1);
}

void TaskIRSensor(void *pvParameters){
  for(;;){
    digitalWrite(IR_CONTROL_PIN,HIGH);    // Turning ON LED
    vTaskDelay(0.5 / portTICK_PERIOD_MS);  //wait
    L1 = analogRead(LIR_PIN);        //take reading from photodiode  :noise+signal
    R1 = analogRead(RIR_PIN);
    P1 = analogRead(BIR_PIN);
    digitalWrite(IR_CONTROL_PIN,LOW);     //turn Off LED
    vTaskDelay(0.5 / portTICK_PERIOD_MS);  //wait
    lir = analogRead(LIR_PIN)-L1;        //take reading from photodiode  :noise
    rir = analogRead(RIR_PIN)-R1;
    bir = analogRead(BIR_PIN)-P1;
    if (lir == 0 && rir == 0 && bir == 0) {}
    else if (bir < birTH) {
      forward = true;
    } 
    else if (rir < rirTH || lir < lirTH) {
      forward = false;
    }
    vTaskDelay(8 / portTICK_PERIOD_MS);
  }
}

void TaskUSSensor(void * pvParameters) {
  for (;;) {
    servoAngle += servoStep;
    servo.write(servoAngle+90);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (servoAngle > 80 || servoAngle < -80){
      servoStep *= -1;
      pastDistance = 1000;
    }
    else {
      int d = getDistance();
      
      if (d > pastDistance + 10 && pastDistance < 30) {
        enemyAngle = servoAngle - 3 * servoStep;
        servoStep *= -1;
      }
      pastDistance = d;
    }
  }
}

int getDistance() {
  pinMode(US_PIN, OUTPUT);
  digitalWrite(US_PIN, LOW);   
  delayMicroseconds(2);
  digitalWrite(US_PIN, HIGH);  
  delayMicroseconds(20);
  digitalWrite(US_PIN, LOW);
  
  pinMode(US_PIN, INPUT);
  float Fdistance = pulseIn(US_PIN, HIGH);  
  Fdistance = Fdistance / 58;       
  return (int)Fdistance;
}

void TaskDrive(void *pvParameters){
  for (;;) {
    vTaskDelay(8 / portTICK_PERIOD_MS);
    if (forward) {
      if (enemyAngle < -45) {
        setWheelDir("left", 0);
        setWheelDir("right", 1);
      } 
      else if  (enemyAngle > 45) {
        setWheelDir("left", 1);
        setWheelDir("right", 0);
      }
      else {
        setWheelDir("left", 1);
        setWheelDir("right", 1);
      }
    }
    else {
      setWheelDir("left", -1);
      setWheelDir("right", -1);
    }
  }
}

void setWheelDir(String side, int dir){ //dir=1 is forward, 0 is stop, -1 is reverse
  int pin1;
  int pin2;
  if(side == "left"){
    pin1 = LW_PIN1;
    pin2 = LW_PIN2;
  }
  else if (side == "right")
  {
    pin1 = RW_PIN1;
    pin2 = RW_PIN2;
  }
  if(dir == 1){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  else if (dir == 0){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
  else if (dir == -1){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
}

void TaskPrintInfo(void *pvParameters){
  for(;;){
    Serial.print("IR - Left = ");
    Serial.print(lir); 
    Serial.print("  ");
    Serial.print("Right = ");
    Serial.print(rir);
    Serial.print("  ");
    Serial.print("Back = ");
    Serial.println(bir);
    
    Serial.print("Servo Angle: ");
    Serial.println(servoAngle);
    Serial.print("Enemy Angle: ");
    Serial.println(enemyAngle);
    Serial.println();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void loop(){

}
