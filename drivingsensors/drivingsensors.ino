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

#define SERVO_PIN 12
#define US_PIN 6

int lirTH = 100;
int rirTH = 100;
int birTH = 120;

void TaskIRSensor( void *pvParameters );
void TaskUSSensor( void *pvParameters );
void TaskDrive( void *pvParameters );
void TaskPrintIR( void *pvParameters );

int L1, R1, P1;
int lir, rir, bir;
int targetAngle;

Servo servo;

void setup() {
  // Set pins to modes
  pinMode(LW_PIN1, OUTPUT);
  pinMode(LW_PIN2, OUTPUT);
  pinMode(RW_PIN1, OUTPUT);
  pinMode(RW_PIN2, OUTPUT);
  pinMode(IR_CONTROL_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  Serial.begin(9600);
  //          Taskname      "name"      Stacksize  Priority
  xTaskCreate(TaskIRSensor, "IRSensor", 128, NULL, 1, NULL);
  //xTaskCreate(TaskUSSensor, "USSensor", 128, NULL, 3, NULL);
  //xTaskCreate(TaskDrive,    "Drive",    128, NULL, 3, NULL);
  xTaskCreate(TaskPrintIR,  "PrintIR",  128, NULL, 2, NULL);

  // Start by driving forward
  setWheelDir("left", 1);
  setWheelDir("right", 1);
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
    //Serial.println(bir);
  }
}

void TaskUSSensor(void * pvParameters) {
  for (;;) {
    continue;
    int closestDistance = 300;
    int closestAngle = 0;
    //Serial.println(closestDistance);
    for(int angle = 2; angle < 180; angle++)
    {
      servo.write(angle);
      int currentDistance = getDistance();
      if(currentDistance < closestDistance)
      {
        closestAngle = angle;
        closestDistance = currentDistance;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
      Serial.println(closestDistance);
    }
    servo.write(closestAngle);
    targetAngle = closestAngle;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskDrive(void *pvParameters){
  for (;;) {
    continue;
    // If IR sensors see black, drive away
    if (lir < lirTH && rir < rirTH) 
    {
      setWheelDir("left", -1);
      setWheelDir("right", -1);
    }
    else if(lir < lirTH)
    {
      setWheelDir("right", -1);
    }
    else if(rir < rirTH)
    {
      setWheelDir("left", -1);
    }
    else if(bir < birTH){
      setWheelDir("left", 1);
      setWheelDir("right", 1);
    }
    else {
      return;
      // Else if no IR-sensors detect, turn towards target angle and drive forwards
      if ( targetAngle < 90) 
      {
        setWheelDir("left", 0);
        setWheelDir("right", 1);
        vTaskDelay((90 - targetAngle) / portTICK_PERIOD_MS);
        setWheelDir("left", 1);
        setWheelDir("right", 1);
      }
      else if ( targetAngle > 90) 
      {
        setWheelDir("left", 0);
        setWheelDir("right", 1);
        vTaskDelay((targetAngle - 90) / portTICK_PERIOD_MS);
        setWheelDir("left", 1);
        setWheelDir("right", 1);
      }
    }

    
    
  }
}

void setWheelDir(String side, int dir){ //dir=1 is forward, 0 is stop, -1 is reverse
  return;
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
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else if (dir == 0){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
  else{
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
}

void TaskPrintIR(void *pvParameters){
  for(;;){
    Serial.print("Left = ");
    Serial.print(lir); 
    Serial.print("  ");
    Serial.print("Right = ");
    Serial.print(rir);
    Serial.print("  ");
    Serial.print("Back = ");
    Serial.println(bir);
    vTaskDelay(500 / portTICK_PERIOD_MS);
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

void loop(){

}