#include <Arduino_FreeRTOS.h>

void TaskIRSensor( void *pvParameters );
void TaskPrintIR( void *pvParameters );


int L1,lir,R1,rir,P1,bir;
int Lw1 = 7;
int Lw2 = 8;
int rw1 = 6;
int rw2 = 5;
int lirTH = 100;
int rirTH = 100;
int birTH = 120;

void setup() {
  // put your setup code here, to run once:
  pinMode(Lw1, OUTPUT);
  pinMode(Lw2, OUTPUT);
  pinMode(rw1, OUTPUT);
  pinMode(rw2, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  //          Taskname      "name"      Stacksize  Priority
  xTaskCreate(TaskIRSensor, "IRSensor", 128, NULL, 1, NULL);
  xTaskCreate(TaskPrintIR, "IRSensor", 128, NULL, 2, NULL);
  Drive(0, 1);
  Drive(1, 1);
}

void TaskIRSensor(void *pvParameters){
  for(;;){
    digitalWrite(2,HIGH);    // Turning ON LED
    vTaskDelay(0.5 / portTICK_PERIOD_MS);  //wait
    L1=analogRead(A0);        //take reading from photodiode  :noise+signal
    R1=analogRead(A1);
    P1=analogRead(A2);
    digitalWrite(2,LOW);     //turn Off LED
    vTaskDelay(0.5 / portTICK_PERIOD_MS);  //wait
    lir=analogRead(A0)-L1;        //take reading from photodiode  :noise
    rir=analogRead(A1)-R1;
    bir=analogRead(A2)-P1;

    //DRIVING: (should maybe be a task for itself)
    if(lir < lirTH)
    {
      Drive(1, -1);
    }
    if(rir < rirTH)
    {
      Drive(0, -1);
    }
    else if(bir < birTH){
      Drive(0, 1);
      Drive(1, 1);
    } 
  }
}

void Drive(int side, int dir){ //side=0 is left, dir=1 is forward.
  return;
  int pin1;
  int pin2;
  if(side==0){
    pin1 = Lw1;
    pin2 = Lw2;
  }
  else
  {
    pin1 = rw1;
    pin2 = rw2;
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

void loop(){
  
}
