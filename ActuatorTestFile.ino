#include <Servo.h>



Servo Actuator1;

int spd = 500;
float spdOfActuator = 0.05; //in/s
int dig = 1; 
volatile long stepCount = 0;
float pulsesPerIn = 441.96;



void actuatorMove(Servo &act, int sensorPin, float dis){ 
  int direction = (dis > 0) ? 1 : -1;
  int aSpd = (constrain(spd, -500,500))* (direction);
  aSpd += 1500;
  stepCount = 0;
  long pulsesNeeded = (long)(abs(dis)*pulsesPerIn); 
  attachInterrupt(digitalPinToInterrupt(sensorPin), countSteps, RISING);
  unsigned long start = millis();
  act.writeMicroseconds(aSpd);
  while(true){
    noInterrupts();
    long steps = stepCount;
    interrupts();

    if (steps >= pulsesNeeded) break;
    if (millis() - start > 5000) break;
    
    delayMicroseconds(100);
    
  }
  act.writeMicroseconds(1500);
  detachInterrupt(digitalPinToInterrupt(sensorPin));
} 

void countSteps() {
  stepCount++;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2,INPUT); // Hall effect sensor of Actuator 1
  Actuator1.attach(12);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    
    String command = Serial.readStringUntil('\n');  // Read until newline
    command.trim(); // Remove spaces or line breaks

    if (command.equalsIgnoreCase("dig")) {
      //Serial.println("Stopping...");
      if(dig == 1){
        dig = 2;
        actuatorMove(Actuator1, 2, 3.0);
        // move actuators as needed
      } else if(dig == 2){
        dig = 1; 
        actuatorMove(Actuator1, 2, -3.0);
        //move actuators as needed
      }

    }
     else if (command.equalsIgnoreCase("stop") || command.equalsIgnoreCase("none")) {
      //Serial.println("Stopping...");
      Motor_Move(0, 0);  // Stop all motors

    }
    
     else {
      //Serial.println("Unknown command. Try: forward / backward / left / right / stop");
      Motor_Move(0,0);
    }
    
  }
}
