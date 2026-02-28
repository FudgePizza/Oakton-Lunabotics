#include <Servo.h>
#include <math.h>

class Actuator { // The class keyword defines a class named Car
  public:    // Access specifier - members are accessible from outside the class
    Servo actuator;
    float pos;
    int pin;
};

Actuator actuator1;


int spd = 500;
float spdOfActuator = 0.05; //in/s
int dig = 1; 
volatile long stepCount = 0;
float pulsesPerIn = 441.9;



void actuatorMove(Actuator &act, float pos){ 
  float dis = pos - act.pos;
  int direction = (dis > 0) ? 1 : -1;
  int aSpd = (constrain(spd, -500,500))* (direction);
  aSpd += 1500;
  stepCount = 0;
  long pulsesNeeded = (long)(fabs(dis)*pulsesPerIn); 
  attachInterrupt(digitalPinToInterrupt(act.pin), countSteps, RISING);
  unsigned long start = millis();
  act.actuator.writeMicroseconds(aSpd);
  while(true){
    noInterrupts();
    long steps = stepCount;
    interrupts();

    if (steps >= pulsesNeeded) break;
    if (millis() - start > 15000) break;
    
    delayMicroseconds(100);
    
  }
  act.pos += direction* (stepCount/pulsesPerIn);
  act.actuator.writeMicroseconds(1500);
  detachInterrupt(digitalPinToInterrupt(act.pin));
} 

void countSteps() {
  stepCount++;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  actuator1.pos = 0.0;
  actuator1.pin = 2;
  pinMode(actuator1.pin,INPUT); // Hall effect sensor of Actuator 1
  actuator1.actuator.attach(12);
  actuatorMove(actuator1,-12);
  actuator1.pos = 0;
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
        actuatorMove(actuator1, 4.0);
        // move actuators as needed
      } else if(dig == 2){
        dig = 1; 
        actuatorMove(actuator1, 2.0);
        //move actuators as needed
      }

    }
     else if (command.equalsIgnoreCase("stop") || command.equalsIgnoreCase("none")) {
      //Serial.println("Stopping...");
        // Stop all motors

    }
    
     else {
      //Serial.println("Unknown command. Try: forward / backward / left / right / stop");
      
    }
    
  }
}
    
  
