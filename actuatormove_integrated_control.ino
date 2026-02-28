#include <Servo.h>
#include <math.h>

class Actuator { // The class keyword defines a class named Car
  public:    // Access specifier - members are accessible from outside the class
    Servo actuator;
    float pos;
    int pin;
};

Actuator actuator1;
//Actuator actuator2;
Servo Motor1;
//Servo Motor2;
//Servo Motor3;
//Servo Motor4;


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

void Motor_Move(int leftSpd, int rightSpd){
  rightSpd = constrain(rightSpd, -500,500);
  leftSpd = constrain(leftSpd, -500,500);
  rightSpd += 1500;
  leftSpd += 1500;
  Motor1.writeMicroseconds(leftSpd);
  //Motor2.writeMicroseconds(rightSpd);
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  actuator1.pin = 2;
  pinMode(actuator1.pin,INPUT); // Hall effect sensor of Actuator 1
  actuator1.actuator.attach(12);
  actuatorMove(actuator1,-12);
  actuator1.pos = 0;

  actuator2.pin = 3;
  pinMode(actuator2.pin,INPUT); // Hall effect sensor of Actuator 2
  actuator2.actuator.attach(11);
  actuatorMove(actuator2,-12);
  actuator2.pos = 0;

  Motor1.attach(10);
  //Motor2.attach(9);
  //Motor3.attach(8);
  //Motor4.attach(7);
}

void loop() {
  // Check if there’s data in the serial buffer
  if (Serial.available() > 0) {
    
    String command = Serial.readStringUntil('\n');  // Read until newline
    command.trim(); // Remove spaces or line breaks

    if (command.equalsIgnoreCase("forward")) {
      //Serial.println("Moving forward...");
      Motor_Move(spd, spd);  // Both sides forward

    } else if (command.equalsIgnoreCase("backward")) {
      //Serial.println("Moving backward...");
      Motor_Move(-1*spd, -1*spd);  // Both sides reverse

    } else if (command.equalsIgnoreCase("left")) {
      //Serial.println("Turning left...");
      Motor_Move(-1*spd, spd);  // Left motors reverse, right motors forward

    } else if (command.equalsIgnoreCase("right")) {
      //Serial.println("Turning right...");
      Motor_Move(spd, -1*spd);  // Left forward, right reverse

    } else if (command.equalsIgnoreCase("dig")) {
      //Serial.println("Stopping...");
      if(dig == 1){
        dig = 2;
        // move actuators as needed
      } else if(dig == 2){
        dig = 3; 
        //move actuators as needed
      } else{
        dig = 1; 
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
