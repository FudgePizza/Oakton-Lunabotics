#include <Servo.h>
#include <math.h>

class Actuator { 
  public:    // 
    Servo actuator;
    float pos;
    int pin;
    volatile long stepCount = 0;
};

Actuator actuator1;
Actuator actuator2;
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

void countSteps1() { actuator1.stepCount++; }
void countSteps2() { actuator2.stepCount++; }

int spd = 500;

int dig = 1; 

float pulsesPerIn = 441.9;

void actuatorSet(Actuator &act, float pos){ 
  float dis = pos - act.pos;
  int direction = (dis > 0) ? 1 : -1;
  int aSpd = (constrain(spd, -500,500))* (direction);
  aSpd += 1500;
  act.stepCount = 0;
  long pulsesNeeded = (long)(fabs(dis)*pulsesPerIn); 
  attachInterrupt(digitalPinToInterrupt(act.pin),
                (&act == &actuator1) ? countSteps1 : countSteps2,
                RISING);                       
  unsigned long start = millis();
  act.actuator.writeMicroseconds(aSpd);
  while(true){
    noInterrupts();
    long steps = act.stepCount;
    interrupts();

    if (steps >= pulsesNeeded) break;
    if (millis() - start > 15000) break;
    
    delayMicroseconds(100);
    
  }
  act.pos += direction* ((float)act.stepCount/pulsesPerIn);
  act.actuator.writeMicroseconds(1500 + direction*200);
  delay(40);
  act.actuator.writeMicroseconds(1500);
  detachInterrupt(digitalPinToInterrupt(act.pin));
} 

void actuatorDig(Actuator &act, int mSpd, int aSpd){ 
  
  int direction = (aSpd > 0) ? 1 : -1;
  aSpd = (constrain(aSpd, -500,500));
  aSpd += 1500;
  act.stepCount = 0;
   
  attachInterrupt(digitalPinToInterrupt(act.pin),
                (&act == &actuator1) ? countSteps1 : countSteps2,
                RISING);
  unsigned long start = millis();
  act.actuator.writeMicroseconds(aSpd);
  Motor_Move(mSpd,mSpd);
  while(true){
    noInterrupts();
    long steps = act.stepCount;
    interrupts();

    if (millis() - start > 15000) break;
    
    delayMicroseconds(100);
    
  }
  act.pos += direction* ((float)act.stepCount/pulsesPerIn);
  act.actuator.writeMicroseconds(1500 + direction*200);
  delay(40);
  act.actuator.writeMicroseconds(1500);
  Motor_Move(0,0);
  detachInterrupt(digitalPinToInterrupt(act.pin));
} 



void Motor_Move(int leftSpd, int rightSpd){
  rightSpd = constrain(rightSpd, -500,500);
  leftSpd = constrain(leftSpd, -500,500);
  rightSpd += 1500;
  leftSpd += 1500;
  Motor1.writeMicroseconds(leftSpd);
  Motor2.writeMicroseconds(leftSpd);
  Motor3.writeMicroseconds(rightSpd);
  Motor4.writeMicroseconds(rightSpd);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  actuator1.pin = 2;
  pinMode(actuator1.pin,INPUT); // Hall effect sensor of Actuator 1
  actuator1.actuator.attach(12);

  actuator2.pin = 3;
  pinMode(actuator2.pin,INPUT); // Hall effect sensor of Actuator 2
  actuator2.actuator.attach(11);
  
  Motor1.attach(10);
  Motor2.attach(9);
  Motor3.attach(8);
  Motor4.attach(7);

  Motor1.writeMicroseconds(1500);
  Motor2.writeMicroseconds(1500);
  Motor3.writeMicroseconds(1500);
  Motor4.writeMicroseconds(1500);
  
  actuator1.actuator.writeMicroseconds(2000);
  actuator2.actuator.writeMicroseconds(2000);
  delay(10000);
  actuator1.actuator.writeMicroseconds(1500);
  actuator2.actuator.writeMicroseconds(1500);
  actuator1.pos = 0;
  actuator2.pos = 0;

  
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
        actuatorDig(actuator1, 250, 500);
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
