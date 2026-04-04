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
  int aSpd = -(constrain(spd, -500,500))* (direction);
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
  act.actuator.writeMicroseconds(1500 - direction*200);
  delay(40);
  act.actuator.writeMicroseconds(1500);
  detachInterrupt(digitalPinToInterrupt(act.pin));
} 

void actuatorDig(Actuator &act, int aSpd){ 
  
  int direction = (aSpd > 0) ? 1 : -1;
  aSpd = -(constrain(aSpd, -500,500));
  aSpd += 1500;
  act.stepCount = 0;
   
  attachInterrupt(digitalPinToInterrupt(act.pin),
                (&act == &actuator1) ? countSteps1 : countSteps2,
                RISING);
  unsigned long start = millis();
  act.actuator.writeMicroseconds(aSpd);
  while(true){
    noInterrupts();
    long steps = act.stepCount;
    interrupts();

    if (millis() - start > 250) break;
    
    delayMicroseconds(100);
    
  }
  act.pos += direction* ((float)act.stepCount/pulsesPerIn);
  act.actuator.writeMicroseconds(1500 - direction*200);
  delay(40);
  act.actuator.writeMicroseconds(1500);
  Motor_Move(0,0);
  detachInterrupt(digitalPinToInterrupt(act.pin));
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
  
  actuator1.actuator.writeMicroseconds(1000);
  actuator2.actuator.writeMicroseconds(1000);
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

    if (command.equalsIgnoreCase("u1")) {
      //Serial.println("Moving forward...");
      actuatorDig(actuator1, 500);

    } else if (command.equalsIgnoreCase("d1")) {
      //Serial.println("Moving backward...");
      actuatorDig(actuator1, -500);

    } else if (command.equalsIgnoreCase("u2")) {
      //Serial.println("Turning left...");
      actuatorDig(actuator1, 500);
    } else if (command.equalsIgnoreCase("d2")) {
      //Serial.pri  ntln("Turning right...");
      actuatorDig(actuator1, -500);
    } else if (command.equalsIgnoreCase("dig")) {
      //Serial.println("Stopping...");

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
