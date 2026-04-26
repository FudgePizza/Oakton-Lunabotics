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
int direction1 = 0;
int direction2 = 0;
void countSteps1() { actuator1.stepCount+= 1*direction1; }
void countSteps2() { actuator2.stepCount+= 1*direction2; }

int spd = 500;

int dig = 1; 

float pulsesPerIn = 441.9;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  actuator1.pin = 2;
  pinMode(actuator1.pin,INPUT_PULLUP); // Hall effect sensor of Actuator 1 
  actuator1.actuator.attach(12); // SM OUT Actuator 1 (Lift)

  actuator2.pin = 3;
  pinMode(actuator2.pin,INPUT_PULLUP); // Hall effect sensor of Actuator 2
  actuator2.actuator.attach(11);// SM OUT Actuator 2 (Tilt)
  
  actuator1.actuator.writeMicroseconds(2000);
  actuator2.actuator.writeMicroseconds(1000);
  delay(20000);
  actuator1.actuator.writeMicroseconds(1500);
  actuator2.actuator.writeMicroseconds(1500);
  actuator1.pos = 0;
  actuator2.pos = 0;
  attachInterrupt(digitalPinToInterrupt(actuator1.pin), countSteps1, RISING);
  attachInterrupt(digitalPinToInterrupt(actuator2.pin), countSteps2, RISING);
}

void loop() {
  // Check if there’s data in the serial buffer
  if (Serial.available() > 0) {
    
    String command = Serial.readStringUntil('\n');  // Read until newline
    command.trim(); // Remove spaces or line breaks
    
    if (command.equalsIgnoreCase("u1")) {
      
      direction1 = 1;
      actuator1.actuator.writeMicroseconds(1000);
    } else if (command.equalsIgnoreCase("d1")) {
      
      direction1 = -1;
      actuator1.actuator.writeMicroseconds(2000);
    } else if (command.equalsIgnoreCase("u2")&& actuator2.pos <= 8.0) {
        direction2 = 1;
        actuator2.actuator.writeMicroseconds(2000);
      
      
    } else if (command.equalsIgnoreCase("d2")) {
      direction2 = -1;
      actuator2.actuator.writeMicroseconds(1000);
    } else if (command.equalsIgnoreCase("set")) {
      

    }
     else if (command.equalsIgnoreCase("stop") || command.equalsIgnoreCase("none")) {
      actuator1.actuator.writeMicroseconds(1500);
      actuator2.actuator.writeMicroseconds(1500);
    }
    
     else { 
      actuator1.actuator.writeMicroseconds(1500);
      actuator2.actuator.writeMicroseconds(1500);
      
    }
    
    
  }
  noInterrupts();
  long steps1 = actuator1.stepCount;
  long steps2 = actuator2.stepCount;
  actuator1.pos = steps1 / pulsesPerIn;
  actuator2.pos = steps2 / pulsesPerIn;
  Serial.print(actuator1.pos);
  Serial.print(",");
  Serial.println(actuator2.pos);
  interrupts();
}
