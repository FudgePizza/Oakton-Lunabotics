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
volatile int direction1 = 0;
volatile int direction2 = 0;
void countSteps1() { actuator1.stepCount+= 1*direction1; }
void countSteps2() { actuator2.stepCount+= 1*direction2; }

int spd = 500;
int agiPwr = 20; 

int dig = 1; 

float pulsesPerIn = 441.9;
Servo agitator;


void setup() {
  // put your setup code here, to run once:
  agitator.attach(7);
  agitator.writeMicroseconds(1500);
  Serial.begin(9600);
  actuator1.pin = 2;
  pinMode(actuator1.pin,INPUT_PULLUP); // Hall effect sensor of Actuator 1 
  actuator1.actuator.attach(12); // SM OUT Actuator 1 (Lift)

  actuator2.pin = 3;
  pinMode(actuator2.pin,INPUT_PULLUP); // Hall effect sensor of Actuator 2
  actuator2.actuator.attach(11);// SM OUT Actuator 2 (Tilt)
  
  /*actuator1.actuator.writeMicroseconds(2000);
  actuator2.actuator.writeMicroseconds(1000);
  delay(20000); */
  actuator1.actuator.writeMicroseconds(1500);
  actuator2.actuator.writeMicroseconds(1500);
  actuator1.pos = 0;
  actuator2.pos = 0;
  attachInterrupt(digitalPinToInterrupt(actuator1.pin), countSteps1, RISING);
  attachInterrupt(digitalPinToInterrupt(actuator2.pin), countSteps2, RISING);
  Serial.setTimeout(20);
}

void loop() {
  // Check if there’s data in the serial buffer
  static unsigned long lastPrint = 0;

  if (Serial.available() > 0) {
    
    String command = Serial.readStringUntil('\n');  // Read until newline
    command.trim(); // Remove spaces or line breaks
    
    if (command.equalsIgnoreCase("u1")) {
      
      direction1 = 1;
      actuator1.actuator.writeMicroseconds(1000);
    } else if (command.equalsIgnoreCase("d1")) {
      
      direction1 = -1;
      actuator1.actuator.writeMicroseconds(2000);
    } else if (command.equalsIgnoreCase("u2")) {
        if (actuator2.pos <= 8.0) {
          direction2 = 1;
          actuator2.actuator.writeMicroseconds(2000);
          agitator.writeMicroseconds(1500 + agiPwr);
      } else {
          actuator2.actuator.writeMicroseconds(1500);
          direction2 = 0;
          agitator.writeMicroseconds(1500);
      }
      
      
    } else if (command.equalsIgnoreCase("d2")) {
      direction2 = -1;
      actuator2.actuator.writeMicroseconds(1000);
      agitator.writeMicroseconds(1500+agiPwr);
    } else if (command.equalsIgnoreCase("set")) {
      noInterrupts();
      actuator1.stepCount = 0;
      actuator2.stepCount = 0;
      interrupts();

    }
     else if (command.equalsIgnoreCase("stop1") || command.equalsIgnoreCase("none")) {
      actuator1.actuator.writeMicroseconds(1500);
      agitator.writeMicroseconds(1500);
      direction1 = 0;
    }
    else if (command.equalsIgnoreCase("stop2") || command.equalsIgnoreCase("none")) {
     
      actuator2.actuator.writeMicroseconds(1500);
      agitator.writeMicroseconds(1500);
      direction2 = 0;
    }
    else if (command.equalsIgnoreCase("stop") || command.equalsIgnoreCase("none")) {
      actuator1.actuator.writeMicroseconds(1500);
      actuator2.actuator.writeMicroseconds(1500);
      agitator.writeMicroseconds(1500);
      direction1 = 0;
      direction2 = 0; 
    }
    
     else { 
      //actuator1.actuator.writeMicroseconds(1500);
      //actuator2.actuator.writeMicroseconds(1500);
      
    }
    
    
  }  
  noInterrupts();
  long steps1 = actuator1.stepCount;
  long steps2 = actuator2.stepCount;
  actuator1.pos = steps1 / pulsesPerIn;
  actuator2.pos = steps2 / pulsesPerIn;
  interrupts();

  if (millis() - lastPrint >= 50) {   // 20 Hz
    lastPrint = millis();
    Serial.print(actuator1.pos);
    Serial.print(",");
    Serial.println(actuator2.pos);
  }
}
