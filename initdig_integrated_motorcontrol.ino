#include <Servo.h>


Servo Motor1;

//Servo Motor2;
//Servo Motor3;
//Servo Motor4;

//Servo Actuator1;
//Servo Actuator2;
//Servo Actuator3;
int spd = 500;
float spdOfActuator = 0.05; //m/s
int digCommand = 1; 


void Actuator_Move(Servo act, int dis, int dc){ //dc is dig constant time for when bucket is slowed due to digging
  int aSpd = constrain(spd, -500,500);
  aSpd += 1500;
  float time = dis/spdOfActuator; 
  act.writeMicroseconds(aSpd + dc);
  delay(time);
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
  Serial.begin(9600);
  
  Motor1.attach(8);
  //Motor2.attach(9); 
  //Motor3.attach(10); 
  //Motor4.attach(11); //Motor Initializes
  //Actuator1.attach(12);
  //Actuator2.attach(13);

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
        dig == 1; 
        //move actuators as needed
      }

    }
     else if (command.equalsIgnoreCase("stop") or command.equalsIgnoreCase("none")) {
      //Serial.println("Stopping...");
      Motor_Move(0, 0);  // Stop all motors

    }
    
     else {
      //Serial.println("Unknown command. Try: forward / backward / left / right / stop");
      Motor_Move(0,0);
    }
    
  }
}
