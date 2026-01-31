/**********************************************************************
  Product     : Freenove 4WD Car for Raspberry Pi Pico (W)
  Auther      : www.freenove.com
  Modification: 2023/07/10
**********************************************************************/
#include <Servo.h>

void Motor_Move(leftSpd, rightSpd){
  constrain(rightSpd, -500,500);
  constrain(lefftSpd, -500,500);
  rightSpd += 1500;
  leftSpd += 1500;
  Motor1.writeMicroseconds(leftSpd);
  Motor2.writeMicroseconds(rightSpd);
  
}

Servo Motor1;
Servo Motor2;
//Servo Motor3;
//Servo Motor4; 

void setup() {
  Serial.begin(9600);
  
  Motor1.attach(8);
  Motor2.attach(9); 
  //Motor3.attach(10); 
  //Motor4.attach(11); //Motor Initializes
}

void loop() {
  // Check if there’s data in the serial buffer
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read until newline
    command.trim(); // Remove spaces or line breaks

    if (command.equalsIgnoreCase("forward")) {
      Serial.println("Moving forward...");
      Motor_Move(250, 250);  // Both sides forward

    } else if (command.equalsIgnoreCase("backward")) {
      Serial.println("Moving backward...");
      Motor_Move(-250, -250);  // Both sides reverse

    } else if (command.equalsIgnoreCase("left")) {
      Serial.println("Turning left...");
      Motor_Move(-200, 200);  // Left motors reverse, right motors forward

    } else if (command.equalsIgnoreCase("right")) {
      Serial.println("Turning right...");
      Motor_Move(200, -200);  // Left forward, right reverse

    } else if (command.equalsIgnoreCase("stop")) {
      Serial.println("Stopping...");
      Motor_Move(0, 0);  // Stop all motors

    } else {
      Serial.println("Unknown command. Try: forward / backward / left / right / stop");
      Motor_Move(0,0);
    }
  }
}
