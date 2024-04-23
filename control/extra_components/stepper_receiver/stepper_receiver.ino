#define dirPin 5
#define stepPin 2
#define motorInterfaceType 1
#include "AccelStepper.h"

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
    pinMode(A2, INPUT);
    Serial.begin(9600);
    
    stepper.setMaxSpeed(10000);
    stepper.setAcceleration(50000);
}

void loop() {
    int speed = 1000;
    int basespeed = 300;
    int f_factor = 1;
    if (Serial.available()) {

        //read icoming text and transform it
        char buffer[32]; // Buffer to hold incoming data
        Serial.setTimeout(10);
        int length = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[length] = '\0'; // Null-terminate the string
        stepper.runSpeed();
        String command = String(buffer);
        stepper.runSpeed();

        if(command.startsWith("sp")) {
            stepper.setMaxSpeed(10000);
            speed = command.substring(2).toInt();
            stepper.setSpeed(speed);
            // Step the motor with a constant speed as set by setSpeed():
            Serial.println("Speed set to " + String(speed));


        }else if(command.startsWith("p")){

          // Extract position and speed from command string
          Serial.println(command);
          // Find the position of 'p' and 's'
          int pIndex = command.indexOf('p');
          int sIndex = command.indexOf('s');
          int semicolonIndex = command.indexOf(';');

          // Extract the position and speed from the command string
          String positionString = command.substring(pIndex + 1, semicolonIndex);
          String speedString = command.substring(sIndex + 1);

          // Convert the position and speed to integers
          int position = positionString.toInt();
          int speed = speedString.toInt();



            Serial.println("position: " + String(position) + "; speed: " + String(speed) );

            // calculate the number of steps required to move the specified distance
            float distanceInMM = position;
            float stepsPerMM = 100.0; // Adjust this value based on your stepper motor specifications
            int steps = distanceInMM * stepsPerMM;

            //set the speed in steps per second
            stepper.moveTo(steps);
            stepper.setSpeed(10);

            unsigned long startTime = millis();
            unsigned long timeout = 10000; // 3 seconds

            while(stepper.distanceToGo() != 0 ){
                stepper.run();
                if(millis()-startTime > timeout){
                Serial.println("timeout");
                break;
                }
            }
            stepper.setSpeed(0);
            Serial.println("ack");
        
        
        }else if(command.startsWith("init")){

            Serial.println("Initializing stepper");
            stepper.setMaxSpeed(10000);
            stepper.setSpeed(3000);
            unsigned long startTime = millis();
            unsigned long duration = 1000; // 3 seconds
            while (millis() - startTime < duration) {
                stepper.runSpeed();
            }
            stepper.setSpeed(-3000);
            startTime = millis();
            duration = 1000; // 3 seconds
            while (millis() - startTime < duration) {
                stepper.runSpeed();
            }
            stepper.setSpeed(0);
            Serial.println("Stepper initialized");

        }else if(command.startsWith("reset")){

          stepper.setCurrentPosition(0);
          Serial.println("position reset");
        
        
        }else {

            Serial.println("Invalid command");
        }
        stepper.runSpeed();
    }

    stepper.runSpeed();
        

    
}