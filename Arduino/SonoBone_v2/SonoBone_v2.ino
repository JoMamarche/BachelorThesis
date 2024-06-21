#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <SimpleDHT.h>
#include <Servo.h>

Servo myservo;

//pt100 module sensor setup
Adafruit_MAX31865 thermo = Adafruit_MAX31865(53);
#define RREF      430.0
#define RNOMINAL  100.0

//digital temperature sensor setup
SimpleDHT22 dht22(35);
float temp;
float hum;

//UV Light setup
unsigned long UVStartTime = 0;
const unsigned long UVDuration = 10000;

//PID setup
double kp_HP_T=20, ki_HP_T=3, kd_HP_T=1;           
double kp_Air_T=20, ki_Air_T=3, kd_Air_T=1;
double input_HP_T, output_HP_T, setpoint_HP_T;
double input_Air_T, output_Air_T, setpoint_Air_T;
int winsize_HP_T = 5000;
int winsize_Air_T = 10000;
unsigned long winstart_HP_T;
unsigned long winstart_Air_T;
PID pid_HP_T(&input_HP_T, &output_HP_T, &setpoint_HP_T, kp_HP_T, ki_HP_T, kd_HP_T, DIRECT);
PID pid_Air_T(&input_Air_T, &output_Air_T, &setpoint_Air_T, kp_Air_T, ki_Air_T, kd_Air_T, DIRECT);

//String for serial readig
String command;
String readString;

//pin setup for relays
#define heaters 22
#define intake_fan 24
#define exhaust_fan 26
#define big_fan 28
#define HP 30
#define UV 32
#define valve1 27
#define valve2 29
#define valve3 31
#define valve4 33

//pin setup for button
#define button 34

int state = 0;

//Functions definition
void setupPinMode(); 
void servoReset();
void initialPinOutput();
void processCommand();
void heatPadControl();
void heatersControl();
void Running();
void printTemperatureSetup();
void cooldown();


void setup() {
  Serial.begin(115200);

  while (!Serial);
  delay(1000);

  while(Serial.available() > 0){
    Serial.read();
  }

  //Run servo to reset psu robot arm
  servoReset();

  //setting up pin for relays
  setupPinMode();

  //Assure everything is off
  initialPinOutput();

  
  thermo.begin(MAX31865_2WIRE);
  
  
  pid_HP_T.SetOutputLimits(0, winsize_HP_T);
  pid_HP_T.SetMode(AUTOMATIC);
  pid_Air_T.SetOutputLimits(0, winsize_Air_T);
  pid_Air_T.SetMode(AUTOMATIC);

  

}

void loop() {
  commandstate();
  processCommand();
 

  if(state == 0){
    initialPinOutput();
    processCommand();
    commandstate();
    Serial.println("state 0");
  }

  while(state == 1){
    processCommand();
    commandstate();
    Running();
    turnOnUV();
    if(input_HP_T >= setpoint_HP_T-5 && input_HP_T <= setpoint_HP_T+5 && input_Air_T >= setpoint_Air_T-1 && input_Air_T <= setpoint_Air_T+1){
      Serial.println("Heatpad and ambient air at right temperature");
      Serial.println("Start printing");
      state = 2;
    }
    if(digitalRead(button) == LOW){
      state = 0;
    }
    Serial.println("state 1");
  }

  while(state == 2){
    Running();
    processCommand();
    commandstate();
    Serial.println("state 2");
    if(digitalRead(button) == LOW){
      state = 0;
    }
  }

  while(state == 3){
    cooldown();
    processCommand();
    commandstate();
    if(thermo.temperature(RNOMINAL, RREF) < 40){
      state = 0;
    }
    if(digitalRead(button) == LOW){
      state = 0;
    }
    Serial.println("state 3");
  }

}

void setupPinMode(){
  pinMode(heaters, OUTPUT);
  pinMode(intake_fan, OUTPUT);
  pinMode(exhaust_fan, OUTPUT);
  pinMode(big_fan, OUTPUT);
  pinMode(HP, OUTPUT);
  pinMode(UV, OUTPUT);
  pinMode(valve1, OUTPUT);
  pinMode(valve2, OUTPUT);
  pinMode(valve3, OUTPUT);
  pinMode(valve4, OUTPUT);
  pinMode(button, INPUT_PULLUP);
}

void servoReset(){
  myservo.attach(9);
  myservo.write(120);
  delay(500);
  myservo.write(135);
  delay(1000);
  myservo.write(120);
}

void initialPinOutput(){
  digitalWrite(heaters, HIGH);
  digitalWrite(intake_fan, HIGH);
  digitalWrite(exhaust_fan, HIGH);
  digitalWrite(big_fan, HIGH);
  digitalWrite(HP, HIGH);
  digitalWrite(UV, HIGH);
  digitalWrite(valve1, HIGH);
  digitalWrite(valve2, HIGH);
  digitalWrite(valve3, HIGH);
  digitalWrite(valve4, HIGH);
  setpoint_HP_T = 0;
  setpoint_Air_T = 0;
}

void commandstate(){
  if (Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println(command);
    if(command.equals("b")){
      Serial.println("Stop Print");
      state = 0;
    }
    if(command.equals("s")){
      Serial.println("Setup Print");
      state = 1;
    }
    if(command.equals("r")){
      Serial.println("Running Print");
      state = 2;
    }
    if(command.equals("f")){
      Serial.println("Finish Print");
      state = 3;
    }
  }
}

void processCommand(){
  while(Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == ',') {
      if (readString.length() >1) {
        Serial.println(readString); //prints string to serial port out
        int n = readString.toInt();  //convert readString into a number
        if(readString.indexOf('p') >0){
          Serial.print("Heatpad at ");
          Serial.print(n);
          Serial.println(" C" ); 
          setpoint_HP_T = n;
        }
        if(readString.indexOf('h') >0){
          Serial.print("Room temperature at ");
          Serial.print(n);
          Serial.println(" C" ); 
          setpoint_Air_T = n;
        }
        if(readString.indexOf('b') >0){
          Serial.println("Stop Print");
          state = 0;
        }
        if(readString.indexOf('s') >0){
          Serial.println("Setup Print");
          state = 1;
        }
        if(readString.indexOf('r') >0){
          Serial.println("Running Print");
          state = 2;
        }
        if(readString.indexOf('f') >0){
          Serial.println("Finish Print");
          state = 3;
        }
        readString=""; //clears variable for new input
      }
    }  
    else {     
      readString += c; //makes the string readString
    }
  }
}

void printTemperatureSetup(){
  processCommand();
  digitalWrite(intake_fan, LOW);
  digitalWrite(exhaust_fan, LOW);
  heatPadControl();
  heatersControl();
}

void Running(){
  digitalWrite(intake_fan, LOW);
  digitalWrite(exhaust_fan, LOW);
  heatPadControl();
  heatersControl();
}

void heatPadControl(){
      input_HP_T = thermo.temperature(RNOMINAL, RREF);
      Serial.print("s_HP_T: "); Serial.print(setpoint_HP_T); Serial.print("\tt_HP_T: ");Serial.println(input_HP_T);
      pid_HP_T.Compute();

      if ((millis() - winstart_HP_T) > winsize_HP_T) {
        winstart_HP_T += winsize_HP_T;
      }
      if (output_HP_T < (millis() - winstart_HP_T)) {
        digitalWrite(HP, HIGH);
      } else {
        digitalWrite(HP, LOW);
      }
}

void heatersControl(){      
      dht22.read2(&temp, &hum, NULL);
      input_Air_T = temp;
      Serial.print("s_Air_T: "); Serial.print(setpoint_Air_T); Serial.print("\tt_Air_T: ");Serial.println(input_Air_T);
      pid_Air_T.Compute();

      if ((millis() - winstart_Air_T) > winsize_Air_T) {
        winstart_Air_T += winsize_Air_T;
      }
      if (output_Air_T < (millis() - winstart_Air_T)) {
        digitalWrite(heaters, HIGH );
      } else {
        digitalWrite(heaters, LOW);
      }
}

void cooldown(){
  processCommand();
  digitalWrite(big_fan, LOW);
  digitalWrite(HP, HIGH);
  heatersControl();
}

void turnOnUV() {
  // Check if the light is currently off and if 5 seconds haven't passed yet
  if (digitalRead(UV) == HIGH && millis() - UVStartTime < UVDuration) {
    // Turn on the light
    digitalWrite(UV, LOW);
    // Record the start time if the light was just turned on
    if (UVStartTime == 0) {
      UVStartTime = millis();
    }
  } else {
    // Turn off the light after 10seconds
    if (millis() - UVStartTime >= UVDuration) {
      digitalWrite(UV, HIGH);
    }
  }
}
