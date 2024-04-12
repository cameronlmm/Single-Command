#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

#define CLK 36  // CLK for Encoder 1
#define DT 39   // DT for Encoder 1
#define CLK2 35 // CLK for Encoder 2
#define DT2 34  // DT for Encoder 2

ESP32Encoder encoder;   // Create rotary encoder object encoder (1st encoder)
ESP32Encoder encoder2;  // Create rotary encoder object encoder2 (2nd encoder)

Servo myservo;            // Create servo object myservo
int servoPin = 13;        // Servo pin on downstairs ESP32
int pos = 0;              // Initialise servo position

// L298N Motor Driver pins
const int motorPin1 = 26;  // IN_A pin on downstairs ESP32 (according to schematic); Direction for Motor 1
const int motorPin2 = 27;  // IN_B pin on downstairs ESP32 (according to schematic); Direction for Motor 1
const int motorPin3 = 14;  // IN_C pin on downstairs ESP32 (according to schematic); Direction for Motor 2
const int motorPin4 = 12;  // IN_D pin on downstairs ESP32 (according to schematic); Direction for Motor 2
const int enablePinA = 33; // EN_A pin on downstairs ESP32 (according to schematic); PWM inputs for speed control of Motor 1
const int enablePinB = 25; // EN_B pin on downstairs ESP32 (according to schematic); PWM inputs for speed control of Motor 2

// Setting PWM properties for motors
const int freq = 2000;
const int pwmChannela = 0;  // PWM channel allocation for speed control of Motor 1
const int pwmChannelb = 1;  // PWM channel allocation for speed control of Motor 2
const int resolution = 8;
int dutyCycle = 140;

char Key;

const float wheelCircumference = 22.0; // EEEBot rear wheel circumference in cm
const int encoderCPR = 47;             // Counts per revolution; determined empirically (rather than from datasheet)
const float pulsePerCM = wheelCircumference / encoderCPR; // Pulse per centimeter; multiply by counts to get distance in cm

void setup() {
  Wire.begin(8);                // I2C slave address
  Wire.onReceive(receiveData);
  Serial.begin(9600);

  // Setting up motor control pins as outputs to control direction of EEEbot (motorPin1-4) and speed (enablePin1-2)
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  // Setting up PWM channels for motor speed control
  ledcSetup(pwmChannela, freq, resolution);
  ledcSetup(pwmChannelb, freq, resolution);
  
  // Attaching enable pins to PWM channels for motor speed control
  ledcAttachPin(enablePinA, pwmChannela); // pwmChannela will define motor speed for motor 1
  ledcAttachPin(enablePinB, pwmChannelb); // pwmChannelb will define motor speed for motor 2

  ESP32PWM::allocateTimer(2);    // Allocate timer for PWM control of servo (timers 0 and 1 of ESP32 already occupied by pwmChannela and pwmChannelb, respectively)
  myservo.setPeriodHertz(50);    // Standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // Attach servo (myservo) to control pin (13) according to schematic}

  encoder.attachHalfQuad(DT, CLK);    // Attach encoder 1 to DT and CLK pins on ESP32
  encoder2.attachHalfQuad(DT2, CLK2); // Attach encoder 2 to DT and CLK pins on ESP32
  encoder.setCount(0);  // Initialise encoder 1 count to 0
  encoder2.setCount(0); // Initialise encoder 2 count to 0
}

void loop() {
        //Serial.print("Command Received: ");
        //Serial.println(Key);
        Serial.println();

        switch (Key) 
        {
            case '0': Straight(); break;
            case '1': Right(); break; //goForward(); motors(dutyCycle, dutyCycle); delay(2000); Stop(); break; // If command 'F' received from upstairs ESP32, execute goForward() function to go forward
            case '2': Left(); break; //Reverse(); motors(dutyCycle, dutyCycle); delay(2000); Stop(); break;   // If command 'B' received from upstairs ESP32, execute Reverse() function to go backwards
            case '3': Serial.println("Number 3 received"); break; //Left(); motors(dutyCycle, dutyCycle); break;      // If command 'L' received from upstairs ESP32, execute Left() function to set steering left then gradually straighten
            case '4': Serial.println("Number 4 received"); break; //Right(); motors(dutyCycle, dutyCycle); break;     // If command 'R' received from upstairs ESP32, execute Right() function to turn steering fully right
            case '5': Serial.println("Number 5 received"); break; //Stop(); motors(dutyCycle, dutyCycle); break;      // If cammond 'S' received from upstairs ESP32, execute Stop() function to stop motors
            case '6': Serial.println("Number 6 received"); break; //Straight(); motors(dutyCycle, dutyCycle); break;  // If command 'N' received from upstairs ESP32, execute Straight() function to straighten steering
            case '7': Serial.println("Number 7 received"); break; //encoderReset(); motors(dutyCycle, dutyCycle); break;  // If command 'E' received from upstairs ESP32, execute encoderReset() function to reset encoders to 0
            case '8': Serial.println("Number 8 received"); break; // turn180Degrees(); motors(dutyCycle, dutyCycle); break;
            case '9': Serial.println("Number 9 received"); break; //turn90Degrees(); motors(dutyCycle, dutyCycle); break;
            case '*': Serial.println("Symbol * received"); break;
            case '#': Serial.println("Symbol # received"); break;
        }

}

void receiveData(int howMany) {
  if (Wire.available()) { // check for incoming data
    Key = Wire.read();    // receive byte as a character
    //Serial.print("Received: "); Serial.println(Key);
  }
}

void motors(int leftSpeed, int rightSpeed) // Function to set motor speed for movement
{
  ledcWrite(pwmChannela, leftSpeed);
  ledcWrite(pwmChannelb, rightSpeed);
  Serial.println ("Motors running");
  Serial.println();
  delay(25);
}

void goForward() // Function to set motor direction to go forward
{
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW); 
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW); 
  Serial.println ("Going Forward");
  Serial.println();

}

void Reverse() // Function to set motor direction to go backward
{
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH); 
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH); 
  Serial.println ("Reversing");
  Serial.println();
}

void Stop() // Function to stop motors
{
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  Serial.print("Motors have stopped");
  Serial.println();
}

void Right() // Function to turn steering to the right
{
  myservo.write(180);
  Serial.println ("Steering to the Right");
  Serial.println();
}

void Left()
{
  myservo.write(0);
}

void Straight() // Function to set steering straight
{
  myservo.write(90);
  Serial.println ("Steering Straight");
  Serial.println();
}