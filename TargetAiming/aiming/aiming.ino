#include <Servo.h>


float panAngle;
float tiltAngle;

// Define pin connections
const int stepPin = 2;    // Pin connected to STEP pin of driver
const int dirPin = 3;     // Pin connected to DIR pin of driver
const int switchPin=4;

int total_steps = 0;
int flagHome = 0; 
int reverseDirection = 0;

// Define motor parameters
const int stepsPerRevolution =200;  // NEMA 17 typically has 200 steps/rev (1.8° per step)
 int motorSpeed = 1000;         // Slower speed: 1000µs
Servo panServo; // S3003
Servo tiltServo; // 996R

// MG996R Constants
const int minAngle1 = 0;
const int maxAngle1 = 180;
const int minPulse1 = 500;
const int maxPulse1 = 2500;

// S3003 Constants
const int minAngle2 = 0;
const int maxAngle2 = 270;
const int minPulse2 = 600;
const int maxPulse2 = 2400;

void shoot(){
  digitalWrite(dirPin, reverseDirection);
  flagHome = 0;
    while(flagHome==0)
  {

    rotateSteps(1);

    if (digitalRead(switchPin) == LOW) {
      // motorSpeed=1000;
      total_steps =0;
    
      // Serial.println(steps);
      // Serial.println(motorSpeed);
      Serial.println("home sweet home");

      digitalWrite(dirPin, 1-reverseDirection);
      rotateSteps(7.2f * 1000);

      flagHome=1;
    }
  }
}

void rotateSteps(int steps) {
  for(int i = 0; i < steps; i++) {
    total_steps++;
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorSpeed);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorSpeed);
    // Serial.println(total_steps);
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);

  panServo.attach(6,500, 2500);  // Attaches the servo on pin 9 to the servo object
  tiltServo.attach(7);

  panServo.write(90);

  tiltServo.write(90);

  //  panServo.write(180);
  //   delay(500);
  //   panServo.write(0);
  //   delay(500);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');  // Read until newline
    data.trim();                                 // Remove whitespace

    int commaIndex = data.indexOf(',');  // Find the comma between pan and tilt

    if (commaIndex > 0) {
      panAngle = data.substring(0, commaIndex).toFloat();    // Pan angle
      tiltAngle = data.substring(commaIndex + 1).toFloat();  // Tilt angle

      // Now you have panAngle and tiltAngle as floats
    }
    panServo.write(panAngle);

    tiltServo.write(tiltAngle);
    //delay(20000);
    // shoot();
    // Serial.println("ACK");
    //delay(30000);
  }
   
    

    

}
