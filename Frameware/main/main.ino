#include <Servo.h>
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
Servo servo1; // MG996R
Servo servo2; // S3003

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

void setup() {
  Serial.begin(9600);

  servo1.attach(6);   // MG996R using angle control
  servo2.attach(7, 500, 2500);                        // S3003 using microsecond control

  Serial.println("Enter angles: MG996R (0–180) S3003 (0–270), separated by space:");

    pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  pinMode(switchPin, INPUT_PULLUP);  // Enable pull-up resistor

  digitalWrite(dirPin, reverseDirection);
  delay(3000);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      int angle1 = input.substring(0, spaceIndex).toInt();
      int angle2 = input.substring(spaceIndex + 1).toInt();

      // angle1 = constrain(angle1, minAngle1, maxAngle1);
      // angle2 = constrain(angle2, minAngle2, maxAngle2);

      // MG996R
      servo1.write(angle1);

      // Map S3003 0-270° to 600-2400 µs and write
      // int pulse2 = map(angle2, minAngle2, maxAngle2, minPulse2, maxPulse2);
      servo2.write(angle2);

      Serial.print("MG996R set to: ");
      Serial.print(angle1);
      Serial.print(" | S3003 set to: ");
      Serial.println(angle2);
      // Serial.print(" (");
      // Serial.print(pulse2);
      // Serial.println(" µs)");
    } else {
      if(input == "shoot"){
      Serial.println("Shooting");
      shoot();
      }else
        Serial.println("Invalid input. Use format like: 90 180");
    }
  }
}
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
