#include <Servo.h>

Servo panServo;  // Create servo object to control a servo
// Servo tiltServo;  // Create servo object to control a servo

 int imageWidth = 640; // width of your camera frame
 int imageHeight = 480; // width of your camera frame
float x_cameraFOV = 70.0; // in degrees
float y_cameraFOV = 53; // in degrees

int targetX = 292;
int targetY = 90;


float calculatePanAngle(int targetX, int imageWidth, float camFOV) {
  int centerX = imageWidth / 2;
  float angle = 90 - ((float)(targetX - centerX) / imageWidth) * camFOV;
  return angle;
}

float calculateTiltAngle(int targetY, int imageHeight, float camFOV) {
  int centerY = imageHeight / 2;
  float angle = 90 - ((float)(targetY - centerY) / imageHeight) * camFOV;
  return angle;
}

void setup() {
  panServo.attach(9);  // Attaches the servo on pin 9 to the servo object
  // tiltServo.attach(5);  // Attaches the servo on pin 9 to the servo object
}

void loop() {

float x_angle = calculatePanAngle(targetX, imageWidth, x_cameraFOV);
// float y_angle = calculateTiltAngle(targetY, imageHeight, y_cameraFOV);

panServo.write(x_angle);
// tiltServo.write(y_angle);

// delay(1000);

// panServo.write(0);
// delay(1000);


}

// #include <Servo.h>

// Servo myServo;  // Create a servo object

// void setup() {
//   Serial.begin(9600);
//   myServo.attach(9);  // Attach the servo to digital pin 9
 

// }

// void loop() {
//   // Nothing needed here if you're only moving once
//    myServo.write(90);  // Move the servo to 90 degrees
//   delay(1000);
//   myServo.write(0);
//     delay(1000);

// }
