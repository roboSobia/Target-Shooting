// // #include <Servo.h>

// // Servo panServo;  // Create servo object to control a servo
// // // Servo tiltServo;  // Create servo object to control a servo

// //  int imageWidth = 640; // width of your camera frame
// //  int imageHeight = 480; // width of your camera frame
// // float x_cameraFOV = 70.0; // in degrees
// // float y_cameraFOV = 53; // in degrees

int targetX = 295;
int targetY = 90;


// // float calculatePanAngle(int targetX, int imageWidth, float camFOV) {
// //   int centerX = imageWidth / 2;
// //   float angle = 90 - ((float)(targetX - centerX) / imageWidth) * camFOV;
// //   return angle;
// // }

// // float calculateTiltAngle(int targetY, int imageHeight, float camFOV) {
// //   int centerY = imageHeight / 2;
// //   float angle = 90 - ((float)(targetY - centerY) / imageHeight) * camFOV;
// //   return angle;
// // }

// // void setup() {
// //   panServo.attach(9);  // Attaches the servo on pin 9 to the servo object
// //   // tiltServo.attach(5);  // Attaches the servo on pin 9 to the servo object
// // }

// // void loop() {

// // float x_angle = calculatePanAngle(targetX, imageWidth, x_cameraFOV);
// // // float y_angle = calculateTiltAngle(targetY, imageHeight, y_cameraFOV);

// // panServo.write(x_angle);
// // // tiltServo.write(y_angle);

// // // delay(1000);

// // // panServo.write(0);
// // // delay(1000);


// // }

// // // #include <Servo.h>

// // // Servo myServo;  // Create a servo object

// // // void setup() {
// // //   Serial.begin(9600);
// // //   myServo.attach(9);  // Attach the servo to digital pin 9
 

// // // }

// // // void loop() {
// // //   // Nothing needed here if you're only moving once
// // //    myServo.write(90);  // Move the servo to 90 degrees
// // //   delay(1000);
// // //   myServo.write(0);
// // //     delay(1000);

// // // }

// #include <Servo.h>

// Servo myServo;
// int pos = 90; // Start at mid-position (0 - 180)

// void setup() {
//   myServo.attach(9); // Attach servo to pin 9
//   myServo.write(pos); // Set initial position
//   Serial.begin(9600); // Start Serial
// }

// void loop() {
//   if (Serial.available()) {
//     char key = Serial.read();
//     if (key == 'L') { // Left arrow
//       pos = constrain(pos - 5, 0, 180);
//       myServo.write(pos);
//     } else if (key == 'R') { // Right arrow
//       pos = constrain(pos + 5, 0, 180);
//       myServo.write(pos);
//     }
//   }
// }

#include <Servo.h>

Servo panServo;
// Servo tiltServo;

int imageWidth = 640;  // DroidCam resolution
int imageHeight = 480;
float x_cameraFOV = 86.0;  // Horizontal FOV
float y_cameraFOV = 64.5;  // Vertical FOV
float z_distance = 2.0;    // Estimated target distance in meters
float servo_offset_x = 0.15; // Servo 15 cm right of camera

float calculatePanAngle(int targetX, int imageWidth, float camFOV, float z, float offset_x) {
  int centerX = imageWidth / 2;
  // Camera angle in degrees
  float theta_camera = ((float)(targetX - centerX) * (camFOV / imageWidth));
  // Convert to radians for calculation
  float theta_camera_rad = theta_camera * PI / 180.0;
  // Adjust for servo offset
  float theta_servo_rad = atan(tan(theta_camera_rad) - offset_x / z);
  // Convert back to degrees and add servo center
  float angle = 90.0 - (theta_servo_rad * 180.0 / PI);
  return angle;
  // return constrain(angle, 0, 180);
}

float calculateTiltAngle(int targetY, int imageHeight, float camFOV) {
  int centerY = imageHeight / 2;
  float angle = 90.0 + ((float)(targetY - centerY) * (camFOV / imageHeight));
  return constrain(angle, 0, 180);
}

void setup() {
  panServo.attach(9);
  // tiltServo.attach(5);
  Serial.begin(9600);
}

void loop() {
      // int targetX = data.substring(0, commaIndex).toInt();
      // int targetY = data.substring(commaIndex + 1).toInt();
      float x_angle = calculatePanAngle(targetX, imageWidth, x_cameraFOV, z_distance, servo_offset_x);
      // float y_angle = calculateTiltAngle(targetY, imageHeight, y_cameraFOV);
      panServo.write(x_angle);
      
}
