#include <Servo.h>

Servo panServo;  // Create servo object to control a servo
Servo tiltServo;

// Arduino code to receive balloon color and position from Python via Serial

String color = "";
int centerX = 0;
int centerY = 0;

int imageWidth = 640;    // width of your camera frame
int imageHeight = 480;   // width of your camera frame
float x_cameraFOV = 86;  // in degrees
float y_cameraFOV = 53;  // in degrees
int currentAngleX = 0, targetAngleX = 0;
int currentAngleY = 0, targetAngleY = 0;
float distance_to_target_cm = 150;
float laser_offset_cm_x = 7;
float laser_offset_cm_y = 0;


//Pan code
float calculatePanAngle(int targetX, int imageWidth, float camFOV) {
  int centerX = imageWidth / 2;
  float angle = 90 - ((float)(targetX - centerX) / imageWidth) * camFOV;
  return angle;
}

//Tilt code
float calculateTiltAngle(int targetY, int imageHeight, float camFOV) {
  int centerY = imageHeight / 2;
  float angle = 90 - ((float)(targetY - centerY) / imageHeight) * camFOV;
  return angle;
}
void setup() {
  Serial.begin(9600);


  panServo.attach(9);  // Attaches the servo on pin 9 to the servo object
  tiltServo.attach(10);

 // panServo.write(90);

  tiltServo.write(90);
 

}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');  // Read until newline
    data.trim();                                 // Remove whitespace

    int colorEnd = data.indexOf(')');                     // Find the end of "rgb(...)"
    int firstComma = data.indexOf(',', colorEnd + 1);     // First comma after color
    int secondComma = data.indexOf(',', firstComma + 1);  // Second comma after color
                                                          // int thirdComma = data.indexOf(',', secondComma + 1);

    if (colorEnd > 0 && firstComma > colorEnd && secondComma > firstComma) {
      color = data.substring(0, colorEnd + 1);  // Include the closing ')'
      centerX = data.substring(firstComma + 1, secondComma).toInt();
      centerY = data.substring(secondComma + 1).toInt();
      // distance_to_target_cm = data.substring(thirdComma + 1).toFloat(); //
    }


    // Example: print received values to Serial Monitor

    float fov_rad = radians(x_cameraFOV);
    float width_at_target = 2 * distance_to_target_cm * tan(fov_rad / 2);  // cm
    float pixels_per_cm_x = imageWidth / width_at_target;
    float LASER_CAMERA_OFFSET_PIXELS_X = laser_offset_cm_x * pixels_per_cm_x;


    currentAngleX = panServo.read();  // Get current angle
    int adjustedTargetX = centerX + LASER_CAMERA_OFFSET_PIXELS_X;
    targetAngleX = round(calculatePanAngle(adjustedTargetX, imageWidth, x_cameraFOV));
    targetAngleX = constrain(targetAngleX, 0, 180);



    //Debugging
    Serial.print("CenterX: ");
    Serial.print(centerX);
    Serial.print(" | Adjusted Angle X: ");
    Serial.println(targetAngleX);

    //tilt code
    float fov_rad_y = radians(y_cameraFOV);                                    // Vertical FOV in radians
    float height_at_target = 2 * distance_to_target_cm * tan(fov_rad_y / 2);   // Height in cm at target distance
    float pixels_per_cm_y = imageHeight / height_at_target;                    // Pixels per cm in vertical direction
    float LASER_CAMERA_OFFSET_PIXELS_Y = laser_offset_cm_y * pixels_per_cm_y;  // Vertical laser offset in pixels

    currentAngleY = tiltServo.read();                                                     // Get current tilt angle
    int adjustedTargetY = centerY + LASER_CAMERA_OFFSET_PIXELS_Y;                         // Adjust target Y for laser offset
    targetAngleY = round(calculateTiltAngle(adjustedTargetY, imageHeight, y_cameraFOV));  // Calculate tilt angle
    targetAngleY = constrain(targetAngleY, 0, 180);                                       // Constrain to servo limits
      //Debugging
    Serial.print("CenterY: ");
    Serial.print(centerY);
    Serial.print(" | Adjusted Angle Y: ");
    Serial.println(targetAngleY);


   // if (abs(currentAngleX - targetAngleX) > 1)
     {
      panServo.write(targetAngleX);
    }
   // if (abs(currentAngleY - targetAngleY) > 1) 
    {
       tiltServo.write(targetAngleY);
    }
  }

  
  // panServo.write(90);

  // // delay(1000);
  // // tiltServo.write(90);
  // delay(500);
  // tiltServo.write(90);

}
