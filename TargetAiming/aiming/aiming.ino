#include <Servo.h>

Servo panServo;  // Create servo object to control a servo

// Arduino code to receive balloon color and position from Python via Serial

String color = "";
int centerX = 0;
int centerY = 0;

int imageWidth = 640;    // width of your camera frame
int imageHeight = 480;   // width of your camera frame
float x_cameraFOV = 86;  // in degrees
float y_cameraFOV = 53;  // in degrees
int currentAngle = 0, targetAngle = 0;
float distance_to_target_cm = 150;
float laser_offset_cm = 7;
float calculatePanAngle(int targetX, int imageWidth, float camFOV) {
  int centerX = imageWidth / 2;
  float angle = 90 - ((float)(targetX - centerX) / imageWidth) * camFOV;
  return angle;
}
void setup() {
  Serial.begin(9600);
  panServo.attach(9);  // Attaches the servo on pin 9 to the servo object
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');  // Read until newline
    data.trim();                                 // Remove whitespace

    int colorEnd = data.indexOf(')');                     // Find the end of "rgb(...)"
    int firstComma = data.indexOf(',', colorEnd + 1);     // First comma after color
    int secondComma = data.indexOf(',', firstComma + 1);  // Second comma after color

    if (colorEnd > 0 && firstComma > colorEnd && secondComma > firstComma) {
      color = data.substring(0, colorEnd + 1);  // Include the closing ')'
      centerX = data.substring(firstComma + 1, secondComma).toInt();
      centerY = data.substring(secondComma + 1).toInt();
    }


    // Example: print received values to Serial Monitor

    float fov_rad = radians(x_cameraFOV);
    float width_at_target = 2 * distance_to_target_cm * tan(fov_rad / 2);  // cm
    float pixels_per_cm = imageWidth / width_at_target;
    float LASER_CAMERA_OFFSET_PIXELS = laser_offset_cm * pixels_per_cm;




    currentAngle = panServo.read();  // Get current angle
    int adjustedTargetX = centerX + LASER_CAMERA_OFFSET_PIXELS;
    targetAngle = round(calculatePanAngle(adjustedTargetX, imageWidth, x_cameraFOV));
    targetAngle = constrain(targetAngle, 0, 180);
    Serial.print("CenterX: ");
    Serial.print(centerX);
    Serial.print(" | Adjusted Angle: ");
    Serial.println(targetAngle);

    //if (abs(currentAngle - targetAngle) > 1)
    {
      panServo.write(targetAngle);
    }
  }

//panServo.write(90);
}

