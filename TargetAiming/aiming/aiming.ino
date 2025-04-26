#include <Servo.h>

Servo panServo;
Servo tiltServo;

float panAngle;
float tiltAngle;

void setup() {
  Serial.begin(9600);

  panServo.attach(9);  // Attaches the servo on pin 9 to the servo object
  tiltServo.attach(10);

  panServo.write(90);
  tiltServo.write(90);
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
      Serial.print("Pan Angle: ");
      Serial.println(panAngle);
      Serial.print("Tilt Angle: ");
      Serial.println(tiltAngle);
    }

    panServo.write(panAngle);
    tiltServo.write(tiltAngle);
  }
}
