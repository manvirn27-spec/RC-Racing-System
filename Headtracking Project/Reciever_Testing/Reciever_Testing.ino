#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// CE on pin 9, CSN on pin 10
RF24 radio(9, 10);

const byte address[6] = "00001";

// Define the same struct for the received data
struct Angles {
  float yawAngle;
  float tiltAngle;
};

Angles angles;
Servo yawServo;
Servo tiltServo;


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  yawServo.attach(5);
  tiltServo.attach(6);
}

void loop() {
  if (radio.available()) {
    radio.read(&angles, sizeof(angles));
    Serial.print("Received: ");
    Serial.print(angles.yawAngle);
    Serial.print(", ");
    Serial.println(angles.tiltAngle);

    int yawServoPos  = constrain(map((int)angles.yawAngle, 0, 180, 0, 180), 0, 180);
    int tiltServoPos = constrain(map((int)angles.tiltAngle, 0, 180, 0, 180), 0, 180);

    yawServo.write(yawServoPos);
    tiltServo.write(tiltServoPos);

    delay(15);
  }

}