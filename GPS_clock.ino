
#include <Wire.h>
#include <Adafruit_GPS.h>

int PPS_1 = 19;

#define GPSSerial Serial2

Adafruit_GPS GPS(&GPSSerial);

float iter_1;

void setup() {

  attachInterrupt(digitalPinToInterrupt(PPS_1), ISR_1, RISING);

  Serial.begin(9600);
  Serial.println("Starting");
  
  GPS.begin(9600);
  iter_1 = 0.;
}

void loop() {
  // put your main code here, to run repeatedly:

}



void ISR_1 () {
  //Serial.println("PPS1 millis " + String(iter_1/1000) + ": " + String(millis() - iter_1));
  //iter_1 += 1000;
  Serial.println("PPS1 millis: " + String(millis()));
}
