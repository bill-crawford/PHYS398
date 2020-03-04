/*
PHYS 398 SP20 Group 1: Data Acquisition Software
Uses open-source code written by Tom Igoe, Alexander Brevig, Arduino, and Adafruit Industries.

 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_GPS.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

#define GPSSerial Serial2 
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_GPS GPS(&GPSSerial);

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 36, d5 = 34, d6 = 32, d7 = 30;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int ROWS = 4;
const int COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3',},
  {'4','5','6',},
  {'7','8','9',},
  {'*','0','#',}
};
byte rowPins[ROWS] = {31,33,35,37}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {2,3,18}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

Adafruit_BME680 bme; // I2C

const int chipSelect = 53;


int PPS_1 = 19
attachInterrupt(digitalPinToInterrupt(PPS_1), ISR_1, RISING);
GPS.begin(9600);
float iter_1 = 0.
void ISR_1 () {
    float systemTime = micros();
    
    iter_1 += 1000;
    


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  GPS.begin(9600);

  if (!bme.begin()) {
    //Serial.println("Could not find a valid BME680 sensor, check wiring!");
    Serial.print("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");



  // Begin a while loop which runs the drift-computing code until a key is pressed...
  float startTime = micros()
  char key = keypad.getKey();
  while (!key = 1)  {
    //put the drift-calculating code here
    //Get the current clock time, and then subtract (# of times PPS has pulsed)*(inital clock time)
    float realTime = startTime + 1000*(number of PPS signals recieved)
    float drift = realTime - micros()
    
    key = keypad.getKey();
  }

}

void loop() {


  
  String test_time = String((micros() - start_time) / 1000.);

  sample = analogRead(7);
  double volts = (sample * 5.0) / 1024;  // convert to volts

  if (! bme.performReading()) {
    //Serial.println("Failed to perform reading :(");
    Serial.print("Failed to perform reading :(");
    return;
  }
  
  // make a string for assembling the data to log:
  String temperature = "Temperature (*C): " + String(bme.temperature);
  String pressure = "Pressure (hPa): " + String(bme.pressure / 100.);
  String humidity = "Humidity (%): " + String(bme.humidity);
  String gas = "Gas (KOhms): " + String(bme.gas_resistance / 1000.);
  String altitude = "Altitude (m): " + String(bme.readAltitude(SEALEVELPRESSURE_HPA));
  
  String mic_volts = "Mic Volts (> 0.5): " + String(volts);

  //before doing slow memory writing:
  
      
    Serial.println("Time (s): " + test_time);
    Serial.println(mic_volts);
    Serial.println(temperature);
    Serial.println(pressure);
    Serial.println(humidity);
    Serial.println(gas);
    Serial.println(altitude);
    Serial.println();
  }
}
  /*
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    if (volts > 0.5) {
      dataFile.println("Time (s): " + test_time);
      dataFile.println(mic_volts);
      dataFile.println(temperature);
      dataFile.println(pressure);
      dataFile.println(humidity);
      dataFile.println(gas);
      dataFile.println(altitude);
      dataFile.println();
      // print to the serial port too:
      Serial.println("Time (s): " + test_time);
      Serial.println(mic_volts);
      Serial.println(temperature);
      Serial.println(pressure);
      Serial.println(humidity);
      Serial.println(gas);
      Serial.println(altitude);
      Serial.println();
    }
    dataFile.close();
    
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
*/
