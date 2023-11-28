#include <Wire.h>
#include "SparkFun_I2C_GPS_Arduino_Library.h"
#include <TinyGPS++.h>
#include <Adafruit_BNO08x.h>

 // select ESP32  I2C pins
#define       SDA0_Pin 8
#define       SCL0_Pin 9
I2CGPS        myI2CGPS;
TinyGPSPlus   gps;

// For SPI mode, we need a CS pin and RESET pin
#define           BNO08X_CS 4
#define           BNO08X_INT 5
#define           BNO08X_RESET 6
Adafruit_BNO08x   bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
int defaultR;
int defaultI;
int defaultJ;
int defaultK;
bool ready_for_bno = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Set up the GPS over I2C
  Wire.begin(SDA0_Pin, SCL0_Pin);
  if (myI2CGPS.begin() == false)//Checks for succesful initialization of GPS
  {
    Serial.println("GPS module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }
  Serial.println("GPS module found!");

  // Try to initialize BNO-085
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit BNO08x test!");
  //if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();

  Serial.println("Reading events");

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR){
    defaultR=sensorValue.un.gameRotationVector.real;
    defaultI=sensorValue.un.gameRotationVector.i;
    defaultJ=sensorValue.un.gameRotationVector.j;
    defaultK=sensorValue.un.gameRotationVector.k;
  }
  
  delay(100);
}

void loop()
{
  while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
  {
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }
  if (gps.time.isUpdated()) //Check to see if new GPS info is available
  {
    displayInfo();
    ready_for_bno = true;
  }

  delay(10);
  
  if(ready_for_bno){
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }

    if (! bno08x.getSensorEvent(&sensorValue)) {
      return;
    }

    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        Serial.print("Game Rotation Vector - r: ");
        Serial.print(sensorValue.un.gameRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.gameRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.gameRotationVector.k);
        // if (sensorValue.un.gameRotationVector.i < -0.75 | sensorValue.un.gameRotationVector.i > 0.75){
        //   Serial.println(defaultI - sensorValue.un.gameRotationVector.i);
        //   Serial.println("down ");
        // }
        // else{
        //   Serial.println(defaultI - sensorValue.un.gameRotationVector.i);
        //   Serial.println("up ");
        // }
        break;
    }
    ready_for_bno = false;
  }
  
}

//Display new GPS info
void displayInfo() {
  //We have new GPS data to deal with!
  Serial.println();

  if (gps.time.isValid())
  {
    Serial.print(F("Date: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());

    Serial.print((" Time: "));
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());

    Serial.println(); //Done printing time
  }
  else
  {
    Serial.println(F("Time not yet valid"));
  }

  if (gps.location.isValid())
  {
    Serial.print("Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  }
  else
  {
    Serial.println(F("Location not yet valid"));
  }
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}
