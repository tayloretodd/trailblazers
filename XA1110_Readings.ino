#include <Wire.h>
#include "SparkFun_I2C_GPS_Arduino_Library.h"
#include <TinyGPS++.h>

 // select ESP32  I2C pins
#define SDA0_Pin 8
#define SCL0_Pin 9
I2CGPS myI2CGPS;
TinyGPSPlus gps;
bool print_measurements = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(SDA0_Pin, SCL0_Pin);
  if (myI2CGPS.begin() == false)//Checks for succesful initialization of GPS
  {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }
  Serial.println("GPS module found!");
}

// void loop() {
//   // put your main code here, to run repeatedly:
//   while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
//   {
//     byte incoming = myI2CGPS.read(); //Read the latest byte from Qwiic GPS

//     if(incoming == '$') Serial.println(); //Break the sentences onto new lines
//     Serial.write(incoming); //Print this character
//   }
// }

void loop()
{
  while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
  {
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }

  if (gps.time.isUpdated() && print_measurements) //Check to see if new GPS info is available
  {
    displayInfo();
  }
}

//Display new GPS info
void displayInfo()
{
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
