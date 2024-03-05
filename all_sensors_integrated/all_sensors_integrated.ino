#include <Wire.h> 
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
//Adafruit sensor libraries
#include <Adafruit_GPS.h> //GPS
#include <Adafruit_BNO08x.h> //BNO
#include "Adafruit_BME680.h"
#include "Adafruit_PM25AQI.h"
#include <Adafruit_INA219.h>


#define GPSSerial Serial2 //GPS
#define RX_PIN    18 //GPS
#define TX_PIN    17 //GPS
#define GPSECHO false //GPS

#define wheelEncoderPin_forward 15 //Encoder
#define wheelEncoderPin_backward 16 //Encoder

#define           BNO08X_CS 4 //BNO
#define           BNO08X_INT 5 //BNO
#define           BNO08X_RESET 6 //BNO

#define GPS_FLAG true
#define BNO true
#define HALL true

Adafruit_BNO08x   bno08x(BNO08X_RESET); //Send Reset to BNO-085
Adafruit_GPS GPS(&GPSSerial); //Connect to the GPS on hardware

uint32_t timer = millis(); //GPS

sh2_SensorValue_t sensorValue;
int defaultR;
int defaultI;
int defaultJ;
int defaultK;
bool ready_for_bno = true;
int ticksPerRotation;
int NumMags;
float wheelDiameter;
int wheelTicks;
float totalDistance;
bool forwardFlag;
bool backwardFlag;
float unitConversion;
bool usingInches;
bool print_measurements;
int times_printed;

//environmental sensors setup
#define SEALEVELPRESSURE_HPA (1013.25)
//initialize all sensors
Adafruit_BME680 bme;
Adafruit_INA219 ina219;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
//sensors variables
int counter = 0;
float total_temp = 0;
float total_hum = 0;
float total_pressure = 0; 
float total_co2 = 0;
int co2_counter = 0;
float total_count = 0;
float total_loadvoltage = 0;
float total_current = 0;
float total_c = 0;


int bt_stationData[5]; // define arrary to transmit GPS, accelerometer, and wheel encoder data

// define wheel encoder interrupts
/* if forward wheel encoder is sensed first,
 * this pin (1) will go low first, then the 
 * backward pin will go low after, the cart
 * is moving forward in this case, and
 * distance should be increased */
void ForwardISR() {
  forwardFlag = backwardFlag ? false : true; // if backward flag is already set, this interrupt was triggered secont
  wheelTicks++;
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); //Sets correct pins for GPS
//Encoder Setup
  pinMode(wheelEncoderPin_forward, INPUT); //Sets Pin to INPUT
  attachInterrupt(wheelEncoderPin_forward, ForwardISR, FALLING); //Sets pin to call Forward service routine on Falling Edge
  ticksPerRotation = 2;
  wheelDiameter = 15.0;
  usingInches = true;
  wheelTicks = 0;   // could be a value if starting from a station
  NumMags = 8;
  unitConversion = 1;
  print_measurements = false; 

//GPS SETUP  
  // 9600 NMEA is recommended by Rylan
  if(GPS.begin(9600)){
    Serial.println("GPS Begin!");
  }
  else{
    Serial.println("GPS HAULT!");
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

//BNO
  // Try to initialize BNO-085
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
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

  if (!bme.begin()) {
    // Serial.println("Could not find a valid BME688 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  if (! ina219.begin()) {
    // Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  if (! aqi.begin_I2C()) {
    // Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }

  delay(100);

}

void loop() {
//Encoder LOOP  
  if(forwardFlag) { // reset flags
    forwardFlag = false;
    backwardFlag = false;}
    totalDistance = ((wheelTicks / ticksPerRotation) * (wheelDiameter * PI) * unitConversion)/NumMags;
  #if HALL
    Serial.print("Total distance = ");
    Serial.print(totalDistance);
    Serial.println(" in");
  #endif

//GPS LOOP
  char c = GPS.read(); //Read the data from the GPS
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())){} // this also sets the newNMEAreceived() flag to false
      //return; // we can fail to parse a sentence in which case we should just wait for another
  }
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
      //#if (GPS_FIX)
        Serial.print("Fix: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //#endif
    }
  }
//BNO Loop
  #if BNO
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
        Serial.print(sensorValue.un.gameRotationVector.i); // slope
        Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j); // cross grade
        Serial.print(" k: ");
        Serial.println(sensorValue.un.gameRotationVector.k);  
        break;
    }
    ready_for_bno = true;
  }
  #endif

  //sensor readings 
  //BME readings 
  if (! bme.performReading()) {
    // Serial.println("Failed to perform BME reading :(");
    return;
  }
  float temp = (bme.temperature * (9/5)) + 32;
  // total_temp += temp;
  float pressure = (bme.pressure / 100.0) * 30;
  // total_pressure += pressure;
  float humidity = bme.humidity;
  // total_hum += humidity;
  Serial.print("Temperature: ");
  Serial.println(temp);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  //particle count 
  PM25_AQI_Data data;
  if (! aqi.read(&data)) {    
    // Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
    return;
  }
  int particle_count = data.particles_03um + data.particles_05um + data.particles_10um;
  // Serial.print(particle_count);
  // Serial.print(", ");
  total_count += particle_count;
  Serial.print("Particle Count: ");
  Serial.println(particle_count);
  
  //INA Readings 
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  total_loadvoltage += loadvoltage;
  total_current += current_mA;
  // Serial.print(loadvoltage);
  // Serial.print(", ");
  // Serial.print(current_mA);
  // Serial.print(", ");
  Serial.print("Current (mA): ");
  Serial.println(current_mA);

  delay(1000);

  if(GPS.lat = 'S') {
    bt_stationData[0] = -1 * GPS.latitude;
  }  
  else {
    bt_stationData[0] = -1 * GPS.latitude;
  }

  if(GPS.lon = 'W') {
    bt_stationData[1] = -1 * GPS.longitude;
  }
  else {
    bt_stationData[1] = GPS.longitude;
  }

  bt_stationData[2] = sensorValue.un.gameRotationVector.i; // need to adjust to do whatever math necessary to convert from ADU to percentage
  bt_stationData[3] = sensorValue.un.gameRotationVector.j;
  bt_stationData[4] = totalDistance;

  // this array will be sent to the tablet as data packet #1

}
