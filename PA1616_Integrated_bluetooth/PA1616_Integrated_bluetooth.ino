#include <Adafruit_GPS.h> //GPS
#include <Wire.h> 
#include <Adafruit_BNO08x.h> //BNO
#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Sensor Setup
#define GPSSerial Serial2 //GPS
#define RX_PIN    18 //GPS
#define TX_PIN    17 //GPS
#define GPSECHO false //GPS

#define wheelEncoderPin_forward 15 //Encoder
#define wheelEncoderPin_backward 16 //Encoder

#define BNO08X_CS 4 //BNO
#define BNO08X_INT 5 //BNO
#define BNO08X_RESET 6 //BNO

#define GPS_FLAG true
#define BNO false
#define HALL false

Adafruit_BNO08x   bno08x(BNO08X_RESET); //Send Reset to BNO-085
Adafruit_GPS GPS(&GPSSerial); //Connect to the GPS on hardware

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
float totalDistance = 0;
float prevDistance;
bool forwardFlag;
bool backwardFlag;
float unitConversion;
bool usingInches;
bool print_measurements;
int times_printed;
// printing variables
float lat;
char lat_dir;
float lon;
char lon_dir;
//interrupt varibles 
float station_distance = 0; // for automatic station creation
bool create_station = false;  // this will be sent from the tablet

uint32_t timer = millis(); //GPS

//Bluetooth Setup
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
//char rxValue = '\0';
const int readPin = 5  ; // Use GPIO number. See ESP32 board pinouts
const int LED = 48;      // pin of the RGB LED

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"//"2F05023F-F495-4259-8504-CFB307756EAF"// // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"//"B8CE9199-227D-422E-9A96-FD6766014BA8"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"//"DD9E2F15-EE4B-4F96-8231-18ADB1121294"

#define CHUNK_SIZE 20 // Define the chunk size

float bt_stationData[5]; // define arrary to transmit GPS, accelerometer, and wheel encoder data

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]); // print receiving key
        }

        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          Serial.print("Turning ON!");
         // txValue = analogRead(readPin) + 1;
          digitalWrite(LED, HIGH); 
          neopixelWrite(48,0,RGB_BRIGHTNESS,50);
          //pixels.fill(0xFFFF00); //RED   
          //pixels.show();
        }
        else if (rxValue.find("B") != -1) {
          Serial.print("Turning OFF!");
          //txValue = random(1, 10);
          digitalWrite(LED, LOW);
          neopixelWrite(48,64,0,RGB_BRIGHTNESS);
          //pixels.fill(0x800080); //YELLOW
          //pixels.show();
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};

// define wheel encoder interrupts
void ForwardISR() {
  /* if forward wheel encoder is sensed first,
  * this pin (1) will go low first, then the 
  * backward pin will go low after, the cart
  * is moving forward in this case, and
  * distance should be increased */
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
  if(GPS_FLAG){
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
  }

  //BNO
  if(BNO){
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
  }
  
  delay(100);

  // Write the column names to the csv
  if(HALL){
    Serial.print("dist");
  }
  if(GPS_FLAG){
    if(HALL){
      Serial.print(", ");
    }
    Serial.print("lat, lon");
  }
  if(BNO){
    if(HALL || GPS_FLAG){
      Serial.print(", ");
    }
    Serial.print("r, i, j, k");
  }
  Serial.println();

  // Blutooth Setup
  pinMode(LED, OUTPUT); // setting the RGB pin as the output

  // Create the BLE Device
  BLEDevice::init("ESP32 UART Test"); // Give the ESP32 a name
  //BLEDevice::setMTU(100);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  //Encoder LOOP
  //Calculate distance and print to csv every time loop executes
  if(forwardFlag) { // reset flags
    forwardFlag = false;
    backwardFlag = false;}
    prevDistance = totalDistance;
    totalDistance = ((wheelTicks / ticksPerRotation) * (wheelDiameter * PI) * unitConversion)/NumMags;
    station_distance += totalDistance - prevDistance
  #if HALL
    // Serial.print("Total distance = ");
    Serial.print(totalDistance);
    // Serial.println(" in");
  #endif

  //GPS LOOP
  //Get coordinates if there is a fix, print coords or empty to CSV
  lat = 0;
  lat_dir = '0';
  lon = 0;
  lon_dir = '0';
  char c = GPS.read(); //Read the data from the GPS
  // if (GPSECHO)
  //   if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())){} // this also sets the newNMEAreceived() flag to false
      //return; // we can fail to parse a sentence in which case we should just wait for another
  }
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
        lat = GPS.latitude;
        lat_dir = GPS.lat;
        lon = GPS.longitude;
        lon_dir = GPS.lon;
    }
  }
  // Print GPS data to csv
  if(GPS_FLAG){
    if(HALL){
    Serial.print(", ");
    }
    if(lat != 0 && lon != 0){
      Serial.print(lat); Serial.print(lat_dir); Serial.print(", ");
      Serial.print(lon); Serial.print(lon_dir);
    }
    else{
      Serial.print(", ");
    }
  }

  //BNO Loop
  #if BNO
  if(GPS_FLAG || HALL){
    Serial.print(", ");
  }
  if(ready_for_bno){
    if (bno08x.wasReset()) {
      // Serial.print("sensor was reset ");
      setReports();
    }

    if (! bno08x.getSensorEvent(&sensorValue)) {
      return;
    }

    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        // Serial.print("Game Rotation Vector - r: ");
        Serial.print(sensorValue.un.gameRotationVector.real); Serial.print(", ");
        // Serial.print(" i: ");
        Serial.print(sensorValue.un.gameRotationVector.i); Serial.print(", "); // slope
        // Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j); Serial.print(", "); // cross grade
        // Serial.print(" k: ");
        Serial.print(sensorValue.un.gameRotationVector.k);  
        break;
    }
    ready_for_bno = true;
  }
  else{
    Serial.print(", , , ");
  }
  #endif

  Serial.println();

  // if data needs to be sent to the tablet, do it here
  if(station_distance > 300 || create_station){  //recorded 25 feet or user made a station
    if(lat_dir == 'S') {
      bt_stationData[0] = -1 * lat;
    }  
    else {
      bt_stationData[0] = -1 * lat;
    }

    if(lon_dir == 'W') {
      bt_stationData[1] = -1 * lon;
    }
    else {
      bt_stationData[1] = lon;
    }

    bt_stationData[2] = sensorValue.un.gameRotationVector.i; // need to adjust to do whatever math necessary to convert from ADU to percentage
    bt_stationData[3] = sensorValue.un.gameRotationVector.j;
    bt_stationData[4] = totalDistance;

    // this array will be sent to the tablet as data packet #1
    // BLUETOOTH CODE HERE
    pCharacteristic->setValue(bt_stationData); //-> string part

    pCharacteristic->notify(); // Send the value to the app!
    Serial.print("*** Sent Value: ");
    Serial.print(bt_stationData);
    Serial.println(" ***");

    station_distance = 0;
    create_station = false;
  }
}
