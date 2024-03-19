
#include <Wire.h> 
#include <SPI.h>
#include <Arduino.h>
#include <math.h>
#include <Adafruit_Sensor.h>
//Sensor Libraries
#include <Adafruit_BNO08x.h> //9 DOF
#include <Adafruit_GPS.h> //GPS
#include "Adafruit_BME680.h"//Temp,Pressure,Humidity
#include "Adafruit_PM25AQI.h"//Particle Count
#include <Adafruit_INA219.h>//Voltage/Current
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define GPSSerial Serial2 //GPS
#define RX_PIN    18 //GPS
#define TX_PIN    17 //GPS
#define GPSECHO true //GPS

#define wheelEncoderPin_forward 38 //Encoder
#define wheelEncoderPin_backward 16 //Encoder

#define           BNO08X_CS 4 //BNO
#define           BNO08X_INT 5 //BNO
#define           BNO08X_RESET 6 //BNO

//environmental sensors setup
#define SEALEVELPRESSURE_HPA (1013.25)

#define GPS_FLAG true
#define BNO true
#define HALL true
#define BME true
#define INA true
#define PM25AQI true

#define PUTTY false //Print "Acell Z: 0.00 Azell Y: 0.00" or "0.00, 0.00, "


Adafruit_BNO08x   bno08x(BNO08X_RESET); //Send Reset to BNO-085
Adafruit_GPS GPS(&GPSSerial); //Connect to the GPS on hardware


uint32_t timer = millis(); //GPS
//BNO
sh2_SensorValue_t sensorValue; //BNO Sensor Value
int defaultR; //BNO Real
int defaultI; //BNO I Vector
int defaultJ; //BNO J Vector
int defaultK; //BNO K Vector
bool ready_for_bno = true; 
float alpha;
float AccelZ;
float intermed;
//Wheel Encoder
int ticksPerRotation; //# Magnets on the wheel
float wheelDiameter;
int wheelTicks;
float totalDistance;
float prevDistance;
bool forwardFlag;
bool backwardFlag;
bool readFlag;
bool StoreFlag;
float unitConversion;
bool usingInches;
int times_printed;
int test = 0;
unsigned long intTime = 0;
unsigned long lastTime = 0;
//GPS
float lat = 0;
float lon = 0;
char lat_string = 'X';
char lon_string = 'X';
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
//INA Readings 
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
//PM25AQI
int particle_count;
//BME
float temp;
float pressure; 
float humidity; 
// interrupt variables
float station_distance = 0; // for automatic station creation
bool create_station = false;  // this will be sent from the tablet

//Bluetooth Setup
// having each piece of data be its own characteristic will simplify sending multiple packets.
BLECharacteristic *pCharacteristic_GPS;   // GPS data characteristic
BLECharacteristic *pCharacteristic_ACC;   // Accelerometer data characteristic
BLECharacteristic *pCharacteristic_DIST;  // Gyroscope data characteristic
BLECharacteristic *pCharacteristic_BATT;  // Battery voltage characteristic
BLECharacteristic *pCharacteristic_ENV;   // environmental sensors data characteristic (may have to split this up)
BLECharacteristic *pCharacteristic_RX; // characteristic for writing to ESP32

bool deviceConnected = false;
const int LED = 48;      // pin of the RGB LED

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9A"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9B"   // for tablet writes to ESP32
#define CHARACTERISTIC_UUID_GPS "6E400003-B5A3-F393-E0A9-E50E24DCCA9C"
#define CHARACTERISTIC_UUID_ACC "6E400003-B5A3-F393-E0A9-E50E24DCCA9D"
#define CHARACTERISTIC_UUID_DIST "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_BATT "6E400003-B5A3-F393-E0A9-E50E24DCCA9F"
#define CHARACTERISTIC_UUID_ENV "6E400003-B5A3-F393-E0A9-E50E24DCCA90"

#define CHUNK_SIZE 20 // Define the chunk size

float bt_stationData[10]; // define arrary to transmit GPS, accelerometer, and wheel encoder data

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

void BackwardISR(){
  ///backwardFlag = true;
}
// define wheel encoder interrupts
/* if forward wheel encoder is sensed first,
 * this pin (1) will go low first, then the 
 * backward pin will go low after, the cart
 * is moving forward in this case, and
 * distance should be increased */
void IRAM_ATTR ForwardISR()
{
  noInterrupts();
  intTime = millis();
  if((digitalRead(wheelEncoderPin_forward) == LOW) && (intTime - lastTime > 150))
    {
      wheelTicks++;
      if (wheelTicks % 4 == 0){
        readFlag = 1;
      }
      StoreFlag = 1;
      lastTime=intTime;
    }
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
  return;
}

void setup() 
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); //Sets correct pins for GPS
  
  //Encoder Setup
  pinMode(wheelEncoderPin_forward, INPUT); //Sets Pin to INPUT
  attachInterrupt(digitalPinToInterrupt(wheelEncoderPin_forward), ForwardISR, FALLING); //Sets pin to call Forward service routine on Falling Edge
  pinMode(wheelEncoderPin_backward, INPUT); //Sets Pin to INPUT
  attachInterrupt(digitalPinToInterrupt(wheelEncoderPin_backward), BackwardISR, FALLING); //Sets pin to call Back service routine on Falling Edge
  ticksPerRotation = 4; //# mags
  wheelDiameter = 15.0;
  usingInches = true;
  wheelTicks = 0;   // could be a value if starting from a station
  unitConversion = 1;
  alpha = 1/ticksPerRotation;
  AccelZ = -2;
  
  //GPS SETUP  
  #if GPS_FLAG
    // 9600 NMEA is recommended by Rylan
    if(GPS.begin(9600))
    {
      Serial.println("GPS Begin!");
    }
    else
    {
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
  #endif 

  //BNO
  #if BNO
    // Try to initialize BNO-085
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) 
    {
      Serial.println("Failed to find BNO08x chip");
      while (1) { delay(10); }
    }
    Serial.println("BNO08x Found!");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) 
    {
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

    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
      defaultR=sensorValue.un.gameRotationVector.real;
      defaultI=sensorValue.un.gameRotationVector.i;
      defaultJ=sensorValue.un.gameRotationVector.j;
      defaultK=sensorValue.un.gameRotationVector.k;
    }

    Serial.println("sensor was TARED ");
    sh2_setTareNow(SH2_TARE_Z,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
    sh2_setTareNow(SH2_TARE_X,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
    sh2_setTareNow(SH2_TARE_Y,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
    sh2_setTareNow(SH2_TARE_Y,SH2_TARE_BASIS_ROTATION_VECTOR);
    sh2_setTareNow(SH2_TARE_Z,SH2_TARE_BASIS_ROTATION_VECTOR);
    sh2_setTareNow(SH2_TARE_X,SH2_TARE_BASIS_ROTATION_VECTOR);
  #endif 
  
  //BME
  #if BME
    if (!bme.begin()) 
    {
      // Serial.println("Could not find a valid BME688 sensor, check wiring!");
      while (1);
    }
    
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  #endif

  //INA
  #if INA
    if (! ina219.begin()) 
    {
      // Serial.println("Failed to find INA219 chip");
      while (1) 
      { 
        delay(10); 
      }
    }
  #endif

  //PM25Aqi
  #if PM25AQI
    if (! aqi.begin_I2C()) 
    {
      // Serial.println("Could not find PM 2.5 sensor!");
      while (1) 
      {
        delay(10);
      }
    }
    delay(100);
  #endif

  // Blutooth Setup
  pinMode(LED, OUTPUT); // setting the RGB pin as the output

  // Create the BLE Device
  BLEDevice::init("TrailSense ESP32"); // Give the ESP32 a name
  //BLEDevice::setMTU(100);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  // Write to ESP Characteristic
  pCharacteristic_RX = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE
                                      );
  pCharacteristic_RX->setCallbacks(new MyCallbacks());
  // pCharacteristic_RX->addDescriptor(new BLE2902());
  
  // Transmit GPS data Characteristic
  // pCharacteristic_GPS = pService->createCharacteristic(
  //                     CHARACTERISTIC_UUID_GPS,
  //                     BLECharacteristic::PROPERTY_NOTIFY
  //                   );

  // Transmit accelerometer data Characteristic
  pCharacteristic_ACC = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ACC,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // // Transmit whel encoder data Characteristic
  pCharacteristic_DIST = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_DIST,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // // Transmit battery life data Characteristic
  pCharacteristic_BATT = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_BATT,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  //   // Transmit environmental sensors data Characteristic
  pCharacteristic_ENV = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ACC,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}


void loop() 
{
  //Encoder LOOP  
  if(forwardFlag) 
  { // reset flags
    forwardFlag = false;
    backwardFlag = false;
  }
    
  #if HALL
  // if (StoreFlag){
  //       StoreFlag = 0;
  //       intermed = sensorValue.un.accelerometer.z;
  //       if (readFlag){
  //         AccelZ = intermed;
  //         Serial.print(" Accel z Fresh:");
  //         Serial.println(AccelZ);
  //       }
  //       else{
  //         AccelZ = ((1-alpha) * AccelZ) + (alpha * intermed);
  //         Serial.print(" Accel z:");
  //         Serial.print(AccelZ);
  //         Serial.print(" 1 z:");
  //         Serial.print(((1-alpha) * AccelZ));
  //         Serial.print(" 1 z:");
  //         Serial.print((alpha * intermed));
  //         Serial.print(" Value z:");
  //         Serial.println(intermed);
  //       }
  // }

    if(readFlag)
    {
      prevDistance = totalDistance;
      totalDistance = ((wheelTicks / ticksPerRotation) * (wheelDiameter * PI) * unitConversion);
      station_distance += totalDistance - prevDistance;
      
      #if PUTTY
        readFlag = 0;
        #if BNO
          Serial.print( sensorValue.un.accelerometer.z*180);
          Serial.print(", ");
          Serial.print(sensorValue.un.accelerometer.x*180);
          Serial.print(", ");
          Serial.print(sensorValue.un.accelerometer.y*180);
          Serial.print(", ");
          Serial.print(sensorValue.un.gyroscope.z*180);
          Serial.print(", ");
          Serial.print(sensorValue.un.gyroscope.x*180);
          Serial.print(", ");
          Serial.print(sensorValue.un.gyroscope.y*180);
          Serial.print(", ");
        #endif
        
        #if GPS_FLAG
          Serial.print(lon);
          Serial.print(GPS.lon);
          Serial.print(", ");
          Serial.print(lat);
          Serial.print(GPS.lat);
          Serial.print(", ");
        #endif
        
        #if INA
          Serial.print(current_mA);
          Serial.print(", ");
        #endif
        
        #if BME
          Serial.print(temp);
          Serial.print(", ");
          Serial.print(pressure);
          Serial.print(", ");
          Serial.print(humidity);
          Serial.print(", ");
        #endif
        
        #if PM25AQI
          Serial.println(particle_count);
        #endif

      #else // PUTTY
        Serial.print("Total distance = ");
        Serial.print(totalDistance);
        Serial.print(" in");
        Serial.print("Ticks =");
        Serial.print(wheelTicks);
        
        readFlag = 0;
        Serial.print(" Accel Z:");
        Serial.print( sensorValue.un.accelerometer.z*125);
        Serial.print(", ");
        Serial.print(" Accel X:");
        Serial.print(sensorValue.un.accelerometer.x*125);
        Serial.print(", ");
        Serial.print(" Accel Y:");
        Serial.print(sensorValue.un.accelerometer.y*125);
        Serial.print(" Gyro Z:");
        Serial.print(sensorValue.un.gyroscope.z*125);
        Serial.print(", ");
        Serial.print(" Gyro X:");
        Serial.print(sensorValue.un.gyroscope.x*125);
        Serial.print(", ");
        Serial.print(" Gyro Y:");
        Serial.print(sensorValue.un.gyroscope.y*125);
        Serial.print("Lon: ");
        Serial.print(lon);
        Serial.print(GPS.lon);
        Serial.print("Lat: ");
        Serial.print(lat);
        Serial.println(GPS.lat);
        Serial.print("Current (mA): ");
        Serial.print(current_mA);
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.print("Particle Count: ");
        Serial.println(particle_count);
      
      #endif  // putty
        
      interrupts();
        
    } // read flag
      
#endif  // hall
   

  //GPS LOOP
  #if GPS_FLAG
    char c = GPS.read(); //Read the data from the GPS
    if (GPSECHO)
      if (c) //Serial.print(c);
    
    if (GPS.newNMEAreceived()) 
    {
      if (!GPS.parse(GPS.lastNMEA())){} // this also sets the newNMEAreceived() flag to false
        //return; // we can fail to parse a sentence in which case we should just wait for another
    }
    if (millis() - timer > 2000) 
    {
      timer = millis(); // reset the timer

      if (GPS.fix) 
      {
        //#if (GPS_FIX)
          //Serial.print("Fix");
          lat = GPS.latitude;
          lon = GPS.longitude;
          lon_string = GPS.lon;
          lat_string = GPS.lat;
          // for (int i =0l i<GPS.lat.length(); i++){
          //   lat += GPS.lat[i];
          // }
          //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          //Serial.print(", ");
          //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        //#endif
      }
    }
  #endif

  //BNO Loop
  #if BNO
    if(ready_for_bno)
    {
      if (bno08x.wasReset()) 
      {
        //Serial.print("sensor was reset ");
        sh2_setTareNow(SH2_TARE_Z,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
        setReports();
      }
      // test++;
      // if (test > 1000){
      //   test = 0;
      //   //Serial.print("sensor was TARED ");
      //   delay(1000);
      //   sh2_setTareNow(SH2_TARE_Z,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
      //   sh2_setTareNow(SH2_TARE_X,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
      //   sh2_setTareNow(SH2_TARE_Y,SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
      // }

      if (! bno08x.getSensorEvent(&sensorValue)) 
      {
        return;
      }

      ready_for_bno = true;
    }
  #endif

  //BME readings 
  #if BME
    if (! bme.performReading()) 
    {
      // Serial.println("Failed to perform BME reading :(");
      return;
    }
    temp = (bme.temperature * (9/5)) + 32;    // will need option for f or c
    // total_temp += temp;
    pressure = (bme.pressure / 100.0) * 30;
    // total_pressure += pressure;
    humidity = bme.humidity;
    // total_hum += humidity;
  #endif

  //particle count
  #if PM25AQI 
    PM25_AQI_Data data;
    if (! aqi.read(&data)) 
    {    
      // Serial.println("Could not read from AQI");
      delay(500);  // try again in a bit!
      return;
    }
    particle_count = data.particles_03um + data.particles_05um + data.particles_10um;
    total_count += particle_count;
  #endif

  #if INA
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
  #endif

  // if data needs to be sent to the tablet, do it here
  if(station_distance > 10 || create_station){  //recorded 25 feet or user made a station
    if(lat_string == 'S') {
      bt_stationData[0] = -1 * lat;
    }  
    else {
      bt_stationData[0] = -1 * lat;
    }

    if(lon_string == 'W') {
      bt_stationData[1] = -1 * lon;
    }
    else {
      bt_stationData[1] = lon;
    }

    bt_stationData[2] = sensorValue.un.accelerometer.x*180; // need to adjust to do whatever math necessary to convert from ADU to percentage
    bt_stationData[3] = sensorValue.un.accelerometer.y*180;
    bt_stationData[4] = totalDistance;
    bt_stationData[5] = temp;
    bt_stationData[6] = pressure;
    bt_stationData[7] = humidity; 
    bt_stationData[8] = total_count;
    // bt_stationData[9] = /* not sure what is needed here for battery life info */

    // this array will be sent to the tablet as data packet #1
    // BLUETOOTH CODE HERE
    // Onse service with several characteristics will make it simpler to transmit all sensor data in different packets
    // 

    // set values for all characteristics
    // String gps_data = String(bt_stationData[0], 4) + String(lat_string) + "," +
    //                   String(bt_stationData[1], 4) + String(lon_string);
    // pCharacteristic_GPS->setValue(gps_data.c_str()); //-> string part
    // pCharacteristic_GPS->notify(); // Send the value to the app!
    // Serial.print("*** Sent GPS Data Value: ");
    // Serial.print(gps_data);
    // Serial.println(" ***");

    String acc_data = String(bt_stationData[2], 2) + "," +
                      String(bt_stationData[3], 2);
    pCharacteristic_ACC->setValue(acc_data.c_str());
    pCharacteristic_ACC->notify(); // Send the value to the app!
    Serial.print("*** Sent Accelerometer Data Value: ");
    Serial.print(acc_data);
    Serial.println(" ***");

    String dist_data = String(bt_stationData[4], 4);
    pCharacteristic_DIST->setValue(dist_data.c_str());
    pCharacteristic_DIST->notify(); // Send the value to the app!
    Serial.print("*** Sent Wheel Encoder Data Value: ");
    Serial.print(dist_data);
    Serial.println(" ***");

    String env_data = String(bt_stationData[5], 2) + "," + // temp
                      String(bt_stationData[6], 2) + "," + // pressure
                      String(bt_stationData[7], 2) + "," + // humidity
                      String(bt_stationData[8], 2);        // particle count
    pCharacteristic_ENV->setValue(env_data.c_str());
    pCharacteristic_ENV->notify(); // Send the value to the app!
    Serial.print("*** Sent Wheel Encoder Data Value: ");
    Serial.print(dist_data);
    Serial.println(" ***");

    station_distance = 0;
    create_station = false;
  }
}
