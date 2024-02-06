#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
float txValue1 = 0; 
float txValue2 = 0; 
float txValue3 = 0; 
float txValue4 = 0;
float txValue5 = 0;
//char rxValue = '\0';
const int readPin = 5  ; // Use GPIO number. See ESP32 board pinouts
const int LED = 48;      // pin of the RGB LED

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"//"2F05023F-F495-4259-8504-CFB307756EAF"// // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"//"B8CE9199-227D-422E-9A96-FD6766014BA8"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"//"DD9E2F15-EE4B-4F96-8231-18ADB1121294"

#define CHUNK_SIZE 20 // Define the chunk size

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

void setup() {
  Serial.begin(115200);
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
  if (deviceConnected) {
    // Fabricate some randow numbers for now...
    float latitude = random(-90.0, 90.0); // Random latitude (-90 to 90 degrees)
    float longitude = random(-180.0, 180.0); // Random longitude (-180 to 180 degrees)
    float altitude = random(0.0, 5000.0); // Random altitude (0 to 5000 meters)
    float grade = random(0.0, 15.0); // Random grade (0 to 15)
    float slope = random(0.0, 10.0); // Random slope (0 to 15)

    String data = String(latitude, 4) + "," +
                  String(longitude, 4) + "," +
                  String(altitude, 2) + "," +
                  String(grade, 2) + "," +
                  String(slope, 2);

    //float data[] = {latitude, longitude, altitude, grade, slope};
    //String chunks[5];

    //for (int i = 0; i < 5; i++) {
      //chunks[i] = String(data[i]); // 2 decimal places
    //}
    // Send data in chunks
    //int dataLength = data.length();
    //int chunkSize = 19; // New chunk size
    //int numChunks = ceil((float)dataLength / chunkSize);

    //for (int i = 0; i < numChunks; i++) {
      //int start = i * chunkSize;
      //int end = min(start + chunkSize, dataLength);

      //String chunk = data.substring(start, end);
      //pCharacteristic->setValue(chunk.c_str());
      //pCharacteristic->notify();
      //delay(500); // Add a small delay between sending chunks
    
      //Serial.print("*** Sent Value: ");
      //Serial.print(chunk);
      //Serial.println(" ***");
    //}

      
  //}
   // Serial.print("*** Sent Value: ");
   // Serial.print();
    //Serial.println(" ***");
  //delay(1000);
//}

    /*
    txValue = analogRead(readPin) + 1;
    txValue1 = random(1, 10); 
    txValue2 = random(10, 20);
    txValue3 = random(20, 30);
    txValue4 = random(30, 40);
    txValue5 = random(40, 50);

    float data = {txValue, txValue1,txValue2,txValue3,txValue4,txValue5};
    String chunks[6];
    for (int i = 0; i < 6; i++) {
      chunks[i] = String(data[i]);
    }
    */
// To differenciate between which sensor we can have a comment up here or something
    //String data = String(txValue) + "," + String(txValue1) + "," + String(txValue2)+ "," + String(txValue3)+
     //"," + String(txValue4)+ "," + String(txValue5); // string part

   /* int dataLength = data.length();
    int numChunks = ceil((float)dataLength / CHUNK_SIZE);

    for (int i = 0; i < numChunks; i++) {
      int start = i * CHUNK_SIZE;
      int end = min(start + CHUNK_SIZE, dataLength);

      String chunk = data.substring(start, end);
      
      pCharacteristic->setValue(chunks[0].c_str());
      pCharacteristic->notify();
      Serial.print("*** Sent Value: ");
      Serial.print(chunks[0]);
      Serial.println(" ***");

      delay(500); // Add a small delay between sending chunks
    }
  
  delay(1000);
} 
*/
    // Let's convert the value to a char array:
    //char txString[8];
    //dtostrf(data, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer

    pCharacteristic->setValue(data.c_str()); //-> string part
     
    pCharacteristic->notify(); // Send the value to the app!
    Serial.print("*** Sent Value: ");
    Serial.print(data);
    Serial.println(" ***");
  }
  delay(1000);

}


