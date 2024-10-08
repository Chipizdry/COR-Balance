#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define LED_PIN 8
// UART settings
#define RXD1 9  // Change this pin if needed
#define TXD1 10 // Change this pin if needed

// Define PWM properties
const int freq = 5000;     // PWM frequency
const int pwmChannel1 = 0; // PWM channel for first output
const int pwmChannel2 = 1; // PWM channel for second output
const int pwmChannel3 = 2; // PWM channel for third output
const int resolution = 8;  // PWM resolution (8 bits)

int reicive[16]={0, };
BLECharacteristic *pCharacteristic = NULL;
std::string msg;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
       
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        // Convert received value to integer and set it as PWM duty cycle
        int pwmValue = atoi(value.c_str());
        ledcWrite(pwmChannel1, pwmValue);
        ledcWrite(pwmChannel2, pwmValue);
        ledcWrite(pwmChannel3, pwmValue);
      }
    }
};

void setup() {
  Serial.begin(115200);
    delay(100);
   Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  delay(100);
  Serial.println("1- Download and install an BLE Terminal Free");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to ESP32BLE");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");

// Включение командного режима
  uint8_t commandMode[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  Serial1.write(commandMode, sizeof(commandMode));
  delay(100);

  // Остановка автоматической передачи данных
  uint8_t stopAutoSend[] = {0xFF, 0xAA, 0x02, 0x00};
  Serial1.write(stopAutoSend, sizeof(stopAutoSend));
  delay(100);
   uint8_t SaveSettings[] = {0XFF, 0XAA, 0X00, 0X00, 0X00};

    Serial1.write(SaveSettings, sizeof(SaveSettings));
  delay(100);
  // Initialize BLE
  BLEDevice::init("COR-Balance_stick");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("COR-Balance");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();

  Serial.print("Server address:");
  Serial.println(BLEDevice::getAddress().toString().c_str());

  // Configure PWM functionalities
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcSetup(pwmChannel3, freq, resolution);

  // Attach the channels to GPIO pins
  ledcAttachPin(2, pwmChannel1);  // Change GPIO pin as needed
  ledcAttachPin(3, pwmChannel2);  // Change GPIO pin as needed
  ledcAttachPin(4, pwmChannel3);  // Change GPIO pin as needed
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
 // readSerialPort();

   uint8_t requestData[] = {0xFF, 0xAA, 0x27, 0x00};
   Serial1.write(requestData, sizeof(requestData));
   digitalWrite(LED_PIN, HIGH);
   delay(10);
  readSensorData();
   digitalWrite(LED_PIN, LOW);
  //Send data to slave
//  if(msg!=""){
 //   pCharacteristic->setValue(msg);
  //  msg="";
 // }
       // ledcWrite(pwmChannel1, 200);
       // ledcWrite(pwmChannel2, 200);
      //  ledcWrite(pwmChannel3, 200);
  delay(10);
}

void readSerialPort(){
 while (Serial.available()) {
   delay(10);  
   if (Serial.available() >0) {
     char c = Serial.read();  //gets one byte from serial buffer
     msg += c; //add to String
   }
 }
 Serial.flush(); //clean buffer
}


void readSensorData(){
  if (Serial1.available()) {
    uint8_t header = Serial1.read();
   
    if (header == 0x55) {
      uint8_t data_type = Serial1.read();
      uint8_t buffer[8];
      Serial1.readBytes(buffer, 8);
      uint8_t checksum = Serial1.read();
      uint8_t calculated_checksum = header + data_type;
      for (int i = 0; i < 8; i++) {
        calculated_checksum += buffer[i];
      }
      if (calculated_checksum == checksum) {
        switch (data_type) {
          case 0x51: // Acceleration data
            handleAccelerationData(buffer);
            break;
         // case 0x52: // Angular velocity data
        //    handleAngularVelocityData(buffer);
         //   break;
          case 0x53: // Angle data
            handleAngleData(buffer);
            break;
        }
      }
    }
  }
}

void handleAccelerationData(uint8_t *buffer) {
  int16_t ax = (buffer[1] << 8) | buffer[0];
  int16_t ay = (buffer[3] << 8) | buffer[2];
  int16_t az = (buffer[5] << 8) | buffer[4];
  float accelerationX = ax / 32768.0 * 16;
  float accelerationY = ay / 32768.0 * 16;
  float accelerationZ = az / 32768.0 * 16;
//  Serial.printf("Acceleration X: %.2f, Y: %.2f, Z: %.2f\n", accelerationX, accelerationY, accelerationZ);
  // Adjust PWM based on acceleration
  ledcWrite(pwmChannel1, map(abs(ax), 0, 32768, 0, 255));
  
   // Собираем строку с данными
    char accData[64];
    snprintf(accData, sizeof(accData), "Acc X: %.2f, Y: %.2f, Z: %.2f", accelerationX, accelerationY, accelerationZ);

    // Отправляем данные по BLE
    pCharacteristic->setValue(accData);
    pCharacteristic->notify(); 
}

void handleAngularVelocityData(uint8_t *buffer) {
  int16_t wx = (buffer[1] << 8) | buffer[0];
  int16_t wy = (buffer[3] << 8) | buffer[2];
  int16_t wz = (buffer[5] << 8) | buffer[4];
  float angularVelocityX = wx / 32768.0 * 2000;
  float angularVelocityY = wy / 32768.0 * 2000;
  float angularVelocityZ = wz / 32768.0 * 2000;
  Serial.printf("Angular Velocity X: %.2f, Y: %.2f, Z: %.2f\n", angularVelocityX, angularVelocityY, angularVelocityZ);
}

void handleAngleData(uint8_t *buffer) {
  int16_t roll = (buffer[1] << 8) | buffer[0];
  int16_t pitch = (buffer[3] << 8) | buffer[2];
  int16_t yaw = (buffer[5] << 8) | buffer[4];
  float rollAngle = roll / 32768.0 * 180;
  float pitchAngle = pitch / 32768.0 * 180;
  float yawAngle = yaw / 32768.0 * 180;
 // Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", rollAngle, pitchAngle, yawAngle);
  // Adjust PWM based on angle
  ledcWrite(pwmChannel2, map(abs(roll), 0, 32768, 0, 255));
  ledcWrite(pwmChannel3, map(abs(pitch), 0, 32768, 0, 255));

     // Собираем строку с данными
    char angleData[64];
    snprintf(angleData, sizeof(angleData), "Roll: %.2f, Pitch: %.2f", rollAngle, pitchAngle);

    // Отправляем данные по BLE
    pCharacteristic->setValue(angleData);
    pCharacteristic->notify();
}
