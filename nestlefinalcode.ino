#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEServer.h>
#include <BLEAdvertisedDevice.h>
#include <BLEBeacon.h>
#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <ESP32WebServer.h>
#include <EEPROM.h>
#include "RTClib.h"
#include <string>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

//#define BUZ_PIN 4
//#define LED_1 13
//#define LED_2 12
//#define POWER_LED 14
//#define LED_4 27
//#define REG_PIN 2
//#define IGN_PIN 35
#define GPIO_pin 27
#define BUZ_PIN 4
#define LED_1 33
#define LED_2 26
#define POWER_LED 32
#define LED_4 25
#define REG_PIN 34
#define IGN_PIN 35
#define SERVICE_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"  //operation modes
#define CHARACTERISTIC_UUID_D   "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // distance 0-50
#define CHARACTERISTIC_UUID_DN  "beb5483e-36e1-4688-b7f5-ea07361b26b0"  //device name
#define CHARACTERISTIC_UUID_C   "beb5483e-36e1-4688-b7f5-ea07361b26b1"  // collision value threshold 0-1000
#define CHARACTERISTIC_UUID_DCC "beb5483e-36e1-4688-b7f5-ea07361b26b2" // duration
Adafruit_MPU6050 mpu;

#define SDA_2 17
#define SCL_2 16


bool str_to_uint16(const char *tempstr, uint16_t *res);
bool rtc_check(DateTime rtc_time);
char* ssid;
char* password;
uint16_t port;
char* hostserver;
int datetime_show = 0;
int file_downlaod = 0;
int buzzer_use = 0;
BLEAdvertising *pAdvertising;
BLEScan *pBLEScan;
File myFile;
ESP32WebServer server(80);
RTC_DS3231 rtc;
BLECharacteristic *pCharacteristic, *pCharacteristicD, *pCharacteristicDN, *pCharacteristicC, *pCharacteristicDCC;
WiFiClient client;
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
String dev_name, driver, distance_str, ble_state_str, collision_str, col_cord_str, report_date = "25-11-2021", report_time = "11-29-45", tcp_reply;
int operation_state, distance_val, collision_val,collision_val_dur, alert_type;
bool col_x_bool = false, col_y_bool = false, col_z_bool = false, driver_reg = false;
bool collision = false;
int col_x_val = 0, col_y_val = 0, col_z_val = 0;
volatile int play_buzzer;
String ssid_str = "", password_str = "", server_str = "", port_str = "";
bool ssidread = false, passread = false, serverread = false, portread = false, nameread = false, driverread = false, operationread = false, distanceread = false, collisionread = false, collisionsecondread = false;

//void IRAM_ATTR Ext_INT1_ISR()
//{
//  //  taskENTER_CRITICAL(&myMutex);
//  Serial.println("Activity Detected");
//  collision = true;
//  //  taskEXIT_CRITICAL(&myMutex);
//}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      if (advertisedDevice.haveName())
      {
        String device_name = advertisedDevice.getName().c_str();
        if (device_name.startsWith("yunjia_Co.,Ltd") || device_name.startsWith("ITECK")) {
          int cur_rssi = advertisedDevice.getRSSI();
          float cacl_value = (-69 - (float)(cur_rssi)) / 20;
          float distance = pow(10, cacl_value);
          Serial.println("Distance: " + String(distance));
          //          Serial.println(String(cur_rssi) + " " + String(cacl_value) + " " + String(distance));
          device_name.trim();
          driver.trim();
          if ((driver != "NO DRIVER" && device_name != driver) && (distance <= distance_val) && !driver_reg) {
            Serial.println("Violation " + device_name);
            alert_type = 0;
            myFile = SD.open("/report.txt", FILE_APPEND);
            if (myFile) {
              report_date.trim();
              report_time.trim();
              device_name.trim();
              DateTime rtc_time = rtc.now();
              if (rtc_check(rtc_time))
              {
                rtc_time = rtc.now();
                delay(10);
              } 
              Serial.println("*" + rtc_time.timestamp(DateTime::TIMESTAMP_FULL) + "," + String(alert_type) + "," + device_name + ",1,1,1,1,1#");
              myFile.print("*" + rtc_time.timestamp(DateTime::TIMESTAMP_FULL) + "," + String(alert_type) + "," + device_name + ",1,1,1,1,1#");
              myFile.close();
            }
            Serial.println("name saved");
            play_buzzer = 1;
          } else if (driver_reg && distance <= 1) {
            Serial.println("Driver registration");
            myFile = SD.open("/driver.txt", FILE_WRITE);
            if (myFile) {
              myFile.println(device_name);
              myFile.close();
              digitalWrite(LED_1, LOW);
              digitalWrite(LED_2, LOW);
              driver = device_name;
              driver_reg = false;
            }
          } else {
            Serial.println("No Violation");
          }
        }
      }
    }
};
void setup() {
  pinMode(GPIO_pin, INPUT_PULLUP);
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(POWER_LED, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(REG_PIN, INPUT);
  pinMode(IGN_PIN, INPUT);
  pinMode(5, OUTPUT);
  
  
  digitalWrite(POWER_LED, HIGH);
  digitalWrite(LED_1, LOW);
   digitalWrite(LED_2, LOW);
  digitalWrite(IGN_PIN, LOW);
  digitalWrite(REG_PIN, LOW);
  digitalWrite(BUZ_PIN, LOW);
  digitalWrite(LED_4, LOW);
  Serial.begin(115200);
  disableCore0WDT();
  disableCore1WDT();
  delay(100);
    Wire1.begin(SDA_2, SCL_2);
  while (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    delay(1000);
  }
     bool status2 = mpu.begin(0x68, &Wire1);  
  if (!status2) {
    Serial.println("Could not find  check wiring!");
    while (1);
  }
   
  
  while (!SD.begin(5)) {
    Serial.println("initialization failed. Things to check:");
    delay(1000);
  }

  sdcardcheck();

  Serial.println("Wiring is correct and a card is present.");
  myFile = SD.open("/ssid.txt", FILE_READ);
  dev_name = "";
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        dev_name += character;
    }
    myFile.close();
  }
  ssid = (char*)dev_name.c_str();
  Serial.println("SSID: " + String(ssid));
  myFile = SD.open("/password.txt", FILE_READ);
  dev_name = "";
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        dev_name += character;
    }
    myFile.close();
  }
  password = (char*)dev_name.c_str();
  Serial.println("Password: " + String(password));
  myFile = SD.open("/server.txt", FILE_READ);
  dev_name = "";
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        dev_name += character;
    }
    myFile.close();
  }
  hostserver = (char*)dev_name.c_str();
  //host = dev_name.c_str();
  Serial.println("Host: " + String(hostserver));
  myFile = SD.open("/port.txt", FILE_READ);
  dev_name = "";
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        dev_name += character;
    }
    myFile.close();
  }
  str_to_uint16(dev_name.c_str(), &port);
  Serial.println("Port: " + String(port));
  myFile = SD.open("/dev_name.txt", FILE_READ);
  dev_name = "";
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        dev_name += character;
    }
    myFile.close();
  }
  Serial.println("Device Name: " + dev_name);
  myFile = SD.open("/driver.txt", FILE_READ);
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        driver += character;
    }
    myFile.close();
  }
  if (driver == "NO DRIVER") {
    digitalWrite(LED_2, HIGH);
  }
  Serial.println("Driver Name: " + driver);
  myFile = SD.open("/operation_state.txt", FILE_READ);
  if (myFile) {
    char character;
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        ble_state_str += character;
    }
    myFile.close();
  }
  operation_state = ble_state_str.toInt();
  Serial.println(String(operation_state));
  myFile = SD.open("/distance.txt", FILE_READ);
  if (myFile) {
    char character;
    distance_str = "";
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        distance_str += character;
    }
    myFile.close();
  }
  distance_val = distance_str.toInt();
  myFile = SD.open("/collision.txt", FILE_READ);
  if (myFile) {
    char character;
    distance_str = "";
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        collision_str += character;
    }
    myFile.close();
  }
  collision_val = collision_str.toInt();
  myFile = SD.open("/collision_cord.txt", FILE_READ);
  if (myFile) {
    char character;
    col_cord_str = "";
    while (character != '\n') {
      character = myFile.read();
      if (character != '\n')
        col_cord_str += character;
    }
    myFile.close();
  }
  collision_val_dur=col_cord_str.toInt();
  

  //  accel.setRange(ADXL345_RANGE_16_G);
  
  //  determine_col_cord(col_cord_str);
  if (operation_state == 0) {
    pinMode(GPIO_pin, INPUT_PULLUP);
    delay(10);
    if (!digitalRead(GPIO_pin)) {
      Serial.println("Activity Detected");
      collision = true;
    }
    digitalWrite(LED_1, LOW);
    char dev_name_buf[dev_name.length()];
    dev_name.toCharArray(dev_name_buf, dev_name.length());
    BLEDevice::init(dev_name_buf);
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pCharacteristic->setValue("0");
    pCharacteristicD = pService->createCharacteristic(
                         CHARACTERISTIC_UUID_D,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE
                       );
    char str_val[10];
    itoa(distance_val, str_val, 10);
    pCharacteristicD->setValue(str_val);
    pCharacteristicDN = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_DN,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_WRITE
                        );
    pCharacteristicDN->setValue(dev_name_buf);
    char collision_buf[collision_str.length()];
    collision_str.toCharArray(collision_buf, collision_str.length());
    pCharacteristicC = pService->createCharacteristic(
                         CHARACTERISTIC_UUID_C,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE
                       );
    pCharacteristicC->setValue(collision_buf);
    char collision_cord_buf[col_cord_str.length()];
    col_cord_str.toCharArray(collision_cord_buf, col_cord_str.length());
    pCharacteristicDCC = pService->createCharacteristic(
                           CHARACTERISTIC_UUID_DCC,
                           BLECharacteristic::PROPERTY_READ |
                           BLECharacteristic::PROPERTY_WRITE
                         );
    pCharacteristicDCC->setValue(collision_cord_buf);
    pAdvertising = BLEDevice::getAdvertising();
    pService->start();
    BLEBeacon oBeacon = BLEBeacon();
    pAdvertising->start();
    Serial.println("Advertizing started...");
//    pinMode(GPIO_pin, INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(GPIO_pin), Ext_INT1_ISR, FALLING);
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99); // less or equal setInterval value
    if (!digitalRead(GPIO_pin)) {
      Serial.println("Activity Detected");
      collision = true;
    }
    Serial.println(collision_val);
    Serial.println(collision_val_dur);
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
   mpu.setMotionDetectionThreshold(collision_val);
   mpu.setMotionDetectionDuration(collision_val_dur);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  
  Serial.println("");
  delay(100);
//    int collision_val_int=pCharacteristicC->getValue().toInt();
//  
//    int col_cord_str=pCharacteristicDCC->getValue().toInt();
    
    
    xTaskCreate(&ble_scan, "ble_scan", 5012, NULL, 5, NULL);
    xTaskCreate(&collision_detect, "collision_detect", 6036, NULL, 5, NULL);

    xTaskCreate(&buzzer_alarm, "buzzer_alarm", 5012, NULL, 5, NULL);
       
    
  } else if (operation_state == 1) {
    direct_wifi_mode();
    xTaskCreate(&wifi_function, "wifi_function", 6036, NULL, 5, NULL);
    xTaskCreate(&wifi_function_led, "wifi_function_led", 6036, NULL, 5, NULL);
  } else if (operation_state == 2) {
    digitalWrite(LED_1, HIGH);
    WiFi.begin("NFM-ITECK", "19718842");
    //WiFi.begin("itecknologi", "Iteck#@#7831");
    int wifi_connection_count = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("...");
      if (analogRead(IGN_PIN) > 0) {
        myFile = SD.open("/operation_state.txt", FILE_WRITE);
        if (myFile) {
          myFile.println("0");
          myFile.close();
          digitalWrite(LED_4, LOW);
          ESP.restart();
        }
      }
      wifi_connection_count++;
      if (wifi_connection_count == 30) {
        ESP.restart();
      }
    }
    wifi_connection_count = 0;
    Serial.print("WiFi connected with IP: ");
    Serial.println(WiFi.localIP());
    while (!client.connect("192.168.100.2", 12345)) {
    //while (!client.connect(hostserver, 12345)) {
      Serial.println("Connection to host failed");
      if (analogRead(IGN_PIN) > 0) {
        myFile = SD.open("/operation_state.txt", FILE_WRITE);
        if (myFile) {
          myFile.println("0");
          myFile.close();
          digitalWrite(LED_4, LOW);
          ESP.restart();
        }
      }
      delay(1000);
    }
    Serial.println("Connected to server successful!");
    xTaskCreate(&wifi_tcp_connection_read, "wifi_tcp_connection_read", 5012, NULL, 5, NULL);
    xTaskCreate(&wifi_tcp_connection, "wifi_tcp_connection", 5012, NULL, 5, NULL);
  }
//  xTaskCreate(&mpu6, "mpu6", 5012, NULL, 6, NULL); 



}
void loop() {
 
   }
