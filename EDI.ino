
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "ThingSpeak.h"
#include <ESP8266WiFi.h>


const uint8_t SCL_PIN = D1;
const uint8_t SDA_PIN = D2;
#define GPS_RX_PIN D6
#define GPS_TX_PIN D7


////Adafruit_MPU6050 mpu;


const uint8_t MPU6050SlaveAddress = 0x68;
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x69;


int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, Temperature, AccelX_prev, GyroX_prev, GyroY_prev, GyroZ_prev, AccelX_diff, GyroX_diff, GyroY_diff, GyroZ_diff;
int check = 0, accouc = 0;
String gmap = "http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=";

float lat, lon;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // create soft serial object
TinyGPS gps; // create gps object

unsigned long ch_no = 1769527;
const char * write_api = "6PRUTWZ9VPX722ZC";
const char* ssid = "SN"; //ssid of your wifi
const char* pass = "sid12345678"; //password of your wifi

int outputpin = A0;
int alertstop = D5;
int alert = 0;
int LED = D3;
int Drowsycount = 0;
int DrowsyLED = D8;

void MPU6050_Init();
int coll_detect();
WiFiClient  client;

void setup(void) {
  Serial.begin(9600); // connect serial
  gpsSerial.begin(9600); // connect gps sensor
  pinMode(GPS_RX_PIN, INPUT);
  pinMode(GPS_TX_PIN, OUTPUT);
  pinMode(alertstop, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(DrowsyLED, OUTPUT);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass); //connecting to wifi
  while (WiFi.status() != WL_CONNECTED)// while wifi not connected
  {
    delay(500);
    Serial.print("."); //print "...."
  }

  Serial.println("WiFi connected");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.write(0x6B);
  Wire.write(0);
  MPU6050_Init();
  ThingSpeak.begin(client);
}


void loop() {
  //  Serial.println(digitalRead(alertstop));
  if (Drowsycount > 100) {
    long Drowsy = ThingSpeak.readIntField(ch_no, 5);
    if (Drowsy == 1) {
      Serial.print(Drowsy);

      digitalWrite(DrowsyLED, HIGH);
      delay(10000);
    }
    else {
      digitalWrite(DrowsyLED, LOW);
    }
    Drowsycount = 0;
  }

  if (accouc >= 2 ) {
    while (gpsSerial.available()) {

      if (gps.encode(gpsSerial.read())) { // encode gps data
        gps.f_get_position(&lat, &lon); // get latitude and longitude
        Serial.print("lat: ");
        Serial.print(lat, 6);
        Serial.print("          ");
        Serial.print("lon: ");
        Serial.println(lon, 6);
        String gmapsend = gmap + lat + "+" + lon;
        ThingSpeak.setField(1, lat);
        ThingSpeak.setField(2, lon);
        ThingSpeak.setField(3, gmapsend);
        ThingSpeak.setField(4, 1);
        ThingSpeak.writeFields(ch_no, write_api);
      }
    }

    Serial.println("Accident!!");


    if (digitalRead(alertstop)) {
      accouc = 0;
    }
  }
  else {
    coll_detect();
    int analogValue = analogRead(outputpin);
    float millivolts = (analogValue / 1024.0) * 3300; //3300 is the voltage provided by NodeMCU
    float celsius = millivolts / 10;
    Serial.print("DegreeC=   ");
    Serial.println(celsius);
    if (celsius > 300) {
      digitalWrite(LED, HIGH);
    }
    else {
      digitalWrite(LED, LOW);
    }
  }
  Drowsycount += 1;
}


int coll_detect()
{

  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(MPU6050_REGISTER_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050SlaveAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());

  //    Serial.println(AccelX);

  if (check == 0)
  {
    AccelX_diff = 0;
    check = 1;
  }
  else
  {
    AccelX_diff = abs(AccelX - AccelX_prev);
    GyroX_diff = abs(GyroX - GyroX_prev);
    GyroY_diff = abs(GyroY - GyroY_prev);
    GyroZ_diff = abs(GyroZ - GyroZ_prev);
    if ( (AccelX_diff > 15000) && ( (GyroX_diff > 300) || (GyroY_diff > 300) || (GyroZ_diff > 300) ) )
    {
      AccelX_diff = AccelX_diff ;
      accouc += 1;
    }
    AccelX_prev = AccelX;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
    return AccelX_diff;
  }
}



void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x00 );
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/- 250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
