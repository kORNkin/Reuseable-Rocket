// SPI define-----------------------------------------------------------------------
#include "SPI.h"

#define SPI_SCK 5
#define SPI_MISO 19
#define SPI_MOSI 27
#define Select LOW
#define DeSelect HIGH

// LoRa define----------------------------------------------------------------------
#include <LoRa.h>

#define LoRa_CS 18
#define LoRa_RST 14
#define DI0 26
#define BAND 917E6
#define LoRaStatus 32

SPIClass LoRaSPI;

int counter = 0;
unsigned int startTime;
unsigned int curTime;
unsigned long readtime;
unsigned int delayread = 200;

//SD card define--------------------------------------------------------------------
#include "FS.h"
#include "SD.h"

#define SD_CS 12
String path;

//MPU6050 define--------------------------------------------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Kalman.h>
Adafruit_MPU6050 mpu;

double AccX, AccY, AccZ;
double GyroX, GyroY, GyroZ;
double roll, pitch;
double dt, currentTime, previousTime;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float meanX, meanY, meanGyroX, meanGyroY, meanAccX, meanAccY;

Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;
double kalAngleX, kalAngleY;

const int MPU = 0x68;

//BME280 define---------------------------------------------------------------------
#include <Adafruit_BME280.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

float RefAltitude;
float Temp;
float ReaPressure;
float ReaAltitude;
float Humid;

//GPS define------------------------------------------------------------------------
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define RXPin 16
#define TXPin 17
#define GPSstatus 33

int GPSBaud = 9600;
float latt = 0;
float lonn = 0;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);


void setup() {
  Serial.begin(115200);
  //LoRa setup----------------------------------------------------------------------
  pinMode(LoRaStatus, OUTPUT);
  digitalWrite(LoRaStatus, HIGH);
  Serial.println("LoRa Sender");
  LoRa.setPins(LoRa_CS, LoRa_RST, DI0);
  digitalWrite(LoRa_CS, Select);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
  } else {
    Serial.println("LoRa Initial OK!");
    delay(1000);
  }
  Serial.println("Setup done!");
  startTime = millis();
  digitalWrite(LoRaStatus, LOW);
  delay(100);
  digitalWrite(LoRaStatus, HIGH);


  //SD setup-----------------------------------------------------------------------
  if (!SD.begin(SD_CS)) {
    Serial.println("Keep going dont give up");
    return;
  } else {
    Serial.println("SD done!");
  }
  SDlogFile();
  digitalWrite(LoRaStatus, LOW);
  delay(100);
  digitalWrite(LoRaStatus, HIGH);
  
//BME280 setup--------------------------------------------------------------------
  if (bme.begin(0x76)) {
    Serial.println("BME begin");
  }
  RefAltitude = SetRefAltitude();

  readtime = millis() + delayread;
  digitalWrite(LoRaStatus, LOW);
  delay(100);
  digitalWrite(LoRaStatus, HIGH);

  //MPU6050 setup---------------------------------------------------------------------
  if (mpu.begin(MPU)) {
    Serial.println("MPU begin");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
  calculate_IMU_error();
  delay(1000);
  digitalWrite(LoRaStatus, LOW);

  //GPS setup-----------------------------------------------------------------------
  gpsSerial.begin(GPSBaud);
  // pinMode(GPSstatus,OUTPUT);
  digitalWrite(GPSstatus,LOW);
}

void loop() {
  if (millis() >= readtime) {
    digitalWrite(LoRaStatus, HIGH);
    delay(10);
    curTime = millis() - startTime;
    Serial.print(counter);
    Serial.print(",");
    Serial.print(curTime);
    Serial.print(",");

    //MPU6050 loop--------------------------------------------------------------------
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    previousTime = currentTime;                // Previous time is stored before the actual time read
    currentTime = millis();                    // Current time actual time read
    dt = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;
    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;

    AccX -= AccErrorX;
    AccY -= AccErrorY;
    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;
    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;

    AccX -= AccErrorX;
    AccY -= AccErrorY;
    GyroX -= GyroErrorX;
    GyroY -= GyroErrorY;

    double roll = atan2(AccY, AccZ) * RAD_TO_DEG;
    double pitch = atan2(AccX, AccZ) * RAD_TO_DEG;

    kalAngleX = kalmanX.getAngle(roll, GyroX, dt);
    kalAngleY = kalmanY.getAngle(pitch, GyroY, dt);

    // Serial.print("AccelX:");
    Serial.print(AccX);
    Serial.print(",");
    // Serial.print("AccelY:");
    Serial.print(AccY);
    Serial.print(",");
    // Serial.print("AccelZ:");
    Serial.print(AccZ);
    Serial.print(",");
    // Serial.print("GyroX:");
    Serial.print(GyroX);
    Serial.print(",");
    // Serial.print("GyroY:");
    Serial.print(GyroY);
    Serial.print(",");
    // Serial.print("Kroll:");
    Serial.print(kalAngleX);
    Serial.print(",");
    // Serial.print("Kpitch:");
    Serial.print(kalAngleY);
    Serial.print(",");
    

    //BME280 loop---------------------------------------------------------------------
    Temp = bme.readTemperature();
    Temp = ((float)((int)(Temp * 10))) / 10;

    ReaPressure = bme.readPressure() / 100.0F;
    ReaPressure = ((float)((int)(ReaPressure * 10))) / 10;

    ReaAltitude = ReadAltitude(SEALEVELPRESSURE_HPA, ReaPressure) - RefAltitude;

    Humid = bme.readHumidity();
    Humid = ((float)((int)(Humid * 10))) / 10;

    Serial.print(Temp);
    Serial.print(",");
    Serial.print(ReaPressure);
    Serial.print(",");
    Serial.print(Humid);
    Serial.print(",");
    Serial.print(ReaAltitude);
    Serial.print(",");

    //GPS loop----------------------------------------------------------------------
    while (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
        if(gps.location.isValid())
        {
          latt = gps.location.lat();
          lonn = gps.location.lng();
          digitalWrite(GPSstatus,HIGH);
        }
      }
    }
    Serial.print(latt, 6);
    Serial.print(",");
    Serial.print(lonn, 6);
    Serial.println("");
    //LoRa loop-----------------------------------------------------------------------
    LoRa.beginPacket();
    LoRa.println(LoRaPacket());
    LoRa.endPacket();
    counter += 1;

    //SD loop-------------------------------------------------------------------------
    SDDataDump(LoRaPacket());

    digitalWrite(LoRaStatus, LOW);
    delay(10);
    readtime = millis() + delayread;
  }
}
//BME280 Function-------------------------------------------------------------------
float SetRefAltitude(void) {
  float ref;
  for (int i = 0; i <= 1000; i++) {
    ref += bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
  ref /= 1000;
  return ref;
}
float ReadAltitude(float seaLevelhPa, float pressure) {

  float Altitude;
  Altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return Altitude;
}

//MPU6050 Function------------------------------------------------------------------
void calculate_IMU_error() {
  int c = 0;
  while (c < 200) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    // Sum all readings
    AccErrorX += AccX;
    AccErrorY += AccY;
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  while (c < 2000) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    c++;
  }
  //Divide the sum by 2000 to get the error value
  GyroErrorX = GyroErrorX / 2000;
  GyroErrorY = GyroErrorY / 2000;
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
}
//LoRa function------------------------------------------------------------------
String LoRaPacket(){
  String packet;
  packet += String(counter);
  packet += String(",");
  packet += String(curTime);
  packet += String(",");
  packet += String(AccX);
  packet += String(",");
  packet += String(AccY);
  packet += String(",");
  packet += String(AccZ);
  packet += String(",");
  packet += String(GyroX);
  packet += String(",");
  packet += String(GyroY);
  packet += String(",");
  packet += String(kalAngleX);
  packet += String(",");
  packet += String(kalAngleY);
  packet += String(",");
  packet += String(Temp);
  packet += String(",");
  packet += String(ReaPressure);
  packet += String(",");
  packet += String(Humid);
  packet += String(",");
  packet += String(ReaAltitude);
  packet += String(",");
  packet += String(latt,6);
  packet += String(",");
  packet += String(lonn,6);
  return packet;
}

//SD function------------------------------------------------------------------
void SDlogFile() {
  int fileN = 0;
  File root = SD.open("/SDDataLog");
  File file = root.openNextFile();
  while(file){
    fileN += 1;
    file = root.openNextFile();
  }
  // Serial.print("total file: "); Serial.println(fileN);
  fileN += 1;
  path = String("/SDDataLog") + String("/Datalog") + String(fileN) + String(".txt");
  Serial.println(path);
  File newfile = SD.open(path,FILE_WRITE);
  newfile.println("CANSAT data log:");
  newfile.close();
}

void SDDataDump(String packet){
  File dump = SD.open(path,FILE_APPEND);
  dump.println(packet);
}

//Gps funtion------------------------------------------------------------------
