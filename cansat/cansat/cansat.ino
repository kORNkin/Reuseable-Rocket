// SPI define-----------------------------------------------------------------------
#include "SPI.h"

#define SPI_SCK 5
#define SPI_MISO 19
#define SPI_MOSI 27
#define Select LOW
#define DeSelect HIGH

// LoRa define----------------------------------------------------------------------
#include <LoRa.h>

#define  LoRa_CS    18
#define  LoRa_RST   14
#define  DI0        26
#define  BAND    917E6
#define LoRaStatus 32

SPIClass LoRaSPI;

int counter = 0;
unsigned int startTime;
unsigned int curTime;

//SD card define--------------------------------------------------------------------
#include "FS.h"
#include "SD.h"


#define SD_CS 12

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

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double kalAngleX, kalAngleY;

int c = 0;
const int MPU = 0x68;

//BME280 define---------------------------------------------------------------------
#include <Adafruit_BME280.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SEALEVELPRESSURE_HPA (1013.25)

TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

float RefAltitude;
float Temp;
float ReaPressure;
float ReaAltitude;
float Humid;

//GPS define------------------------------------------------------------------------
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define RXPin 16
#define TXPin 17
#define GPSstatus 33

int GPSBaud = 9600;
float latt, lonn;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup(){

}

void loop(){

}


















