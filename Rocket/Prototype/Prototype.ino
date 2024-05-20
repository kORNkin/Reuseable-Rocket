#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_LIS3MDL.h>
#include <ESP32Servo.h>

#include <Wire.h>
#include <Adafruit_BMP280.h>

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Replace with your network credentials
const char* ssid     = "kORNkin";
const char* password = "11111111";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

#define I2C_SDA 18
#define I2C_SCL 19

#define SEALEVELPRESSURE_HPA (1013.25)

float altRef;
float Temp;
float pressureNow;
unsigned int tnow, tlaunch;
unsigned int tset = 0;
float altPrev = 0;
float altNow;
float trend;
int check;

unsigned long delayTime;

Adafruit_BMP280 bmp;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds;

/// Filter Algorithm
Adafruit_NXPSensorFusion filter; // slowest and best
//Adafruit_Madgwick filter;  // faster than NXP , greater than Mahony
//Adafruit_Mahony filter;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 200
#define PRINT_EVERY_N_UPDATES 10

float roll, pitch, yaw;
float gx, gy, gz;
float ax, ay, az;
static uint8_t counter = 0;

sensors_event_t accel, gyro, mag;

uint32_t timestamp;

//------- PID ------- 

#include <PID_v1.h>

//Define Variables For PID (pitch roll yaw)
double pSetpoint, pInput, pOutput;
double rSetpoint, rInput, rOutput;
double ySetpoint, yInput, yOutput;

///Tuning Parameters (Defualt) 
// double aggKp=4, aggKi=0.2, aggKd=1;
// double consKp=1, consKi=0.05, consKd=0.25;

//Tuning Parameters (Tuning)
double aggKp=4, aggKi=0, aggKd=0.5;
double consKp=1, consKi=0, consKd=0.01;

//Specify the links and initial tuning parameters
PID pPID(&pInput, &pOutput, &pSetpoint, consKp, consKi, consKd, DIRECT);
PID rPID(&rInput, &rOutput, &rSetpoint, consKp, consKi, consKd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, consKp, consKi, consKd, DIRECT);

//Servo Fin
#include <ESP32Servo.h>

Servo fin[5];

bool DEBUG_YAW = 0;

int state = 1;

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SPI_SCK 14
#define SPI_MISO 12
#define SPI_MOSI 13
#define SD_CS 2

SPIClass spi1;

char charVal[10];  

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        //Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        //Serial.println("Message appended");
    } else {
        //Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

void SetupIMU_PID(){
    //IMU Part
  if (!lsm6ds.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lsm6ds.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  filter.begin(FILTER_UPDATE_RATE_HZ);

  //PID Setpoints for directions of Rocket
  pSetpoint = 0;
  rSetpoint = 0;
  ySetpoint = 0;

  //Turn the PID on and set range of output 
  pPID.SetMode(AUTOMATIC);
  pPID.SetOutputLimits(-45, 45);

  rPID.SetMode(AUTOMATIC);
  rPID.SetOutputLimits(-45, 45);

  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-45, 45);
}

void SetupSdcard(){
    if(!SD.begin(2)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  writeFile(SD, "/MPU.txt", "yaw pitch roll\n");
}

bool HeadIsOK(double yaw){
  int gapDeg = abs(yaw - ySetpoint);
  if(gapDeg < 10){
    return true;
  }
  return false;
}

// Compute yaw value for useable to pid's error parameter 
double UseableYaw(double yaw){
  if(yaw > 180) return yaw - 360;
  return yaw; 
}

void YawControl(){
  GapCheck(yPID, ySetpoint, yInput, 20);

  yPID.Compute();
  
  Serial.print(", ");
  Serial.println(yOutput);

  //yaw controlling fin
  fin[1].write( 90 + yOutput );
  fin[2].write( 90 + yOutput );
  fin[3].write( 90 + yOutput );
  fin[4].write( 90 + yOutput );
}

void RowPitchControl(){
  //Gap check for using aggressive or constant parameters 
  GapCheck(pPID, pSetpoint, pInput, 10);
  GapCheck(rPID, rSetpoint, rInput, 10);

  pPID.Compute();
  rPID.Compute();
  
  Serial.print(", ");
  Serial.print(pOutput);
  Serial.print(", ");
  Serial.println(rOutput);

  /// Control fin
  // pitch controlling fin 
  fin[1].write( 90 - pOutput );
  fin[2].write( 90 + pOutput );
  // roll controlling fin 
  fin[3].write( 90 - rOutput );
  fin[4].write( 90 + rOutput );
}

void GapCheck(PID &pid, double Setpoint, double Input, double limit){
  double gap = abs(Setpoint - Input);
  if(gap < limit)
  {  //we're close to setpoint, use conservative tuning parameters
    pid.SetTunings(consKp, consKi, consKd);
    //Serial.print(" con ");
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
    pid.SetTunings(aggKp, aggKi, aggKd);
    //Serial.print(" agg ");
  }
  
}

float SetRefAltitude(void) {
  float ref;
  ref = ReadAltitude(SEALEVELPRESSURE_HPA);
  return ref;
}

float ReadAltitude(float seaLevelhPa) {
  float Altitude;

  float pressure = bmp.readPressure(); // in Si units for Pascal
  pressure /= 100;
  pressure = ((float)((int)(pressure * 10))) / 10;

  Altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return Altitude;
}

void IMURun(){
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;

  filter.update(gx, gy, gz, 
                ax, ay, az, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  
  // Delay time between printing              
  // if (counter++ <= PRINT_EVERY_N_UPDATES) {
  //   return;
  // }
  // counter = 0;
  

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  
  //yaw = UseableYaw(yaw);

  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);

  pInput = pitch;
  rInput = roll;
  yInput = yaw;

  appendFile(SD, "/MPU.txt", dtostrf(yaw, 4, 3, charVal));
  appendFile(SD, "/MPU.txt", ", ");
  appendFile(SD, "/MPU.txt", dtostrf(pitch, 4, 3, charVal));
  appendFile(SD, "/MPU.txt", ", ");
  appendFile(SD, "/MPU.txt", dtostrf(roll, 4, 3, charVal));
  appendFile(SD, "/MPU.txt", "\n");
}

void Ejection(){
  // -------------
  ///code here !!!
  // -------------
}

void PreLaunch(){
  if (tnow > tset) {
    Temp = bmp.readTemperature();
    Temp = ((float)((int)(Temp * 10))) / 10;
    pressureNow = bmp.readPressure() / 100.0F;
    pressureNow = ((float)((int)(pressureNow * 10))) / 10;
    altNow = ReadAltitude(SEALEVELPRESSURE_HPA) - altRef;
    trend = altNow - altPrev;
    tset = tnow + 300;
    altPrev = altNow;
    if(trend > 1)
      check++;
    else check--;  

    Serial.print(tnow);
    Serial.println(" checkLunch");
  }

  tlaunch = tnow;
  if(check >= 5){
    Serial.println("Launch!!!");
    state = 2;
    
    check = 0;
  }
  Serial.print(", ");
  Serial.print(altNow);
}      

void EjectingPreparationState(){
  if (tnow > tset) {
    Temp = bmp.readTemperature();
    Temp = ((float)((int)(Temp * 10))) / 10;
    pressureNow = bmp.readPressure() / 100.0F;
    pressureNow = ((float)((int)(pressureNow * 10))) / 10;
    altNow = ReadAltitude(SEALEVELPRESSURE_HPA) - altRef;
    trend = altNow - altPrev;
    tset = tnow + 200;
    altPrev = altNow;
    if(trend < 2)
      check++;
    else check--;  
    
    Serial.print(tnow);
    Serial.println(" checkEject");
  }


  if(check >= 5){
    Serial.println("Eject");
    Ejection();
    state = 3;
    check = 0;
  }
  Serial.print(", ");
  Serial.print(altNow);
}



void EjectionRedundency(){
  unsigned int tair = tnow - tlaunch;
  if(tair > 8 && tair < 9){
    Serial.println("Eject (rdd)");
    Ejection();
    state = 3;
  }
}

void NavigationState(){

}

void setup() {

  Serial.begin(9600);

  state = 1;

  /// FIN Pin Servo 
  // pitch controling fin
  fin[1].attach(25); 
  fin[2].attach(33);
  // roll controling fin
  fin[3].attach(32);  
  fin[4].attach(13); 

  SetupIMU_PID();
  SetupSdcard();

//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   // Print local IP address and start web server
//   Serial.println("");
//   Serial.println("WiFi connected.");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());

// // Initialize a NTPClient to get time
//   timeClient.begin();
//   // Set offset time in seconds to adjust for your timezone, for example:
//   // GMT +1 = 3600
//   // GMT +8 = 28800
//   // GMT -1 = -3600
//   // GMT 0 = 0
//   timeClient.setTimeOffset(25200);


  bool status;
  status = bmp.begin(0x76, 0x60);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  altRef = SetRefAltitude();
}

void loop() {
  IMURun();

  //------------ DEBUG YAW --------------
  // if(Serial.available()){
  
  //   String in = Serial.readStringUntil('\n');
  //   // if(in.toInt() == 1) Serial.println("--------- Yaw activate / ---------");
  //   // else Serial.println("--------- Yaw deactivate X ---------");
  //   // DEBUG_YAW = in.toInt();
  
  //   state = in.toInt();
  // }
  //-------------------------------------
  
  tnow = millis();
  
  if(state == 1 ){ // Pre Launch State
    Serial.println(" Pre Launch State");
    PreLaunch();
  }else if(state == 2) { //Ejecting Preparation State
    Serial.println("Ejecting Preparation State");
    EjectingPreparationState();
    EjectionRedundency();
  }else if(state == 3) { //Navigation State

  }
}



