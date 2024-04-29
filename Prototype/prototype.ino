#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_LIS3MDL.h>
#include <ESP32Servo.h>
Servo fin1;

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

void setup() {

  Serial.begin(9600);

  //FIN Pin Servo 
  fin[1].attach(15);
  fin[2].attach(2);

  //IMU Prt
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

  //PID Setpoints for Rocket's directions
  pSetpoint = 0;
  rSetpoint = 0;

  //turn the PID on
  pPID.SetMode(AUTOMATIC);
  pPID.SetOutputLimits(-75, 75);

  rPID.SetMode(AUTOMATIC);
  rPID.SetOutputLimits(-75, 75);

}

void loop() {

  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;

  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // Delay time between printing              
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  counter = 0;

  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);

  pInput = pitch;
  rInput = roll;

  //Gap check for using aggressive or constant parameters 
  GapCheck(pPID, pSetpoint, pInput);
  GapCheck(rPID, rSetpoint, rInput);

  pPID.Compute();
  rPID.Compute();
  
  Serial.print(", ");
  Serial.print(pOutput);
  Serial.print(", ");
  Serial.println(rOutput);

  //Controll fin
  fin[1].write( 90 - pOutput );
  fin[2].write( 90 - rOutput );
}

void GapCheck(PID &pid, double Setpoint, double Input){
  double gap = abs(Setpoint - Input);
  if(gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    pid.SetTunings(consKp, consKi, consKd);
    Serial.print(" con ");
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
    pid.SetTunings(aggKp, aggKi, aggKd);
    Serial.print(" agg ");
  }
}
