
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu;

#define I2C_SDA 21
#define I2C_SCL 22

double AccX, AccY, AccZ;
double GyroX, GyroY, GyroZ;
double compAngleX, compAngleY, compAngleZ;
double roll, pitch, yaw;
double dt, currentTime, previousTime;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float meanX, meanY;
Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;
double kalAngleX, kalAngleY;

int c = 0;
const int MPU = 0x68;

void setup(void) {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
  calculate_IMU_error();
  delay(1000);
}

void loop() {

  /* Get new sensor events with the readings */
  for (int i = 0; i <= 10; i++) {
    //    Serial.println(i);
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


    GyroX -= GyroErrorX;  // GyroErrorX ~(-0.56)
    GyroY -= GyroErrorY;  // GyroErrorY ~(2)

    double roll = atan2(AccY, AccZ) * RAD_TO_DEG;
    double pitch = atan2(AccX, AccZ) * RAD_TO_DEG;

    compAngleX = 0.97 * (compAngleX + GyroX * dt) + 0.03 * roll;  // Calculate the angle using a Complimentary filter
    compAngleY = 0.97 * (compAngleY + GyroY * dt) + 0.03 * pitch;

    kalAngleX = kalmanX.getAngle(roll, GyroX, dt);
    kalAngleY = kalmanY.getAngle(pitch, GyroY, dt);

    meanX += kalAngleX;
    meanY += kalAngleY;
  }
  meanX /= 10;
  meanY /= 10;


   /* Print out the values */
   Serial.print("AccelX:");
   Serial.print(AccX);
   Serial.print(",");
   Serial.print("AccelY:");
   Serial.print(AccY);
   Serial.print(",");
   Serial.print("AccelZ:");
   Serial.print(AccZ);
   Serial.print(",");
  //  Serial.print("GyroX:");
  //  Serial.print(GyroX);
  //  Serial.print(",");
  //  Serial.print("GyroY:");
  //  Serial.print(GyroY);
  //  Serial.print(",");
  //  Serial.print("GyroZ:");
  //  Serial.print(GyroZ);
  //  Serial.println("");
  //  Serial.print("roll:");
  //  Serial.print(roll);
  //  Serial.print(",");
  //  Serial.print("CompRoll:");
  //  Serial.print(compAngleX);
  //  Serial.print(",");
  //  Serial.print("Kroll:");
  //  Serial.print(kalAngleX);
  //  Serial.print(",");
  
  //  Serial.print("pitch:");
  //  Serial.print(pitch);
  //  Serial.print(",");
  //  Serial.print("CompPitch:");
  //  Serial.print(compAngleY);
  //  Serial.print(",");
  //  Serial.print("Kpitch:");
  //  Serial.print(kalAngleY);
  //  Serial.println("");

  //  Serial.print("GyroX:");
  //  Serial.print(GyroX);
  //  Serial.print(",");
  //  Serial.print("roll:");
  //  Serial.print(roll);
  //  Serial.print(",");
  //  Serial.print("GyroY:");
  //  Serial.print(GyroY);
  //  Serial.println("");
   Serial.print("Kroll:");
   Serial.print(meanX);
   Serial.print(",");
   Serial.print("Kpitch:");
   Serial.print(meanY);
   Serial.print(",");
  //  Serial.print("CompRoll:");
  //  Serial.print(compAngleX);
  //  Serial.print(",");
   Serial.print("dt:");
   Serial.print(dt);
  //  Serial.print(",");
  //  Serial.print("CompPitch:");
  //  Serial.print(compAngleY);
  Serial.println("");
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;
    // Sum all readings
    AccErrorX += AccX;
    AccErrorY += AccY;
    AccErrorZ += AccZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  AccErrorZ = AccErrorZ / 200;
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
  Serial.print("AccErrorZ: ");
  Serial.println(AccErrorZ);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
}
