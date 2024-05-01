#include <IMUGY85.h>

IMUGY85 imu;
double ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw;

uint32_t timestamp;

void setup()
{
  Serial.begin(9600);
  imu.init();
}

void loop()
{
  imu.update();
  
  printAccel();
  printGyro();
  printMagneto();
  // printRollPitchYaw();
  
  delay(10);
}

void printAccel()
{
  imu.getAcceleration(&ax, &ay, &az);
  Serial.print(ax);Serial.print("\t");
  Serial.print(ay);Serial.print("\t");
  Serial.print(az);Serial.print("\t");
  Serial.println();
}

void printGyro()
{
  imu.getGyro(&gx, &gy, &gz);
  Serial.print(gx);Serial.print("\t");
  Serial.print(gy);Serial.print("\t");
  Serial.print(gz);Serial.print("\t");
  Serial.println();
}

void printMagneto()
{
  imu.getMagneto(&mx, &my, &mz);
  Serial.print(mx);Serial.print("\t");
  Serial.print(my);Serial.print("\t");
  Serial.print(mz);Serial.print("\t");
  Serial.println();
}

void printRollPitchYaw()
{
  roll = imu.getRoll();
  pitch = imu.getPitch();
  yaw = imu.getYaw();
  Serial.print(pitch);Serial.print("\t");
  Serial.print(roll);Serial.print("\t");
  Serial.print(yaw);Serial.print("\t");
  Serial.println();
}

