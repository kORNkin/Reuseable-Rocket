#include <IMUGY85.h>


IMUGY85 imu;
double ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw;
double* Offset;

void setup()
{
  Serial.begin(9600);
  imu.init();
  Offset = IMUOffset();
  Serial.print(Offset[0]); Serial.print(", ");
  Serial.print(Offset[1]); Serial.print(", ");
  Serial.print(Offset[2]); Serial.println(" ");
  
  
}

void loop()
{
  imu.update();
  
  // // printAccel();
  // // printGyro();
  // // printMagneto();
  printRollPitchYaw(Offset);

  delay(10);
}

void printAccel()
{
  imu.getAcceleration(&ax, &ay, &az);
  Serial.print(ax);Serial.print("\t");
  Serial.print(ay);Serial.print("\t");
  Serial.print(az);Serial.print("\t");
}

void printGyro()
{
  imu.getGyro(&gx, &gy, &gz);
  Serial.print(gx);Serial.print("\t");
  Serial.print(gy);Serial.print("\t");
  Serial.print(gz);Serial.print("\t");
}

void printMagneto()
{
  imu.getMagneto(&mx, &my, &mz);
  Serial.print(mx);Serial.print("\t");
  Serial.print(my);Serial.print("\t");
  Serial.print(mz);Serial.print("\t");
  Serial.println();
}

void printRollPitchYaw(double* offset)
{
  roll = imu.getRoll() - offset[0];
  pitch = imu.getPitch() - offset[1];
  yaw = imu.getYaw() - offset[2];
  yaw = UseableYaw(yaw);
  Serial.print(pitch);Serial.print("\t");
  Serial.print(roll);Serial.print("\t");
  Serial.print(yaw);Serial.print("\t");
  Serial.println();
}

double UseableYaw(double yaw){
  if(yaw > 180) return yaw - 360;
  return yaw; 
}
//wait to add timer
double* IMUOffset(){
  unsigned int timer;
  timer = millis() + 10000;
  while(millis() <= timer){
    imu.update();
    delay(10);
  }
  int c = 0;
  static double Er[3];
  double rollEr,pitchEr,yawEr ;
  while(c < 2000){
    imu.update();
    rollEr += imu.getRoll();
    pitchEr += imu.getPitch();
    yawEr += imu.getYaw();
    c++;
  }
  Er[0] = rollEr / c;
  Er[1] = pitchEr / c;
  Er[2] = yawEr / c;
  return Er;
}
