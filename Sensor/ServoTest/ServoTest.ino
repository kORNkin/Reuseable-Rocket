#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_LIS3MDL.h>
#include <ESP32Servo.h>

#include <Wire.h>

//Servo Fin
#include <ESP32Servo.h>

Servo fin[5];
Servo para;

void setup() {

  Serial.begin(9600);


  /// FIN Pin Servo 
  // pitch controling fin
  fin[1].attach(25); 
  fin[2].attach(33);
  // roll controling fin
  fin[3].attach(32);  
  fin[4].attach(13); 
  para.attach(12);

}

void loop() {

  para.write(90);
  //setfinAll(90);
  //servoCheck();

}

void setfin(int n){
  fin[1].write(n);
}

void setfinAll(int n){
  fin[1].write(n);
  fin[2].write(n);
  fin[3].write(n);
  fin[4].write(n);
}

void servoCheck(){
  fin[1].write(45);
  fin[2].write(45);
  fin[3].write(45);
  fin[4].write(45);

  delay(500);

  fin[1].write(90);
  fin[2].write(90);
  fin[3].write(90);
  fin[4].write(90);

  delay(500);

  fin[1].write(135);
  fin[2].write(135);
  fin[3].write(135);
  fin[4].write(135);

  delay(1000);

  fin[1].write(45);
  fin[2].write(135);
  fin[3].write(45);
  fin[4].write(135);

  delay(1000);

  fin[1].write(90);
  fin[2].write(90);
  fin[3].write(90);
  fin[4].write(90);

  delay(500);
}
