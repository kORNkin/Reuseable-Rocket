#include <ESP32Servo.h>

#define Inverse true

Servo fin[6];
void setup() {
  Serial.begin(9600);
  fin[1].attach(15);
  fin[2].attach(33);
  fin[3].attach(32);
  fin[4].attach(13);
  fin[5].attach(12);
}

String in;
void loop() {
  if(Serial.available()){
    in = Serial.readString();
    if(in != ""){
      setfin(1, in.toInt(),false);
      setfin(2, in.toInt(),Inverse);
      setfin(3, in.toInt(),false);
      setfin(4, in.toInt(),Inverse);
      setfin(5, in.toInt(),false);
    }
    
  }
}

void setfin(int idx, int deg, bool inverse){
  if (inverse) {
    fin[idx].write(180-deg);
  }
  else{
    fin[idx].write(deg);
  }
  Serial.println("fin" + String(idx) + ": " + String(deg));
  
}
