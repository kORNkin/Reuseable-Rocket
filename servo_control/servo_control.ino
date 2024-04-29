#include <ESP32Servo.h>

Servo fin[5];
void setup() {
  Serial.begin(9600);
  fin[1].attach(15);
  fin[2].attach(2);
  fin[3].attach(4);
  fin[4].attach(16);
}

String in;
void loop() {
  if(Serial.available()){
    in = Serial.readString();
    if(in != ""){
      setfin(1, in.toInt());
      setfin(2, in.toInt());
      setfin(3, in.toInt());
      setfin(4, in.toInt());
    }
  }
}

void setfin(int idx, int deg){
  fin[idx].write(deg);
  Serial.println("fin" + String(idx) + ": " + String(deg));
}
