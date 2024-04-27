#include <ESP32Servo.h>

Servo fin[5];
void setup() {
  Serial.begin(9600);
  fin[1].attach(15);
}

String in;
void loop() {
  if(Serial.available()){
    in = Serial.readString();
    if(in != ""){
      setfin(1, in.toInt());
    }
  }
}

void setfin(int idx, int deg){
  fin[idx].write(deg);
  Serial.println("fin" + String(idx) + ": " + String(deg));
}
