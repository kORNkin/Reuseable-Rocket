#include <ESP32Servo.h>

Servo fin[5];
Servo para, eject;

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
  eject.attach(15);
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
      para.write(in.toInt());
      eject.write(in.toInt());
    }
  }
}

void setfin(int idx, int deg){
  fin[idx].write(deg);
  Serial.println("fin" + String(idx) + ": " + String(deg));
}
