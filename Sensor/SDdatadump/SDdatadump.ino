#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SPI_SCK 5
#define SPI_MISO 19
#define SPI_MOSI 27
#define SD_CS 12
#define LORA_CS 18

String path;
int a = 0;
void setup() {
  Serial.begin(115200);
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  if (!SD.begin(SD_CS)) {
    Serial.println("Keep going dont give up");
    return;
  } else {
    Serial.println("Done");
  }
  
  // if(SD.mkdir("/SDDataLog")){
  //       Serial.println("Dir created");
  //   } else {
  //       Serial.println("mkdir failed");
  //   }
  SDlogFile();

}

void loop() {
  SDDataDump(String(a));
  a++;
}

void SDlogFile() {
  int fileN = 0;
  File root = SD.open("/SDDataLog");
  File file = root.openNextFile();
  while(file){
    fileN += 1;
    file = root.openNextFile();
  }
  // Serial.print("total file: "); Serial.println(fileN);
  fileN += 1;
  path = String("/SDDataLog") + String("/Datalog") + String(fileN) + String(".txt");
  Serial.println(path);
  File newfile = SD.open(path,FILE_WRITE);
  newfile.println("CANSAT data log:");
  newfile.close();
}

void SDDataDump(String packet){
  File dump = SD.open(path,FILE_APPEND);
  dump.println(packet);
}
