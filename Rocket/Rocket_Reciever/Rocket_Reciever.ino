#include <SPI.h>
#include <LoRa.h>

#define  LoRa_SCK   18
#define  LoRa_MISO  19
#define  LoRa_MOSI  23
#define  LoRa_CS    5
#define  LoRa_RST   14
#define  DI0        2
#define  BAND    433E6

#define  Select    LOW   //  Low CS means that SPI device Selected
#define  DeSelect  HIGH  //  High CS means that SPI device Deselected

void setup() {
  Serial.begin(9600);
  while (!Serial);

  SPI.begin( LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_CS );
  LoRa.setPins( LoRa_CS, LoRa_RST, DI0 );
  digitalWrite(LoRa_CS, Select);   //  SELECT (low) LoRa SPI
  //Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
  } else {
    Serial.println("LoRa Initial OK!");
    delay(1000);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    } 
}
