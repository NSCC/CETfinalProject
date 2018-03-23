#include <SD.h>
#include <SPI.h>
#include <LoRa.h>

const int chipSelect = 4;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  delay(5000);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet, ");

    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      // read packet
      while (LoRa.available()) {
      dataFile.print((char)LoRa.read());
    }
    dataFile.println("");
    dataFile.close();
  }

    // print RSSI of packet
    Serial.print("RSSI :");
    Serial.println(LoRa.packetRssi());
  }
}
