#include <Wire.h>
#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD;
const int SLAVE_ADDRESS = 0xAA;

// Buffer to store incoming data
String dataBuffer = "";
bool dataReady = false;
String filename = "avionics_log_";

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  if (!SD.begin(chipSelect)) {
    return;
  }

  // Find lowest available filename
  for (unsigned short filename_num = 0; filename_num < 10000; filename_num++) {
    filename = "avionics_log_" + String(filename_num) + ".txt";
    if (!SD.exists(filename.c_str())) {
      break;
    }
  }

  // Create the file immediately
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  dataFile.close();
}

void loop() {
  // Save data to SD card in the main loop, not in the ISR
  if (dataReady) {
    File dataFile = SD.open(filename.c_str(), FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataBuffer);
      dataFile.close();
    }
   
    dataBuffer = "";
    dataReady = false;
  }
}

void receiveEvent(size_t count) {
  while (Wire.available()) {
    char c = Wire.read();
    dataBuffer += c;
  }
  dataReady = true;
}
