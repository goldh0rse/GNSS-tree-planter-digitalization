#include "mem_SD.h"

File myFile;


void setupSD(void){
  // See if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    writeStrToMonitor("Card failed, or not present. Freezing...", true);
    // don't do anything more:
    while (1);
  }
  
  char fileName[12] = "b001.ubx";
  uint16_t id = 1;
  while(SD.exists(fileName) && id < 1000){
    id++;
    sprintf(fileName, FILE_NAME, id);
    if (id > 999){
      sprintf(fileName, FILE_NAME, 0);
    }
  }
  
  myFile = SD.open(fileName, FILE_WRITE);
  if(!myFile) {
    writeStrToMonitor("Failed to create UBX data file! Freezing...", true);
    while (1);
  }
}