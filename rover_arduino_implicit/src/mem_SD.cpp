#include "mem_SD.h"

File rxmFile;
File timeSlotFile;

bool setupSD(void){
  // See if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    writeStrToMonitor("Could not connect to SD!", true);
    return false;
  }
  writeStrToMonitor("Here!!", true);

  
  uint16_t id = 1;
  char rxmFileName[12] = "r001.ubx";
  while(SD.exists(rxmFileName) && id < 1000){
    id++;
    sprintf(rxmFileName, "r%03d.ubx", id);
  }
  if (id > 999){
    return false;
  }

  // rxmFile = SD.open(rxmFileName, FILE_WRITE); //FILE NAME LIMITED TO 8 CHARS
  rxmFile = SD.open(rxmFileName, O_CREAT | O_WRITE);
  if(!rxmFile) {
    return false;
  }

  id = 1;
  char timeFileName[12] = "t001.txt";
  while(SD.exists(timeFileName) && id < 1000){
    id++;
    sprintf(timeFileName, "t%03d.txt", id);
  }
  if (id > 999){
    return false;
  }

  // timeSlotFile = SD.open(timeFileName, FILE_WRITE);
  timeSlotFile = SD.open(timeFileName, O_CREAT | O_WRITE);
  if(!timeSlotFile) {
    return false;
  }

  return true;
}