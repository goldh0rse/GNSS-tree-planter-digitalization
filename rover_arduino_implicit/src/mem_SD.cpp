/**
 * @file mem_SD.cpp
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief  Contains the memory handling functionalities of the project.
 * @version 0.1
 * @date 2021-12-09
 *
 */
#include "mem_SD.h"

// Files
File rxmFile;
File timeSlotFile;

/**
 * @brief The setup function for commication with the SD-card module.
 *        Also handles the creation of the files & and their naming.
 *
 * @return true
 * @return false
 */
bool setupSD(void){
  // See if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    writeStrToMonitor("Could not connect to SD!", true);
    return false;
  }

  // Check for a new RXM-RAWX filename
  uint16_t id = 1;
  char rxmFileName[12] = "r001.ubx";
  while(SD.exists(rxmFileName) && id < 1000){
    id++;
    sprintf(rxmFileName, "r%03d.ubx", id);
  }
  if (id > 999){A
    return false;
  }

  // Open new RXM-RAWX file
  rxmFile = SD.open(rxmFileName, O_CREAT | O_WRITE);
  if(!rxmFile) {
    return false;
  }

  // Check for a new Timeslot filename
  id = 1;
  char timeFileName[12] = "t001.txt";
  while(SD.exists(timeFileName) && id < 1000){
    id++;
    sprintf(timeFileName, "t%03d.txt", id);
  }
  if (id > 999){
    return false;
  }

  // Open new Timeslot file
  timeSlotFile = SD.open(timeFileName, O_CREAT | O_WRITE);
  if(!timeSlotFile) {
    return false;
  }

  return true;
}
