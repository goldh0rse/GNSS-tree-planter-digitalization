#include "util.h"


void setupMonitor(){
  Serial.begin(__BAUDRATE__);
  while (!Serial); //Wait for user to open terminal

  while (Serial.available()) {
      // Make sure the Serial buffer is empty
      Serial.read();
  }

  Serial.println(F("Press any key to start logging."));
  monitorAvailable = true;
  // Wait for the user to press a key
  while (!Serial.available()) {
    ; // Do nothing
  }
  Serial.println(F("Exiting monitor setup."));
  delay(100); // Wait, just in case multiple characters were sent

  // Empty the Serial buffer
  while (Serial.available()) {
    Serial.read();
  }
}

void writeStrToMonitor(const char *str, boolean line){
  if(monitorAvailable){
    if(line){
      Serial.println(F(str));
    } else {
      Serial.print(F(str));
    }
  }
}

void writeIntToMonitor(uint32_t i, boolean line){
  if(monitorAvailable){
    if(line){
      Serial.println(i);
    } else {
      Serial.print(i);
    }
  }
}