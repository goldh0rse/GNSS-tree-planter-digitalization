/**
 * @file util.cpp
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief  The utility function definitions used for debugging & writing to
 *         Serial monitor without having to comment/uncomment all serial
 *         source code.
 * @version 0.1
 * @date 2021-12-09
 * 
 */
#include "util.h"

/**
 * @brief Sets up the serial communication with a baudrate of
 *        __BAUDRATE__ (115200)
 *
 */
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

/**
 * @brief A function used to write strings to Serial monitor.
 *
 * @param str The string to write to Serial monitor
 * @param line A boolean for picking serial write style, .print or .println
 */
void writeStrToMonitor(const char *str, boolean line){
  if(monitorAvailable){
    if(line){
      Serial.println(F(str));
    } else {
      Serial.print(F(str));
    }
  }
}

/**
 * @brief A function used to write integers to Serial monitor.
 *
 * @param i The integer to be written to Serial monitor
 * @param line A boolean for picking serial write style, .print or .println
 */
void writeIntToMonitor(uint32_t i, boolean line){
  if(monitorAvailable){
    if(line){
      Serial.println(i);
    } else {
      Serial.print(i);
    }
  }
}
