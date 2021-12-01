#ifndef _MAIN_H_
#define _MAIN_H_

//Headers
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include "const.h"
#include "util.h"
#include "mem_SD.h"


//Functions
void endLogging_ISR(void);
void dateTime(uint16_t* date, uint16_t* time);
void updateOled(void);

// Externals
extern File myFile;

#endif // _MAIN_H_
