/**
 * @file main.h
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief the main.h file linking the entire program 
 * @version 0.1
 * @date 2021-12-09
 * 
 */
#ifndef MAIN_H_
#define MAIN_H_

//Headers
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "const.h"
#include "util.h"
#include "mem_SD.h"
#include "my_GNSS.h"


//Functions
void endLogging_ISR(void);
void logDataPoint_ISR(void);
void dateTime(uint16_t* date, uint16_t* time);
void updateOled(void);
bool getCurrentTime(void);

// External file pointers
extern File rxmFile;
extern File timeSlotFile;

#endif // MAIN_H_
