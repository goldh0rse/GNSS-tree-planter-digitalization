#ifndef _MAIN_H_
#define _MAIN_H_

//Headers
#include <Arduino.h>
// #include <ArduinoBLE.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "const.h"
#include "util.h"
#include "mem_SD.h"


//Functions
void endLogging_ISR(void);
void logDataPoint_ISR(void);
void dateTime(uint16_t* date, uint16_t* time);
void updateOled(void);

// Externals
extern File rxmFile;
extern File timeSlotFile;


class SFE_UBLOX_GPS_ADD : public SFE_UBLOX_GNSS{
    public:
        bool getModuleInfo(uint16_t maxWait = defaultMaxWait); 
        bool getCurrentTime(uint16_t maxWait = defaultMaxWait);

        struct minfoStructure  {
            char swVersion[30];
            char hwVersion[10];
            uint8_t extensionNo = 0;
            char extension[10][30];
        } minfo;

        struct timeStructure {
            uint16_t year;
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t minute;
            uint8_t second;
        } tStruct;
};

#endif // _MAIN_H_
