/**
 * @file my_GNSS.h
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief  The SFE_UBLOX_GPS_ADD class definition.
 * @version 0.1
 * @date 2021-12-09
 *
 */

#ifndef MY_GNSS_H_
#define MY_GNSS_H_

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


class SFE_UBLOX_GPS_ADD : public SFE_UBLOX_GNSS{
    public:
        bool getModuleInfo(uint16_t maxWait = defaultMaxWait);

        struct minfoStructure  {
            char swVersion[30];
            char hwVersion[10];
            uint8_t extensionNo = 0;
            char extension[10][30];
        } minfo;

};

#endif
