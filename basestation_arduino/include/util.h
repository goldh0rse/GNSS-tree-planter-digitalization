#ifndef _UTIL_H_
#define _UTIL_H_

#include <Arduino.h>
#include "const.h"

// Functions
void writeIntToMonitor(uint32_t i, boolean line);
void writeStrToMonitor(const char *str, boolean line);
void setupMonitor();

// External variables
extern volatile boolean monitorAvailable;

#endif
