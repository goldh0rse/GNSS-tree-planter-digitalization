/**
 * @file util.h
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief  The utilities definitions of the project.
 *         
 *         Future version may hold the memory handling functionalities also
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef UTIL_H_
#define UTIL_H_

#include <Arduino.h>
#include "const.h"

// Functions
void writeIntToMonitor(uint32_t i, boolean line);
void writeStrToMonitor(const char *str, boolean line);
void setupMonitor();

// External variables
extern volatile boolean monitorAvailable;

#endif
