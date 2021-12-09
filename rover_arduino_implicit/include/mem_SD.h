/**
 * @file mem_SD.h
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief  The definition of the memory handling functionalities of the project
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef MEM_SD_H_
#define MEM_SD_H_

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "const.h"
#include "util.h"

// Functions
bool setupSD();

#endif // MEM_SD_H_