#ifndef _CONST_H_
#define _CONST_H_

// MACROS
#define SETBIT(ADDR, BIT) (ADDR |= (1<<BIT))
#define CLEARBIT(ADDR, BIT) (ADDR &= ~(1<<BIT))
#ifndef __BAUDRATE__
    #define __BAUDRATE__ 115200
#endif

// LEDs
#define READY_LED 9
#define LOG_DATA_LED 8
#define END_LOGGING_LED 7
// BUTTONS
#define LOG_DATA_PIN 3
#define END_LOGGING_PIN 2

// SD-Card
#define SD_CHIP_SELECT 10       // Primary SPI Chip Select is CS for the MicroMod Artemis Processor. Adjust for your processor if necessary.
#define SD_WRITE_SIZE 512       // Write data to the SD card in blocks of 512 bytes
#define FILE_BUFFER_SIZE 16384  // Allocate 16KBytes of RAM for UBX message storage
#define FILE_NAME "b%03d.ubx"


// GNSS
#define __NAV_RATE__ 1
#define __MEAS_RATE__ 1
#define MAX_PACKET_PAYLOAD_SIZE 2616 // My bullshit lets seeeeee...... RAW messages are biiiiiiiiiig
#define PACKET_PAYLOAD_SIZE  512 // Adjusted for writesize of SD card

// OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
#define OLED_ADDRESS 0x3D
#define ROW 10


#endif