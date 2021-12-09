#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <SD.h>

/*--------------------------- DEFINITIONS-------------------------------------*/
// MACROS
#define SETBIT(ADDR, BIT) (ADDR |= (1<<BIT))
#define CLEARBIT(ADDR, BIT) (ADDR &= ~(1<<BIT))
#ifndef __BAUDRATE__
    #define __BAUDRATE__ 115200
#endif

// COLORS ??
#define GREEN 9
#define RED 8
#define BLUE 7

// LEDs
#define READY_LED GREEN
#define LOG_DATA_LED RED
#define END_LOGGING_LED BLUE

// BUTTONS
#define END_LOGGING_PIN 3

// SD-Card
#define SD_CHIP_SELECT 10       // Primary SPI Chip Select for SD
#define SD_WRITE_SIZE 512       // SD card Wrie Block size, 512 bytes
#define FILE_BUFFER_SIZE 16384  // Allocate 16KB of RAM for UBX message storage

// GNSS
#define __NAV_RATE__ 1
#define __MEAS_RATE__ 1
#define __FIX_TYPE__ 3
#define MAX_PACKET_PAYLOAD_SIZE 2616 // RAW messages are big...
#define PACKET_PAYLOAD_SIZE  512 // Adjusted for writesize of SD card

// OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
#define OLED_ADDRESS 0x3D
#define ROW 10

/*=============================GLOBALS========================================*/
// Files
File rxmFile;

// Values
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;

// Extended SFE_UBLOX_GPS class
SFE_UBLOX_GNSS myGNSS;
/*============================FUNCTIONS=======================================*/

/*----SETUP----*/
/**
 * @brief The main setup function for the program, sets up the communication
 *        with the other modules connected to the arduino.
 *
 */
void setup(void){
  //LEDS
  pinMode(READY_LED, OUTPUT);
  pinMode(LOG_DATA_LED, OUTPUT);
  pinMode(END_LOGGING_LED, OUTPUT);

  //START LED SETUP
  digitalWrite(READY_LED, LOW);
  digitalWrite(LOG_DATA_LED, HIGH);
  digitalWrite(END_LOGGING_LED, HIGH);

  //Buttons
  pinMode(END_LOGGING_PIN, INPUT_PULLUP);

  //SD
  pinMode(SD_CHIP_SELECT, OUTPUT);

  // Init I2C
  Wire.begin();
  Wire.setClock(400000); //Optional. Increase I2C clock speed to 400kHz.

  // Init Serial monitor, uncomment if serial monitor is wanted
  //setupMonitor();

  // RAWX data can legitimately contain 0x7F,
  // so we need to disable the "7F" check in checkUbloxI2C
  myGNSS.disableUBX7Fcheck();

  // setFileBufferSize must be called _before_ .begin
  myGNSS.setFileBufferSize(FILE_BUFFER_SIZE);

  delay(1000); // Give the module some extra time to get ready

  //Connect to the u-blox module using Wire port
  if (!myGNSS.begin()) {
    writeStrToMonitor("u-blox GNSS not detected at default I2C address.", true);
    exit(-1);
  }

  // Reset to factory default (This can probably be commented out)
  myGNSS.factoryDefault(); delay(5000);

  // Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setI2COutput(COM_TYPE_UBX);

  // Enable all GNSS signals except IMES
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_IMES);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);

  delay(2000); // Give the module some time to load

  // Save (only) the communications port settings to flash and BBR
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  //Produce one navigation solution per second
  // (that's plenty for Precise Point Positioning)
  myGNSS.setNavigationFrequency(__NAV_RATE__);

  // Enable automatic RXM SFRBX messages no callback; no implicit update
  myGNSS.setAutoRXMSFRBX(true, false);
  myGNSS.logRXMSFRBX(true); // Enable RXM SFRBX data logging

  // Enable automatic RXM RAWX messages: no callback; no implicit update
  myGNSS.setAutoRXMRAWX(true, false);
  myGNSS.logRXMRAWX(true); // Enable RXM RAWX data logging

  delay(1000); // Give the module some extra time to get ready

  // Set dynamic model to "stationary", Affects the 'gnssFixOk'-flag.
  // MODE       Altitude  Hor.Vel  Ver.Vel  Sanity check        Position Dev.
  // Stationary 9000      10       6        Altitude & velocity Small
  // Pedestrian 9000      30       20       Altitude & velocity Small
  // Portable   12000     310      50       Altitude & velocity Medium
  myGNSS.setDynamicModel(DYN_MODEL_STATIONARY); // Biggest filter
  // myGNSS.setDynamicModel(DYN_MODEL_PEDESTRIAN); // 2nd biggest filter
  // myGNSS.setDynamicModel(DYN_MODEL_PORTABLE);  // Default (3rd)

  // Disable TimePulse-mode
  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (myGNSS.getTimePulseParameters(&timePulseParameters) == false) {
    writeStrToMonitor("getTimePulseParameters failed! Freezing...", true);
    exit(-1);
  }

  // Make sure the active flag is low to disable the time pulse.
  timePulseParameters.flags.bits.active = 0;
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false){
    writeStrToMonitor("setTimePulseParameters failed!", true);
    exit(-1);
  }

  // Save full configuration
  myGNSS.saveConfiguration();

  // Wait here until the date and time can be fully resolved
  while(!(myGNSS.getDateValid() && myGNSS.getTimeValid()));

  // Init SD-card
  SdFile::dateTimeCallback(dateTime);
  setupSD();

  // Initialize external interrupts
  attachInterrupt(digitalPinToInterrupt(END_LOGGING_PIN), endLogging_ISR, FALLING);
  interrupts();

  //END SETUP LEDS
  digitalWrite(LOG_DATA_LED, LOW);
  digitalWrite(END_LOGGING_LED, LOW);
  digitalWrite(READY_LED, HIGH);

  writeStrToMonitor("Setup Finished!", true);
}

/**
 * @brief The setup function for commication with the SD-card module.
 *        Also handles the creation of the files & and their naming.
 *
 * @return true
 * @return false
 */
bool setupSD(void){
  // See if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    writeStrToMonitor("Could not connect to SD!", true);
    return false;
  }

  // Check for a new RXM-RAWX filename
  uint16_t id = 1;
  char rxmFileName[12] = "r001.ubx";
  while(SD.exists(rxmFileName) && id < 1000){
    id++;
    sprintf(rxmFileName, "r%03d.ubx", id);
  }
  if (id > 999){
    return false;
  }

  // Open new RXM-RAWX file
  rxmFile = SD.open(rxmFileName, O_CREAT | O_WRITE);
  if(!rxmFile) {
    return false;
  }

  return true;
}

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

/*----MAIN----*/
/**
 * @brief The main loop of the program.
 *
 */
void loop(void) {

}


/**
 * @brief The end logging ISR function called by the external interrupt on
 *        the END_LOGGING_PIN (3)
 *
 */
void endLogging_ISR(void){
  endLoggingState = true;
}

/**
 * @brief Writes the date & time to arduino global values, causes filenames
 *        to be properly date set.
 *
 * @param date defaults to arduino global date variable
 * @param time defaults to arduino gloabl time variable
 */
void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
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
