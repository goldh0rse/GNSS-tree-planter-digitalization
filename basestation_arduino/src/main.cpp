/**
 * @file main.cpp
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief  The Base-station main file, handles the communication,
 *         reading, logging of data & the user interface.
   *
   *      Built around the functionality of the ZED-F9P GNSS chip from ublox.
 *
 *        I2C: GNSS ZED-F9P
 *              https://www.u-blox.com/en/product/zed-f9p-module
 *
 *        SPI: Adafruit 5v ready Micro-SD Breakout board+
 *             https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial
 *
 * @version 0.1
 * @date 2021-12-09
 *
 */
#include "main.h"

//Globals
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;
SFE_UBLOX_GNSS myGNSS;

/**
 * @brief The main setup function for the program, sets up the communication
 *        with the connected modules.
 *
 */
void setup(){
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
  pinMode(LOG_DATA_PIN, INPUT_PULLUP);

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

  // Enable automatic RXM SFRBX messages without callback; without implicit update
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


void loop() {
  digitalWrite(READY_LED, HIGH);
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.

  while (myGNSS.fileBufferAvailable() > SD_WRITE_SIZE){
    digitalWrite(READY_LED, LOW);
    digitalWrite(LOG_DATA_LED, HIGH);

    // Create our own buffer to hold the data while we write it to SD card
    uint8_t myBuffer[SD_WRITE_SIZE];

    // Extract exactly SD_WRITE_SIZE bytes from the UBX file buffer &
    // put them into myBuffer
    myGNSS.extractFileBufferData((uint8_t *)&myBuffer, SD_WRITE_SIZE);

    // Write exactly SD_WRITE_SIZE bytes from myBuffer to the
    // rxmFile on the SD card
    rxmFile.write(myBuffer, SD_WRITE_SIZE);
    myGNSS.checkUblox();

    digitalWrite(READY_LED, HIGH);
    digitalWrite(LOG_DATA_LED, LOW);
  }

  // Exit logging
  if (endLoggingState){
    digitalWrite(READY_LED, LOW);

    // Fetch remaining data from GNSS RXM buffer
    uint16_t remainingBytes = myGNSS.fileBufferAvailable();
    while(remainingBytes > 0){
      digitalWrite(LOG_DATA_LED, HIGH);

      // Create our own buffer to hold the data while we write it to SD card
      uint8_t myBuffer[SD_WRITE_SIZE];
      // Write the remaining bytes to SD card sdWriteSize bytes at a time
      uint16_t bytesToWrite = remainingBytes;

      if (bytesToWrite > SD_WRITE_SIZE) {
          bytesToWrite = SD_WRITE_SIZE;
      }

      // Extract bytes from the ubx filebuffer & put them into myBuffer
      myGNSS.extractFileBufferData((uint8_t *)&myBuffer, bytesToWrite);
      // Write bytesToWrite bytes from myBuffer to the rxmFile on the SD card
      rxmFile.write(myBuffer, bytesToWrite);

      remainingBytes -= bytesToWrite; // Decrement remainingBytes
      digitalWrite(LOG_DATA_LED, LOW);
    }

    rxmFile.close(); // Close the rxmdata file
    digitalWrite(END_LOGGING_LED, HIGH);
    digitalWrite(LOG_DATA_LED, LOW);
    digitalWrite(READY_LED, LOW);

    exit(0);
  }
} // END MAIN

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
