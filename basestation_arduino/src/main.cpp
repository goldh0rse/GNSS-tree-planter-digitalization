#include "main.h"

//Globals
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;
SFE_UBLOX_GNSS myGNSS;


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

  myGNSS.disableUBX7Fcheck(); // RAWX data can legitimately contain 0x7F, so we need to disable the "7F" check in checkUbloxI2C

  myGNSS.setFileBufferSize(FILE_BUFFER_SIZE); // setFileBufferSize must be called _before_ .begin

  delay(1000); // Give the module some extra time to get ready

  //Connect to the u-blox module using Wire port
  if (!myGNSS.begin()) {
    writeStrToMonitor("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing...", true);
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_IMES);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);

  delay(2000);

  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(__NAV_RATE__); //Produce one navigation solution per second (that's plenty for Precise Point Positioning)

  myGNSS.setAutoRXMSFRBX(true, false); // Enable automatic RXM SFRBX messages: without callback; without implicit update

  myGNSS.logRXMSFRBX(true); // Enable RXM SFRBX data logging

  myGNSS.setAutoRXMRAWX(true, false); // Enable automatic RXM RAWX messages: without callback; without implicit update

  myGNSS.logRXMRAWX(true); // Enable RXM RAWX data logging

  delay(1000); // Give the module some extra time to get ready

  myGNSS.setDynamicModel(DYN_MODEL_STATIONARY); // For the Base
  // myGNSS.setDynamicModel(DYN_MODEL_PEDESTRIAN); // For the Rover, Affects the 'gnssFixOk'-flag.

  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (myGNSS.getTimePulseParameters(&timePulseParameters) == false) {
    writeStrToMonitor("getTimePulseParameters failed! Freezing...", true);
    exit(-1);
  }

  timePulseParameters.flags.bits.active = 0; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false){
    writeStrToMonitor("setTimePulseParameters failed!", true);
    exit(-1);
  }
  while(!(myGNSS.getDateValid() && myGNSS.getTimeValid()));

  // Init SD-card
  SdFile::dateTimeCallback(dateTime);
  setupSD();

  attachInterrupt(digitalPinToInterrupt(END_LOGGING_PIN), endLogging_ISR, FALLING);
  interrupts();

  //END SETUP LEDS
  digitalWrite(READY_LED, HIGH);
  digitalWrite(LOG_DATA_LED, LOW);
  digitalWrite(END_LOGGING_LED, LOW);

  writeStrToMonitor("Setup Finished!", true);
}


void loop() {
  myGNSS.checkUblox();

  uint16_t remainingBytes = myGNSS.fileBufferAvailable();
  while(remainingBytes > 0){
    digitalWrite(READY_LED, LOW);
    digitalWrite(LOG_DATA_LED, HIGH);

    uint8_t myBuffer[SD_WRITE_SIZE]; // Create our own buffer to hold the data while we write it to SD card
    uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time

    if (bytesToWrite > SD_WRITE_SIZE) {
        bytesToWrite = SD_WRITE_SIZE;
    }

    myGNSS.extractFileBufferData((uint8_t *)&myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer

    myFile.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxDataFile on the SD card

    remainingBytes -= bytesToWrite; // Decrement remainingBytes
    digitalWrite(LOG_DATA_LED, LOW);
    digitalWrite(READY_LED, HIGH);
  }


  if (endLoggingState){
    digitalWrite(READY_LED, LOW);

    myFile.close(); // Close the data file
    digitalWrite(END_LOGGING_LED, HIGH);
    digitalWrite(LOG_DATA_LED, LOW);
    digitalWrite(READY_LED, LOW);

    exit(0);
  }
} // END MAIN

void endLogging_ISR(void){
  endLoggingState = true;
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
}

void updateOled(void){
  display.clearDisplay();
  display.setCursor(0, ROW*0);
  display.print(F("SIV: ")); display.println(myGNSS.getSIV());

  display.setCursor(0, ROW*1);
  display.print(F("Time & Date Valid: ")); display.println(myGNSS.getTimeValid() && myGNSS.getDateValid());

  display.setCursor(0, ROW*2);
  byte fixType = myGNSS.getFixType();
  display.print(F("Fix Type: "));
  display.setCursor(10, ROW*3);
  if(fixType == 0) display.println(F("No fix"));
  else if(fixType == 1) display.println(F("Dead reckoning"));
  else if(fixType == 2) display.println(F("2D"));
  else if(fixType == 3) display.println(F("3D"));
  else if(fixType == 4) display.println(F("GNSS + Dead reckoning"));
  else if(fixType == 5) display.println(F("Time only"));

  display.setCursor(0, ROW*4);
  display.print(F("GNSS Fix: "));

  if(myGNSS.getGnssFixOk()) display.println(F("OK"));
  else display.println(F("Not OK"));

  display.display();
}
