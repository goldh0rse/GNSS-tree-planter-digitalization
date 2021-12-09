/**
 * @file main.cpp
 * @author Klas Holmberg (hed16khg@cs.umu.se)
 * @brief The main program for connecting and extracting data from the GNSS
 *        module over i2c. Logs RXM_RAWX data & timestamps to two seperate
 *        files, Rxxx.ubx & Txxx.txt respectively.
 *
 *        Built around the functionality of the ZED-F9P GNSS chip from ublox.
 *
 *        I2C: GNSS ZED-F9P, Adafruit SSD1306 display
 *              https://www.u-blox.com/en/product/zed-f9p-module
 *              https://learn.adafruit.com/monochrome-oled-breakouts/arduino-library-and-examples
 *
 *        SPI: Adafruit 5v ready Micro-SD Breakout board+
 *              https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial
 *
 * @version 0.1
 * @date 2021-12-09
 *
 */

#include "main.h"


//Globals
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;
volatile boolean logDataState = false;
uint16_t posCounter = 0;

// Extended SFE_UBLOX_GPS class
SFE_UBLOX_GPS_ADD myGNSS;

// SSD1306 SPI dislay.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/**
 * @brief The main setup function for the program, sets up the communication
 *        with the other modules connected to the arduino.
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

  //OLED
  pinMode(OLED_RESET, OUTPUT);

  // Init I2C
  Wire.begin();
  Wire.setClock(400000); //Optional. Increase I2C clock speed to 400kHz.

  // Init Serial monitor, uncomment if serial monitor is to be used.
  // setupMonitor();

  // Connect to the Screen over i2c
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    writeStrToMonitor("SSD1306 allocation failed", true);
    exit(-1);
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display(); delay(1000); // Pause for 1 seconds
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  display.println(F("Init GNSS..."));
  display.display();

  char buffer[40] = "";
  display.setCursor(0, ROW*3);
  display.print(F("Buffer: "));
  sprintf(buffer, "%d Byte", FILE_BUFFER_SIZE);
  display.println(buffer);

  display.setCursor(0, ROW*4);
  sprintf(buffer, "%d Byte", UBX_RXM_RAWX_MAX_LEN);
  display.print(F("Packet: ")); display.println(buffer);
  display.display(); delay(2000);

  display.clearDisplay();
  display.setCursor(0, ROW * 0);
  display.println(F("Starting GNSS..."));
  display.display();

  // RAWX data can legitimately contain 0x7F,
  // so we need to disable the "7F" check in checkUbloxI2C
  myGNSS.disableUBX7Fcheck();

  // setFileBufferSize must be called _before_ .begin
  myGNSS.setFileBufferSize(FILE_BUFFER_SIZE);

  // RawX data is big, need to resize payload to accomodate for the size
  myGNSS.setPacketCfgPayloadSize(UBX_RXM_RAWX_MAX_LEN);
  delay(1000); // Give the module some extra time to get ready

  // Connect to the u-blox module using Wire port
  if (!myGNSS.begin()) {
    writeStrToMonitor("u-blox GNSS not detected at default I2C address.", true);
    exit(-1);
  }
  display.setCursor(0, ROW*1);
  display.println(F("Connected over i2c!"));
  display.setCursor(0, ROW*2);
  display.println(F("Resetting GNSS Settings to factory default..."));
  display.display();

  // Reset to factory default (This can probably be commented out)
  myGNSS.factoryDefault(); delay(5000);
  // Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setI2COutput(COM_TYPE_UBX);

  // Fetch module info for display
  if (!myGNSS.getModuleInfo(1100)){
    display.clearDisplay();
    display.println(F("getModuleInfo failed!"));
    exit(-1);
  }

  // Display GNSS module info
  display.clearDisplay();
  display.setCursor(0, ROW*0); display.println(F("Module Info"));
  display.setCursor(0, ROW*1); display.println(F("Soft Version:"));
  display.setCursor(0, ROW*2); display.println(myGNSS.minfo.swVersion);
  display.setCursor(0, ROW*4); display.println(F("Hard Version:"));
  display.setCursor(0, ROW*5); display.println(myGNSS.minfo.hwVersion);
  display.display();

  // Enable all GNSS signals except IMES
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_IMES);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
  delay(2000);

  // Show the enabled systems to the user
  display.clearDisplay();
  display.setCursor(0, ROW*0);
  display.println(F("Enabled systems:"));
  display.setCursor(0, ROW*2);
  display.print(F("GPS: ")); display.print(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_GPS));
  display.print(F(" BeiDou: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_BEIDOU));

  display.setCursor(0, ROW*3);
  display.print(F("SBAS: ")); display.print(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_SBAS));
  display.print(F(" Galileo: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_GALILEO));

  display.setCursor(0, ROW*4);
  display.print(F("IMES: ")); display.print(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_IMES));
  display.print(F(" QZSS: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_QZSS));

  display.setCursor(0, ROW*5);
  display.print(F("GLONASS: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_GLONASS));
  display.display();

  // Save (only) the communications port settings to flash and BBR
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  // Increase allowed i2c polling speed
  myGNSS.setI2CpollingWait(25);

  //Produce one navigation solution per second
  // (that's plenty for Precise Point Positioning)
  myGNSS.setNavigationFrequency(__NAV_RATE__);

  // Enable automatic RXM RAWX messages: no callback; no implicit update
  myGNSS.setAutoRXMRAWX(true, false);
  myGNSS.logRXMRAWX(true); // Enable RXM RAWX data logging

  // Set dynamic model to "pedestrian", Affects the 'gnssFixOk'-flag.
  // MODE       Altitude  Hor.Vel  Ver.Vel  Sanity check        Position Dev.
  // Pedestrian 9000      30       20        Altitude & velocity Small
  myGNSS.setDynamicModel(DYN_MODEL_PEDESTRIAN);
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, ROW*0);
  display.print(F("Disabling TP..."));
  display.display(); delay(500);

  // Disable TimePulse-mode
  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (!myGNSS.getTimePulseParameters(&timePulseParameters)) {
    writeStrToMonitor("getTimePulseParameters failed! Freezing...", true);
    display.println(F("Failed")); display.display();
    exit(-1);
  }

  // Make sure the active flag is low to disable the time pulse.
  timePulseParameters.flags.bits.active = 0;
  if (!myGNSS.setTimePulseParameters(&timePulseParameters)){
    writeStrToMonitor("setTimePulseParameters failed!", true);
    display.println(F("Failed")); display.display();
    exit(-1);
  }
  display.println(F("Done"));
  display.setCursor(0, ROW*1);
  display.println(F("Checking for valid")); display.setCursor(0, ROW*2);
  display.print(F("date  & time..."));
  display.display(); delay(1000);

  // Save full configuration
  myGNSS.saveConfiguration();

  // Wait here until the date and time can be fully resolved
  while(!(myGNSS.getDateValid() && myGNSS.getTimeValid()));

  display.println(F("Done"));
  display.setCursor(0, ROW*3);
  display.print(F("SD setup...."));
  display.display(); delay(1000);


  // Init SD-card
  SdFile::dateTimeCallback(dateTime);
  if (!setupSD()){
    display.println(F("Failed.")); display.display(); delay(1000);
    exit(-1);
  }

  // Initialize external interrupts
  attachInterrupt(digitalPinToInterrupt(LOG_DATA_PIN), logDataPoint_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(END_LOGGING_PIN), endLogging_ISR, FALLING);
  interrupts();

  display.println(F("   Done"));
  display.setCursor(0, ROW*4);
  display.println(F("Entering loop!"));
  display.display(); delay(1000);
  //END SETUP LEDS
  digitalWrite(LOG_DATA_LED, LOW);
  digitalWrite(END_LOGGING_LED, LOW);
  digitalWrite(READY_LED, HIGH);
}

/**
 * @brief The main loop of the program.
 *
 */
void loop() {
  digitalWrite(READY_LED, HIGH);
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  
  while (myGNSS.fileBufferAvailable() > SD_WRITE_SIZE){
    // Create our own buffer to hold the data while we write it to SD card
    uint8_t myBuffer[SD_WRITE_SIZE];

    // Extract exactly SD_WRITE_SIZE bytes from the UBX file buffer &
    // put them into myBuffer
    myGNSS.extractFileBufferData((uint8_t *)&myBuffer, SD_WRITE_SIZE);

    // Write exactly SD_WRITE_SIZE bytes from myBuffer to the
    // rxmFile on the SD card
    rxmFile.write(myBuffer, SD_WRITE_SIZE);
  }

  // Log timestamp
  if(logDataState){
    digitalWrite(READY_LED, LOW);
    digitalWrite(LOG_DATA_LED, HIGH);
    display.clearDisplay(); display.setCursor(0, ROW*0);
    display.println(F("Logging timestamp"));
    display.display();

    while(logDataState){

      // Uncomment for stricter datacheck of data at timeslot
      // if(myGNSS.getGnssFixOk()){
        //if(myGNSS.getDateValid() && myGNSS.getTimeValid()) {
        // Get current date & time from GNSS module
        getCurrentTime();
        logDataState = false;
        posCounter++;
      // }
    }

    digitalWrite(READY_LED, HIGH);
    digitalWrite(LOG_DATA_LED, LOW);
  }

  // Update data to show the user
  updateOled();

  // Exit logging state
  if (endLoggingState){
    // Fetch remaining data from GNSS RXM buffer
    uint16_t remainingBytes = myGNSS.fileBufferAvailable();
    while (remainingBytes > 0){

      uint8_t buffer[SD_WRITE_SIZE];
      int bytesToWrite = remainingBytes;

      if(bytesToWrite > SD_WRITE_SIZE){
        bytesToWrite = SD_WRITE_SIZE;
      }

      myGNSS.extractFileBufferData((uint8_t *)&buffer, bytesToWrite);
      rxmFile.write(buffer, bytesToWrite);

      remainingBytes -= bytesToWrite;
    }

    // Close the rxmFile & timeSlotFile
    rxmFile.close();
    timeSlotFile.close();

    // Clear screen & Show exit screen.
    display.clearDisplay(); display.setCursor(0, ROW*0);
    display.println(F("Exited program")); display.display();
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
 * @brief The log data ISR function called by the external interrupt on
 *        the LOG_DATA_PIN (6)
 *
 */
void logDataPoint_ISR(void){
  logDataState = true;
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
 * @brief The main update function for displaying information to the user
 *
 */
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

  display.setCursor(0, ROW*5);
  display.print(F("Count: ")); display.println(posCounter);

  display.display();
}

/**
 * @brief Get the Current Time from the GNSS module & write the data to the
 *        timeSlotFile.
 *
 * @return true
 * @return false
 */
bool getCurrentTime(void){

  char buffer[40] = "";

  sprintf(buffer, "%u/%02d/%02d %02d-%02d-%02d.000\n",
    myGNSS.getYear(),
    myGNSS.getMonth(),
    myGNSS.getDay(),
    myGNSS.getHour(),
    myGNSS.getMinute(),
    myGNSS.getSecond()
  );
  timeSlotFile.write(buffer, 24);
  timeSlotFile.flush();

  return (true);
}
