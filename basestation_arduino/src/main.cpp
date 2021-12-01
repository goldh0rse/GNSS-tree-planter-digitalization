#include "main.h"

//Globals
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;
SFE_UBLOX_GNSS myGNSS;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


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

  // Init Serial monitor
  //setupMonitor();

  // if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
  //   writeStrToMonitor("SSD1306 allocation failed", true);
  //   for(;;); // Don't proceed, loop forever
  // }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  // display.display();
  // delay(1000); // Pause for 2 seconds
  // display.clearDisplay();
  // display.setTextSize(1);             // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE);        // Draw white text
  // display.setCursor(0,0);             // Start at top-left corner
  // display.println(F("Init GNSS..."));
  // display.display();

  // display.setCursor(0, ROW*3);
  // display.print(F("Databuffer: "));
  // display.println(FILE_BUFFER_SIZE);
  
  // display.setCursor(0, ROW*4);
  // display.print(F("Packet size: "));
  // display.println(MAX_PACKET_PAYLOAD_SIZE);
  // display.display();
  // delay(2000);

  // display.clearDisplay();
  // display.setCursor(0, ROW * 0);
  // display.println(F("Starting GNSS..."));
  // display.display();

  myGNSS.disableUBX7Fcheck(); // RAWX data can legitimately contain 0x7F, so we need to disable the "7F" check in checkUbloxI2C

  myGNSS.setFileBufferSize(FILE_BUFFER_SIZE); // setFileBufferSize must be called _before_ .begin
  
  // myGNSS.setPacketCfgPayloadSize(UBX_CLASS_RXM, UBX_RXM_RAWX); // RawX data is big, need to resize payload to accomodate for the size

  delay(1000); // Give the module some extra time to get ready

  //Connect to the u-blox module using Wire port
  if (!myGNSS.begin()) {
    writeStrToMonitor("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing...", true);
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // display.setCursor(0, ROW*1);
  // display.println(F("Connected over i2c!"));
  // display.setCursor(0, ROW*3);
  // display.println(F("Enabling systems..."));
  // display.display();

  // writeStrToMonitor("Initializing to FactoryDefault...", false);
  // myGNSS.factoryDefault(); delay(5000);
  // writeStrToMonitor("Done!", true);

  // myGNSS.enableDebugging(); // Uncomment this line to enable lots of helpful debug messages
  // myGNSS.enableDebugging(Serial, true); // Uncomment this line to enable the minimum of helpful debug messages


  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS); 
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_IMES);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS); 
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS); 
  
  delay(2000);

  // display.clearDisplay();
  // display.setCursor(0, ROW*0);
  // display.println(F("Enabling systems..."));
  // display.setCursor(0, ROW*2);
  // display.print(F("GPS: ")); display.print(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_GPS));
  // display.print(F(" BeiDou: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_BEIDOU));
  
  // display.setCursor(0, ROW*3);
  // display.print(F("SBAS: ")); display.print(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_SBAS));
  // display.print(F(" Galileo: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_GALILEO));
  
  // display.setCursor(0, ROW*4);
  // display.print(F("IMES: ")); display.print(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_IMES));
  // display.print(F(" QZSS: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_QZSS));
  
  // display.setCursor(0, ROW*5);
  // display.print(F("GLONASS: ")); display.println(myGNSS.isGNSSenabled(SFE_UBLOX_GNSS_ID_GLONASS));
  // display.display();

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
    while (1) ; // Do nothing more
  }

  timePulseParameters.flags.bits.active = 0; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false){
    writeStrToMonitor("setTimePulseParameters failed!", true);
    while(1);
    // Do nothing more
  }
  // checkFixType();
 
  while(!(myGNSS.getDateValid() && myGNSS.getTimeValid())){
    // display.clearDisplay();
    // display.setCursor(0, ROW*0);
    // display.println(F("Waiting for valid Date and Time."));
    // display.display();
  }
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
  digitalWrite(READY_LED, HIGH);
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

  // digitalWrite(READY_LED, HIGH);


  if (endLoggingState){
    digitalWrite(READY_LED, LOW);

    myFile.close(); // Close the data file
    digitalWrite(END_LOGGING_LED, HIGH);
    digitalWrite(LOG_DATA_LED, LOW);
    digitalWrite(READY_LED, LOW);
    
    // myGNSS.end(); // Frees all RAM onboard the zed-f9p

    while(1);
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