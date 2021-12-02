#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>

// Definitions
#define __BAUDRATE__ 115200

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

// Globals
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;
SFE_UBLOX_GNSS myGNSS;
File myFile;

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

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(END_LOGGING_PIN), endLogging_ISR, FALLING);

  //END SETUP LEDS
  digitalWrite(READY_LED, HIGH);
  digitalWrite(LOG_DATA_LED, LOW);
  digitalWrite(END_LOGGING_LED, LOW);

  writeStrToMonitor("Setup Finished!", true);
  interrupts();
}

void loop(){
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

}

void setupSD(){
  // See if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    writeStrToMonitor("Card failed, or not present. Freezing...", true);
    // don't do anything more:
    while (1);
  }

  char fileName[12] = "b001.ubx";
  uint16_t id = 1;
  while(SD.exists(fileName) && id < 1000){
    id++;
    sprintf(fileName, FILE_NAME, id);
    if (id > 999){
      sprintf(fileName, FILE_NAME, 0);
    }
  }

  myFile = SD.open(fileName, FILE_WRITE);
  if(!myFile) {
    writeStrToMonitor("Failed to create UBX data file! Freezing...", true);
    while (1);
  }
}

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

void writeStrToMonitor(const char *str, boolean line){
  if(monitorAvailable){
    if(line){
      Serial.println(F(str));
    } else {
      Serial.print(F(str));
    }
  }
}

void writeIntToMonitor(uint32_t i, boolean line){
  if(monitorAvailable){
    if(line){
      Serial.println(i);
    } else {
      Serial.print(i);
    }
  }
}
