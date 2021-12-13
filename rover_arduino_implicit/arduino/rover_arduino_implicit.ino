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
#define LOG_DATA_PIN 6
#define END_LOGGING_PIN 3

// SD-Card
#define SD_CHIP_SELECT 10       // Primary SPI Chip Select for SD
#define SD_WRITE_SIZE 512       // SD card Wrie Block size, 512 bytes
#define FILE_BUFFER_SIZE 16384  // Allocate 16KBytes of RAM for UBX message storage

// GNSS
#define __NAV_RATE__ 1
#define __MEAS_RATE__ 4
#define __FIX_TYPE__ 3
#define MAX_PACKET_PAYLOAD_SIZE 2616 // RAW messages are big...
#define PACKET_PAYLOAD_SIZE  512 // Adjusted for writesize of SD card

// OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
#define OLED_ADDRESS 0x3D
#define ROW 10

/*============================CLASS DEFINITIONS===============================*/
/**
 * @brief A custom packet function that depicts how to send custom payloads to
 *        the GNSS module. Used in this case to fetch the hardware & softwave
 *        versions.
 *
 * @param maxWait default value
 * @return true
 * @return false
 */


class SFE_UBLOX_GPS_ADD : public SFE_UBLOX_GNSS{
   public:
       struct minfoStructure  {
           char swVersion[30];
           char hwVersion[10];
           uint8_t extensionNo = 0;
           char extension[10][30];
       } minfo;

       bool getModuleInfo(uint16_t maxWait){
         this->minfo.hwVersion[0] = 0;
         this->minfo.swVersion[0] = 0;
         for (int i = 0; i < 10; i++)
             this->minfo.extension[i][0] = 0;
         this->minfo.extensionNo = 0;

         // Let's create our custom packet
         uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes

         // setPacketCfgPayloadSize tells the library how many bytes our customPayload can hold.
         // If we call it here, after the .begin, the library will attempt to resize the existing 256 byte payload buffer
         // by creating a new buffer, copying across the contents of the old buffer, and then delete the old buffer.
         // This uses a lot of RAM and causes the code to fail on the ATmega328P. (We are also allocating another 341 bytes for minfo.)
         // To keep the code ATmega328P compliant - don't call setPacketCfgPayloadSize here. Call it before .begin instead.
         //myGNSS.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

         // The next line creates and initialises the packet information which wraps around the payload
         ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

         // The structure of ubxPacket is:
         // uint8_t cls           : The message Class
         // uint8_t id            : The message ID
         // uint16_t len          : Length of the payload. Does not include cls, id, or checksum bytes
         // uint16_t counter      : Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
         // uint16_t startingSpot : The counter value needed to go past before we begin recording into payload array
         // uint8_t *payload      : The payload
         // uint8_t checksumA     : Given to us by the module. Checked against the rolling calculated A/B checksums.
         // uint8_t checksumB
         // sfe_ublox_packet_validity_e valid            : Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
         // sfe_ublox_packet_validity_e classAndIDmatch  : Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID

         // sendCommand will return:
         // SFE_UBLOX_STATUS_DATA_RECEIVED if the data we requested was read / polled successfully
         // SFE_UBLOX_STATUS_DATA_SENT     if the data we sent was writted successfully (ACK'd)
         // Other values indicate errors. Please see the sfe_ublox_status_e enum for further details.

         // Referring to the u-blox M8 Receiver Description and Protocol Specification we see that
         // the module information can be read using the UBX-MON-VER message. So let's load our
         // custom packet with the correct information so we can read (poll / get) the module information.

         customCfg.cls = UBX_CLASS_MON; // This is the message Class
         customCfg.id = UBX_MON_VER;    // This is the message ID
         customCfg.len = 0;             // Setting the len (length) to zero let's us poll the current settings
         customCfg.startingSpot = 0;    // Always set the startingSpot to zero (unless you really know what you are doing)

         // Now let's send the command. The module info is returned in customPayload

         if (sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
           return (false); //If command send fails then bail
         // Now let's extract the module info from customPayload

         uint16_t position = 0;
         for (int i = 0; i < 30; i++){
           minfo.swVersion[i] = customPayload[position];
           position++;
         }
         for (int i = 0; i < 10; i++){
           minfo.hwVersion[i] = customPayload[position];
           position++;
         }

         while (customCfg.len >= position + 30){
           for (int i = 0; i < 30; i++){
             minfo.extension[minfo.extensionNo][i] = customPayload[position];
             position++;
           }
           minfo.extensionNo++;
           if (minfo.extensionNo > 9)
             break;
         }

         return (true); //Success!
       }
};

/*=============================GLOBALS========================================*/
// Files
File rxmFile;
File timeSlotFile;

// Values
volatile boolean monitorAvailable = false;
volatile boolean endLoggingState = false;
volatile boolean logDataState = false;
uint16_t posCounter = 0;

// Extended SFE_UBLOX_GPS class
SFE_UBLOX_GPS_ADD myGNSS;

// SSD1306 SPI dislay.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
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

  // Enable automatic RXM RAWX messages & enable RXM RAWX datalogging
  myGNSS.setAutoRXMRAWX(true, false);
  myGNSS.logRXMRAWX(true);

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

  // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
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

  // Check for a new Timeslot filename
  id = 1;
  char timeFileName[12] = "t001.txt";
  while(SD.exists(timeFileName) && id < 1000){
    id++;
    sprintf(timeFileName, "t%03d.txt", id);
  }
  if (id > 999){
    return false;
  }

  // Open new Timeslot file
  timeSlotFile = SD.open(timeFileName, O_CREAT | O_WRITE);
  if(!timeSlotFile) {
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

  sprintf(buffer, "%u/%02d/%02d %02d:%02d:%02d.000\n",
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
