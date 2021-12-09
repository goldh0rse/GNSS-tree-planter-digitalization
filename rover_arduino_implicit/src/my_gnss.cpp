/**
 * @file my_gnss.cpp
 * @author Klas Holmberg (hed16khg@cs.umu.com)
 * @brief  The function definitions of the SFE_UBLOX_GPS_ADD class
 * @version 0.1
 * @date 2021-12-09
 *
 */
#include "my_GNSS.h"


/**
 * @brief A custom packet function that depicts how to send custom payloads to
 *        the GNSS module. Used in this case to fetch the hardware & softwave
 *        versions.
 *
 * @param maxWait default value
 * @return true
 * @return false
 */
bool SFE_UBLOX_GPS_ADD::getModuleInfo(uint16_t maxWait){
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
    this->minfo.swVersion[i] = customPayload[position];
    position++;
  }
  for (int i = 0; i < 10; i++){
    this->hwVersion[i] = customPayload[position];
    position++;
  }

  while (customCfg.len >= position + 30){
    for (int i = 0; i < 30; i++){
      this->extension[this->minfo.extensionNo][i] = customPayload[position];
      position++;
    }
    this->minfo.extensionNo++;
    if (this->minfo.extensionNo > 9)
      break;
  }

  return (true); //Success!
}
