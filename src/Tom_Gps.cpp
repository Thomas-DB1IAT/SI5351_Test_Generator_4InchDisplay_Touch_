#include <HardwareSerial.h>
#include "Tom_Gps.h"
#include <Arduino.h>

uint8_t LocatorExtention = 2;  // 0..2   JN48DX 50 MS


char UBLOX_M7_Set_Time_5_2Hz_10Hz[] {
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x0A,0x00,
  0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0xD3,0x9E
};

// U-BLOX NEO 7M
char UBLOX_Set_Time_5_10KHz[] {
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x10,0x27,0x00,0x00,
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0x35,0xCC,
  //0xCD,0xCC,0xCC,0xCC,0xCD,0xCC,0xCC,0xCC,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0x97,0x98,

  //B5 62 06 31 20 00 00 01 00 00 00 00 00 00 10 27 00 00 10 27 00 00 
  //0xCD,0xCC,0xCC,0xCC,0xCD,0xCC,0xCC,0xCC,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0x97,0x98,
};  

// U-BLOX NEO 7M
char UBLOX_Set_Time_5_100KHz[] {
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0xA0,0x86,0x01,0x00,0xA0,0x86,0x01,0x00,
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0x15,0x4A,
};  

// U-BLOX NEO 7M
char UBLOX_Set_Time_5_1MHz[] {
  // Set Time5 Pulse to 1.000000 MHz and 50% Duty
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x32,0x00,0x00,0x00,0x40,0x42,0x0F,0x00,0x40,0x42,0x0F,0x00,  
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0xEF,0x00,0x00,0x00,0x9B,0x4A,              
};

// U-BLOX NEO 7M
char UBLOX_Set_Time_5_8MHz[] {
  // Set Time5 Pulse to 8.000000 MHz and 50% Duty
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x7A,0x00,0x00,0x12,0x7A,0x00,
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0xDF,0xAA,                          
};

// U-BLOX NEO 7M
char UBLOX_Set_Time_5_10MHz[] {
  // Set Time5 Pulse to 10.000000 MHz and 50% Duty
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x96,0x98,0x00,0x80,0x96,0x98,0x00,
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0x23,0x02,                          
};

char UBLOX_M7_Set_TIMEUTC[] {
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA 
};


// Send GPS Configuration to UBlox  NEO6M
void SendMemToGps(char GpsConf[], int l, HardwareSerial* SerialGps) {  
  for(int i = 0; i < l; i++) {                        
    SerialGps->write(GpsConf[i]);
    //Serial.print( pgm_read_byte(GpsConf+i), HEX );
    delay(10); // 10
  }
}

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize, UBXMessage* ubxMessage) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(ubxMessage))[i]; //&
    CK[1] += CK[0];
  }
}


// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader, UBXMessage* ubxMessage) {
  unsigned char* ptr = (unsigned char*)(ubxMessage); // &
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}


// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS(HardwareSerial* SerialGps, UBXMessage* ubxMessage) {
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while ( SerialGps->available() ) {
    byte c = SerialGps->read(); 
    //Serial.print(c, HEX); Serial.print(""); 
    
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(ubxMessage))[fpos-2] = c; //&

      fpos++;
      // Got 4 Bytes, check Message-Type we are readning
      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_POSLLH_HEADER, ubxMessage) ) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(UbxNavPosllh);
          //Serial.print("NAV_POSLLH  ");
        }
        else if ( compareMsgHeader(NAV_STATUS_HEADER, ubxMessage) ) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(UbxNavStatus);
          //Serial.print("NAV_STATUS  ");
        }
        else if ( compareMsgHeader(NAV_PVT_HEADER, ubxMessage) ) {
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(UbxNavPvt);
          //Serial.print("NAV_PVT     ");
        }
        else {
          // unknown message type, bail
          fpos = 0;
          //Serial.println("Unknown Message Type");
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize, ubxMessage);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          Serial.println("Checksum Not OK");
          // Checksum doesn't match, reset to beginning state and try again.
          return currentMsgType; 
          fpos = 0; 
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          //Serial.println("Checksum OK");
          payloadSize = 4;
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
        Serial.println("EndOfData");
      }
    }
  }
  return MT_NONE;
}

String LatLngToLocator(double Lat, double Long, int Ext) {
  //int v;
  String locator = "";

  Lat += 90;
  Long += 180;

  locator += (char) ('A' + floor(Long / 20));
  locator += (char) ('A' + floor(Lat / 10));
  Long = remainder(Long, 20);
  if (Long < 0) Long += 20;
  Lat = remainder(Lat, 10);
  if (Lat < 0) Lat += 10;

  locator += (char) ('0' + floor(Long / 2));
  locator += (char) ('0' + floor(Lat / 1));
  Long = remainder(Long, 2);
  if (Long < 0) Long += 2;
  Lat = remainder(Lat, 1);
  if (Lat < 0) Lat += 1;

  locator += (char) ('A' + floor(Long * 12));
  locator += (char) ('A' + floor(Lat * 24));
  Long = remainder(Long, (double) 1 / 12);
  if (Long < 0) Long += (double) 1 / 12;
  Lat = remainder(Lat, (double) 1 / 24);
  if (Lat < 0) Lat += (double) 1 / 24;

  if (Ext >= 1) {
    locator += (char) ('0' + floor(Long * 120));
    locator += (char) ('0' + floor(Lat * 240));
    Long = remainder(Long, (double) 1 / 120);
    if (Long < 0) Long += (double) 1 / 120;
    Lat = remainder(Lat, (double) 1 / 240);
    if (Lat < 0) Lat += (double) 1 / 240;
  }

  if (Ext >= 2) {
    locator += (char) ('A' + floor(Long * 120 * 24));
    locator += (char) ('A' + floor(Lat * 240 * 24));
    Long = remainder(Long, (double) 1 / 120 / 24);
    if (Long < 0) Long += (double) 1 / 120 / 24;
    Lat = remainder(Lat, (double) 1 / 240 / 24);
    if (Lat < 0) Lat += (double) 1 / 240 / 24;
  }
  return locator;
}

//******************************************************************
// Compute the MH-LOCATOR form latiture longitude
//******************************************************************
String compute_Locator(double latitude, double longitude) {
  return LatLngToLocator(latitude, longitude, LocatorExtention);
}
