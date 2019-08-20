/** ***************************************************************************
 * @file driverGPSAllentrance.c GPS Driver for Internal/GPS NAV.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *  This module provides high level GPS management,
 *  based on product type and GPS receiver. This module
 *  is interfaced with NAV processing and other specific GPS
 *  process files. The functions are provided in this files are common
 *  for all GPS protocol. Protocol-specific process is provided in other GPS files
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/


#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define   GPS_BAUD_RATE_UNDEFINED -1
#include "uart.h"    // for uart_IrqOnOff()
#include "driverGPS.h"
#include "platformAPI.h"

#include "gps.h" // the streaming flag
#include "platformAPI.h"
#include "BITStatus.h"
#include "osapi.h"

/// GPS data struct
// to change to NMEA then pass GPS out debug port: un-comment this and
// set LOGGING_LEVEL to LEVEL_STREAM. Nugget must have GPS config file loaded
// output stream is at 115200 baud even though the internal gps may be at 4800
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM // [1]
GpsData_t gGpsData = {
    .GPSbaudRate     = 4800,
    .GPSProtocol     = NMEA_TEXT,
    //.sirfInitialized = FALSE,
};

GpsData_t *gGpsDataPtr = &gGpsData;

float getGpsHdop() {return gGpsDataPtr->HDOP;}

// local functions
void    _configGPSReceiver(GpsData_t* GPSData);
void    _parseGPSMsg(int *numInBuffPrt, GpsData_t *GPSData);
int16_t _getIndexLineFeed(uint16_t numInBuff, unsigned *msgLength);
void    _setGPSMessageSignature(GpsData_t* GPSData);
void    _userCmdBaudProtcol(GpsData_t* GPSData);

void loadGpsCommSettings(GpsData_t* GPSData)
{
    GPSData->GPSAUTOSetting = 0;
  	_setGPSMessageSignature(GPSData);
	GPSData->GPSConfigureOK = 0;
}

BOOL  SetGpsBaudRate(int rate, int fApply)
{

    switch (rate){
        case 4800:
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 230400:
            break;
        default:
            return FALSE;
    }

    if(fApply){
        gGpsData.GPSbaudRate = rate;
    }

    return TRUE;
}
BOOL  SetGpsProtocol(int protocol, int fApply)
{
    switch(protocol){
        case NMEA_TEXT:
        case NOVATEL_BINARY:
        case UBLOX_PVT:
            break;
        default:
            return FALSE;
    }
    if(fApply){
        gGpsData.GPSProtocol = protocol;
    }
    return TRUE;
}


/** ****************************************************************************
 * @name initGPSDataStruct initializes data GPS structure using all configuration
 *       structure that is read from EEPROM.
 * @athor Dong An
 * @param [in] baudRate - enumeration to translate
 * @retval actual baud rate value
 ******************************************************************************/
void initGPSDataStruct(void)
{
	gGpsDataPtr = &gGpsData;
    memset((void*)&gGpsData, 0,  sizeof(GpsData_t));
    gGpsData.GPSbaudRate = 4800;
    gGpsData.GPSProtocol = NMEA_TEXT;
}


/** ****************************************************************************
 * @name initGPSHandler initializes data GPS structure using all configuration
 *       structure that is read from EEPROM.
 * @athor Dong An
 * @param [in] baudRate - enumeration to translate
 * @retval actual baud rate value
 ******************************************************************************/
void initGPSHandler(void)
{
 #ifdef GPS
    gGpsDataPtr->HDOP = 50.0f;
	gGpsDataPtr->GPSTopLevelConfig |= (1 << HZ2); // update to change internal (ublox) GPS to 2Hz
    /// Configure GPS structure, from Flash (EEPROM)
	loadGpsCommSettings(gGpsDataPtr);
    initGpsUart(gGpsDataPtr->GPSbaudRate);
#endif
}


BOOL _handleGpsMessages(GpsData_t *GPSData)
{
	static uint8_t gpsUartBuf[100]; 
    static uint8_t gpsMsg[MAX_MSG_LENGTH];
    static int bytesInBuffer = 0;
    unsigned char tmp;
	unsigned static int  pos = 0;
    
	while(1){
        if(!bytesInBuffer){
            bytesInBuffer = uart_read(gpsSerialChan, gpsUartBuf, sizeof (gpsUartBuf));
            if(!bytesInBuffer){
                return 0; // nothing to do
            }
            GPSData->rxCounter += bytesInBuffer;
            pos = 0; 
        }
        tmp = gpsUartBuf[pos++];
        bytesInBuffer--;
        switch(GPSData->GPSProtocol){
            case NMEA_TEXT: 
                parseNMEAMessage(tmp, gpsMsg, GPSData);
                break; 
            case NOVATEL_BINARY:
                parseNovotelBinaryMessage(tmp, gpsMsg, GPSData);
                break;
            case UBLOX_PVT:
                parseUbloxPVTMessage(tmp, gpsMsg, GPSData);
                break;
            default:
                break;
        }
    }
}
/* end _handleGpsMessages */


/** ****************************************************************************
 * @name GPSHandler GPS stream data and return GPS data for NAV algorithms
 * @athor Dong An
 * @retval N/A
 * @brief The _configGPSReceiver() has a 2 second delay in it that causes a
 * DAQ restart in the data acquisition task
 ******************************************************************************/
void GPSHandler(void)
{
	gGpsDataPtr->Timer100Hz10ms     = getSystemTime();      ///< get system clock ticks
    gGpsDataPtr->isGPSBaudrateKnown = 1;

    // parse moved to top since is always called after init
  	if (gGpsDataPtr->GPSConfigureOK > 0 ) {              // Configuration is completed
        _handleGpsMessages(gGpsDataPtr);
    } else { ///< configure GPS receiver if needed: OK < 0
		_configGPSReceiver(gGpsDataPtr);
    }
}

// only used by internal (Origin) GPS to sequence through configurations
const struct { int baudRate; enumGPSProtocol protocol; } options[] = {
             { 4800,    NMEA_TEXT},   // most likely: cold boot
             { 230400,  SIRF_BINARY}, // simple reset
             { 4800,    SIRF_BINARY}, // shouldn't happen but there you go
             { 115200,  NMEA_TEXT} }; // external GPS

/** ****************************************************************************
 * @name _configGPSReceiver LOCAL configure GPS parses GPS stream data and fills
 *       GPS data into NAV filter structure.
 * @brief All GPS receiver configuration commands should be sent here if needed.
 * @author Dong An
 * @param [out] bytesFromBuffer - data from GPS data buffer
 * @param [out] GPSData - gps data structure
 * @retval N/A
 ******************************************************************************/
void _configGPSReceiver(GpsData_t     *GPSData)
{

    _handleGpsMessages(GPSData); // load GPSdata structure

	if (GPSData->GPSProtocol == SIRF_BINARY) { ///< 0 - internal GPS receiver is Origin SiRF binary or NMEA
         configSiRFGPSReceiver(GPSData, 230400);
	    _setGPSMessageSignature(GPSData);
	} else { 
         // assuming that rate and protocol configured properly from the beginning
         GPSData->GPSConfigureOK = 1;
                }
}

/** ****************************************************************************
 * @name _setGPSMessageSignature LOCAL input GPS message spec (or feature) into
 *       GPS structure
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 ******************************************************************************/
void _setGPSMessageSignature(GpsData_t* GPSData)
{
	switch(GPSData->GPSProtocol) {
        case SIRF_BINARY :
			GPSData->GPSMsgSignature.GPSheader              = SIRF_BINARY_HEADER; // 0xa0a2
			GPSData->GPSMsgSignature.GPSheaderLength        = 2;
			GPSData->GPSMsgSignature.lengthOfHeaderIDLength = 4; // header len + 2 payload len
			GPSData->GPSMsgSignature.crcLength              = 2;
			GPSData->GPSMsgSignature.binaryOrAscii          = 0;
		break;
        case NOVATEL_BINARY :
			GPSData->GPSMsgSignature.GPSheader              = NOVATEL_OME4_BINARY_HEADER; // 0xAA4412
	        GPSData->GPSMsgSignature.GPSheaderLength        = 3;
			GPSData->GPSMsgSignature.lengthOfHeaderIDLength = 5;
			GPSData->GPSMsgSignature.binaryOrAscii          = 0;
			GPSData->GPSMsgSignature.crcLength              = 4;
		break;
		case NMEA_TEXT :
			GPSData->GPSMsgSignature.GPSheader              = NMEA_HEADER; // "$GP"
	        GPSData->GPSMsgSignature.GPSheaderLength        = 3;
			GPSData->GPSMsgSignature.binaryOrAscii          = 1;
			GPSData->GPSMsgSignature.lengthOfHeaderIDLength = 3;
		    break;
        default:
            break;
	}
    GPSData->GPSMsgSignature.startByte = GPSData->GPSMsgSignature.GPSheader >> ( (GPSData->GPSMsgSignature.GPSheaderLength * 8) - 8);
}






