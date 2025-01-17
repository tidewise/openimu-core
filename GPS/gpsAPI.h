/** ***************************************************************************
 * @file gps.h GPS Driver for Inertial/GPS NAV.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief This is a generalized GPS interface, taken loosely from the DMU440
 * project possibly implemented using the Origin ORG4475 GPS module (or NovAtel
 * or uBlox) the GPS may communicated via SPI or UART, that is passed in on init
 *  03.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
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

#ifndef GPS_API_H
#define GPS_API_H

#include <stdint.h>
#include "GlobalConstants.h"

void initGPSHandler(void); /// note: should pass in SPI or UART
void initGPSDataStruct(void); /// note: should pass in SPI or UART
void GPSHandler(void);

// FIXME ECW: implement GpsWhoAmI and GpsSelfTest
uint8_t  GpsWhoAmI(uint32_t *whoami); /// returns true if value is as expected
uint8_t  GpsSelfTest();

// debugging stream out the debug port
#define GPS_NO_STREAM         0
#define GPS_NMEA_DEBUG_STREAM 1

#ifndef STREAM_GPS
//#define STREAM_GPS GPS_NO_STREAM
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM
#endif

#ifdef __cplusplus
extern "C" {
#endif

void TaskGps(void const *argument);

#ifdef __cplusplus
}
#endif

typedef struct  {
    uint8_t              gpsValid;   // 1 if data is valid
    uint8_t              updateFlag;    // 1 if contains new data
    uint32_t             updateCount;
    
    int                  latSign;    // latitude sign
    int                  lonSign;    // longitude sign 
    double               latitude;   // latitude ,  degrees 
    double               longitude;  // longitude,  degrees 
    float                vNed[3];   // velocities,  m/s  NED (North East Down) x, y, z
    float                trueCourse; // [deg]
    float                rawGroundSpeed;    // NMEA kph, SiRf m/s - change to m/s
    float                altitude;          // above mean sea level [m]
    double               GPSSecondFraction; 
    float                altEllipsoid; // [km] altitude above ellipsoid for WMM

    uint32_t             itow;         // gps Time Of Week, miliseconds
 
    uint8_t              GPSmonth;     // mm
    uint8_t              GPSday;       // dd
    uint8_t              GPSyear;      // yy last two digits of year
    char                 GPSHour;      // hh
    char                 GPSMinute;    // mm
    char                 GPSSecond;    // ss
    uint8_t              latQ;
    uint8_t              lonQ;
    uint8_t              hgtQ;
    uint8_t              useSigmas;

    float                GPSHorizAcc, GPSVertAcc, GPSHVelAcc, GPSVVelAcc;
} gpsDataStruct_t;

extern gpsDataStruct_t gGPS;

/** ****************************************************************************
 * @name GetGPSData
 * @brief Get GPS data 
 * @param [in] data - pointer to external GPS structure
 * @retval N/A
 ******************************************************************************/
void  GetGPSData(gpsDataStruct_t *data);

/** Whether the current GPS data is valid
 */
uint8_t IsGPSValid();

/** Return the count of valid GPS updates received so far */
uint32_t GetGPSUpdateCount();

/** Return the HDOP of the last received GPS reading */
float GetGPSHDOP();

/** Return the number of bytes received on the GPS UART
 */
uint32_t GetGPSRXCounter();

/** Return the number of times the GPS UART handler overflown
 */
uint16_t GetGPSOverflowCounter();

/** Return the time of last received GPS message
 * 
 * This is set regardless of whether the solution is valid
 * 
 * Returns zero if no message was ever received
 */
uint32_t GetLastReceivedGPS();

/** Return the time of last received GPS message
 * 
 * This is set regardless of whether the solution is valid
 * 
 * Returns zero if no message was ever received
 */
uint32_t GetLastReceivedGPS();

BOOL  SetGpsBaudRate(int rate, int fApply);
BOOL  SetGpsProtocol(int protocol, int fApply);

#endif /* GPS_API_H */
