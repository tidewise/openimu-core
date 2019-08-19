/** ***************************************************************************
 * @file processUbloxGPS.c Processing for ublox GPS receiver.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *	 This file includes all specific processing, including configuring, ..,
 *   for ublox GPS receiver.
 *****************************************************************************/
/*******************************************************************************
Copyright 2019 TideWise Ltda

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
#include <stdlib.h>

#include "driverGPS.h"

static void _computeUbloxCheckSumCrc(uint8_t      *msg,
                                     unsigned int msgLength,
                                     unsigned int *cCKA,
                                     unsigned int *cCKB);

static void processUbloxPVTMessage(uint8_t      *msg,
                            unsigned int  msgLength,
                            GpsData_t     *GPSData);

int parseUbloxPVTMessage(uint8_t inByte, uint8_t *gpsMsg, GpsData_t *GPSData)
{
    static int state = 0;
    static unsigned int len = 0;
    static unsigned int packetLen;

    if (len == MAX_MSG_LENGTH) {
        GPSData->overflowCounter++;
        goto reset;
    }

    gpsMsg[len++] = inByte;

    switch(state) {
        case 0: // wait for sync1
            if (inByte == 0xB5)
                state = 1;
            else
                goto reset;
            return 0;

        case 1: // wait for sync2
            if (inByte == 0x62)
                state = 2;
            else
                goto reset;
            return 0;

        case 2: // wait for class id
            if (inByte == 0x01)
                state = 3;
            else
                goto reset;
            return 0;

        case 3: // wait for message id
            if (inByte == 0x07)
                state = 4;
            else
                goto reset;
            return 0;
        
        case 4: // wait for payload length
            if (len == 6) {
                packetLen = 8 + (gpsMsg[4] | (gpsMsg[5] << 8));
                state = 5;
            }
            return 0;

        case 5: // wait for packet length
            if (len < packetLen) {
                return 0;
            }
    }

    state = 0;
    len = 0;

    unsigned int expectedChecksum[2];
    _computeUbloxCheckSumCrc(gpsMsg, packetLen,
                             expectedChecksum, expectedChecksum + 1);

    if (expectedChecksum[0] == gpsMsg[packetLen - 2] ||
        expectedChecksum[1] == gpsMsg[packetLen - 1]) {
        processUbloxPVTMessage(gpsMsg, packetLen, GPSData);
        return 0;
    }
    else 
        return 1;

reset:
    state = 0;
    len = 0;
    return 1;
} 

uint16_t decode_u2(uint8_t const* bytes)
{
    return bytes[0] | (((uint16_t)bytes[1]) << 8);
}
uint32_t decode_u4(uint8_t const* bytes)
{
    return bytes[0] |
           (((uint32_t)bytes[1]) << 8) |
           (((uint32_t)bytes[2]) << 16) |
           (((uint32_t)bytes[1]) << 24);
}
int32_t decode_i4(uint8_t const* bytes)
{
    uint32_t u4 = decode_u4(bytes);
    return *(int32_t const*)(&u4);
}

void normalizeLatLon(GpsData_t* data) {
    if (data->lat > 0)
        data->latSign = 1;
    else
    {
        data->lat *= -1;
        data->latSign = -1;
    }

    if (data->lon > 0)
        data->lonSign = 1;
    else
    {
        data->lon *= -1;
        data->lonSign = -1;
    }
}

void processUbloxPVTMessage(uint8_t      *msg,
                            unsigned int  msgLength,
                            GpsData_t     *GPSData)
{
    uint8_t const* payload = msg + 6;

    GPSData->gpsValid = (payload[20] == 3 || payload[20] == 4);
    GPSData->itow = decode_u4(payload);
    GPSData->GPSmonth = payload[6];
    GPSData->GPSday = payload[7];
    GPSData->GPSyear = decode_u2(payload + 4);
    GPSData->GPSHour = payload[8];
    GPSData->GPSMinute = payload[9];
    GPSData->GPSSecond = payload[10];
    GPSData->GPSSecondFraction = ((double)decode_i4(payload + 16)) * 1e-9;

    if (!GPSData->gpsValid) {
        return;
    }

    GPSData->lat  = ((long double)decode_i4(payload + 28)) * 1e-7;
    GPSData->lon  = ((long double)decode_i4(payload + 24)) * 1e-7;
    normalizeLatLon(GPSData); // fills the latSign/lonSign
    GPSData->vNed[0]   = ((double)decode_i4(payload + 48)) * 1e-3;
    GPSData->vNed[1]   = ((double)decode_i4(payload + 52)) * 1e-3;
    GPSData->vNed[2]   = ((double)decode_i4(payload + 56)) * 1e-3;
    // Note: time-related fields are filled before the valid
    // test as we can have a time solution before we have
    // a position solution

    GPSData->updateFlagForEachCall |= 1<<GOT_GGA_MSG;
    GPSData->updateFlagForEachCall |= 1<<GOT_VTG_MSG;
    GPSData->totalGGA++;
    GPSData->totalVTG++;

    GPSData->trueCourse = ((double)decode_i4(payload + 64)) * 1e-5;
    GPSData->rawGroundSpeed = ((double)decode_i4(payload + 60)) * 1e-3;

    GPSData->alt  = ((long double)decode_i4(payload + 36)) * 1e-3;
    GPSData->filteredAlt  = GPSData->alt;
    GPSData->altEllipsoid = ((long double)decode_i4(payload + 32)) * 1e-3;
    // Note: time-related fields are filled before the valid
    // test as we can have a time solution before we have
    // a position solution

    // Unused GPSData->GPSFix = ;
    GPSData->HDOP = ((float)decode_u2(payload + 76)) * 1e-2;
    GPSData->GPSVelAcc = ((double)decode_u4(payload + 68)) * 1e-3;
    // GPSData->GPSStatusWord = ;
    GPSData->GPSHorizAcc = ((double)decode_u4(payload + 40)) * 1e-3;
    GPSData->GPSVertAcc = ((double)decode_u4(payload + 44)) * 1e-3;

    GPSData->numSatelites = payload[23];
}

/** ****************************************************************************
 * @name _computeUbloxCheckSumCrc compute checksum of a complete ubblox binary
 *       message.
 * @author Dong An
 * @param [in] msg- message
 * @param [in] msgLength - input buffer length
 * @param [out] cCKA - checksum A
 * @param [out] cCKB - checksum B
 * @retval N/A
 ******************************************************************************/
void _computeUbloxCheckSumCrc(uint8_t      *msg,
                              unsigned int msgLength,
                              unsigned int *cCKA,
                              unsigned int *cCKB)
{
    int          i;
    int          CKRange;
    unsigned int checksumACalcu = 0;
    unsigned int checksumBCalcu = 0;

    CKRange = msgLength - 4;  ///exclude header and crc itself
    for (i = 0; i < CKRange; i++)		{
        checksumACalcu = checksumACalcu + (msg[i+2] & 0xFF);
        checksumACalcu = checksumACalcu& 0xFF;

        checksumBCalcu = checksumBCalcu + checksumACalcu;
        checksumBCalcu = checksumBCalcu & 0xFF;
    }
    *cCKA = checksumACalcu;
    *cCKB = checksumBCalcu;
}

