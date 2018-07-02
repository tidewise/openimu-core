/** ***************************************************************************
 * @file   commands.c callback functions from the commandTable
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Commands available to the commandLine.c shell
 ******************************************************************************/
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


#include <math.h> // floating point sqrt
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "commandLine.h"
#include "algorithmAPI.h"
#include "osapi.h"


#define LOGGING_LEVEL LEVEL_INFO
#include "debug.h"
#include "dmu.h"
#include "Indices.h"
#include "gpsAPI.h"
#include "Indices.h"
#include "timer.h"
//#include "xbowsp_version.h"
#include "platformAPI.h"


// for user-defined baud rate (terminal)
#include "debug_usart.h"

char versionString[100];
// Display firmware version
void CmdVersion(uint32_t data)
{
    int i;
    char *ptr = unitVersionString();
    // version is put in using 16 bit chars from TI
    for (i = 0; i < sizeof(versionString); i++) {
        versionString[i] = ptr[i*2];
    }
    versionString[i] = 0;
    DEBUG_STRING(versionString);
    DEBUG_ENDLINE();
} // End of display firmware version

/** ***************************************************************************
 * @name CmdReadAccelerometer() Read accelerometer
 * @brief  
 *
 * @retval N/A
 ******************************************************************************/
void CmdReadAccelerometer(uint32_t readInGs)
{
    double   readings[NUM_AXIS];
    char     str[30];

    OSDisableHook();
    for(int i = 0; i < NUM_AXIS; i++ ){
       readings[i] = pScaledSensors[XACCEL+i]; 
    }
    OSEnableHook();
    
    DEBUG_STRING("Reading accelerometers data\r\n");
    sprintf(str, " X = %3.5lf\r\n", readings[0]);
    DEBUG_STRING(str);
    sprintf(str, " Y = %3.5lf\r\n", readings[1]);
    DEBUG_STRING(str);
    sprintf(str, " Z = %3.5lf\r\n", readings[2]);
    DEBUG_STRING(str);

}

/** ***************************************************************************
 * @name CmdReadGyro() Read gyro data
 * @brief  
 *
 * @retval N/A
 ******************************************************************************/
void CmdReadGyro(uint32_t readInGs)
{
    double   readings[NUM_AXIS];
    char     str[30];

    OSDisableHook();
    for(int i = 0; i < NUM_AXIS; i++ ){
       readings[i] = pScaledSensors[XRATE+i]; 
    }
    OSEnableHook();
    
    DEBUG_STRING("Reading gyro data\r\n");
    sprintf(str, " X = %3.5lf\r\n", readings[0]);
    DEBUG_STRING(str);
    sprintf(str, " Y = %3.5lf\r\n", readings[1]);
    DEBUG_STRING(str);
    sprintf(str, " Z = %3.5lf\r\n", readings[2]);
    DEBUG_STRING(str);

}

/** ***************************************************************************
 * @name CmdReadMagnetometer. Read magnetometer data
 * @brief  
 *
 * @retval N/A
 ******************************************************************************/
void CmdReadMagnetometer(uint32_t readInGs)
{
    double   readings[NUM_AXIS];
    char     str[30];

    OSDisableHook();
    for(int i = 0; i < NUM_AXIS; i++ ){
       readings[i] = pScaledSensors[XMAG+i]; 
    }
    OSEnableHook();
    
    DEBUG_STRING("Reading magnetometer data\r\n");
    sprintf(str, " X = %3.5lf\r\n", readings[0]);
    DEBUG_STRING(str);
    sprintf(str, " Y = %3.5lf\r\n", readings[1]);
    DEBUG_STRING(str);
    sprintf(str, " Z = %3.5lf\r\n", readings[2]);
    DEBUG_STRING(str);

}


#include "commandTable.h"
