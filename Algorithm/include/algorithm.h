/*****************************************************************************
* @name xbowsp_algorithm.h
*
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
*   -Algorithm data structure used in sensor calibration and communication
* protocols with outside world.
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

#ifndef XBOWSP_ALGORITHM_H
#define XBOWSP_ALGORITHM_H

#include "GlobalConstants.h"

//#include "BITStatus.h" // BITStatusStruct
#include "Indices.h"
#include "algorithmAPI.h"

// Define the algorithm states
#define STABILIZE_SYSTEM    0
#define INITIALIZE_ATTITUDE 1
#define HIGH_GAIN_AHRS      2
#define LOW_GAIN_AHRS       3
#define INS_SOLUTION        4


// Specify the minimum state times (in seconds)
#define STABILIZE_SYSTEM_DURATION    0.36    // [sec]
#define INITIALIZE_ATTITUDE_DURATION 0.64    // ( 1.0 - 0.36 ) [sec]
#define HIGH_GAIN_AHRS_DURATION      30.0    // 60.0 * SAMPLING_RATE
#define LOW_GAIN_AHRS_DURATION       30.0    // 30.0 * SAMPLING_RATE

typedef struct {
    uint32_t Stabilize_System;      // SAMPLING_RATE * 0.36
    uint32_t Initialize_Attitude;   // SAMPLING_RATE * ( 1.0 - 0.36 )
    uint32_t High_Gain_AHRS;        // 60.0 * SAMPLING_RATE
    uint32_t Low_Gain_AHRS;         // 30.0 * SAMPLING_RATE
} DurationStruct;

typedef struct {
    real positionError;
    real velocityError;
    real attitudeError;
} InnovationStruct;

typedef struct {
    int32_t Max_GPS_Drop_Time;   // [msec]
    int32_t Max_Rest_Time_Before_Drop_To_AHRS;   // [msec]
    int32_t Declination_Expiration_Time;   // [msec]

    uint16_t Free_Integration_Cntr;   // [count]
//#define LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS     60000 // 60000 [ msec ] = 60 [ sec ]
//#define LIMIT_DECL_EXPIRATION_TIME                  60000  // 60,000 [ counts ] = 10 [ min ]

    real accelSwitch;
    uint32_t linAccelSwitchDelay;
    
    InnovationStruct       Innov;

    //
    real rateAlarm, accelAlarm, magAlarm;
} LimitStruct;

/// specifying how the user sets up the device algorithm
struct algoBehavior_BITS  {        /// bits   description
   uint16_t freeIntegrate     : 1; /// 0
   uint16_t useMag            : 1; /// 1
   uint16_t useGPS            : 1; /// 2 - Not used yet
   uint16_t stationaryLockYaw : 1; /// 3 - Not used yet
   uint16_t restartOnOverRange: 1; /// 4
   uint16_t dynamicMotion     : 1; /// 5 - Not used
   uint16_t rsvd              :10; /// 6:15
};

union AlgoBehavior
{
   uint16_t                 all;
   struct algoBehavior_BITS bit;
};


/* Global Algorithm structure  */
typedef struct {

    // Used to generate the system ICs
    real              accumulatedAccelVector[3];
    real              accumulatedMagVector[3];
    real              averagedAccelVector[3];
    real              averagedMagVector[3];

    // Yaw-Rate (Turn-Switch) filter
    real              filteredYawRate, filteredYawRatePast;
    real              yawRateMeasPast;

    real              attitude[3];              /// from qb2t
    real              leveledMags[3];           /// x and y mags (tangent frame w/o user alignment)
    volatile uint16_t counter;                  /// inc. with every continuous mode output packet
    volatile uint32_t timer;  	                 ///< timer since power up (ms)

    real              downRate;

    real              compassHeading;

    real              RateOutSF, AccelOutSF, MagOutSF, TempOutSF;

    // control the stage of operation for the algorithms
    uint8_t           state;   // takes values from HARDWARE_STABILIZE to INIT_ATTITUDE to HG_AHRS
    uint32_t          stateTimer; // may not need

    int insFirstTime;
    int applyDeclFlag;

    int32_t timeOfLastSufficientGPSVelocity;
    int32_t timeOfLastGoodGPSReading;

    double rGPS0_E[3];
    real  rGPS_N[3];

    int   magAlignUnderway;

    float             tempMisalign[18];

    double              llaRad[3];
    double              rGPS_E[3];
    real R_NinE[3][3];

    uint32_t            itow;
    uint32_t            dITOW;

    uint8_t  callingFreq;
    real                dt;
    real                dtOverTwo;
    real                dtSquared;
    real                sqrtDt;

    // The following variables are used to increase the Kalman filter gain when the
    //   acceleration is very close to one (i.e. the system is at rest)
    real aMag;
    real aFilt_N[3];
    real aMagThreshold;
    uint32_t linAccelSwitchCntr;
    uint8_t linAccelSwitch;
    real R_Mult;

    DurationStruct    Duration;
    LimitStruct       Limit;

    uint8_t linAccelLPFType;
    union   AlgoBehavior Behavior;
    float    turnSwitchThreshold; // 0, 0.4, 10 driving, 1 flying [deg/sec]   0x000d
} AlgorithmStruct;

extern AlgorithmStruct gAlgorithm;

#endif
