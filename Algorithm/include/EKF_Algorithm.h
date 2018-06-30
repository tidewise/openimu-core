/*
 * File:   EKF_Algorithm.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:23 AM
 */

#ifndef _EKF_ALGORITHM_H_
#define _EKF_ALGORITHM_H_

#include <stdint.h>

#include "GlobalConstants.h"
#include "StateIndices.h"
#include "UpdateMatrixSizing.h"  // used to specify the size of the update vectors

#include "Indices.h"

// Changed to 1e-2 on Sep 13, 2016
#define INIT_P 0.01


// Global Kalman Filter structure
typedef struct {
    // States
    real Velocity_N[3];
    real Position_N[3];
    real rateBias_B[3];
    real accelBias_B[3];

    real stateUpdate[NUMBER_OF_EKF_STATES];

    // Prediction variables: P = FxPxFTranspose + Q
    real Q[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real F[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real P[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];

    real correctedRate_B[3];
    real correctedAccel_B[3];
    real aCorr_N[3];
    real aMotion_N[3];

    real R_BinN[3][3];   // 321-Rotation matrix

    real quaternion[4],  measuredQuaternion[4], quaternion_Past[4];
    real eulerAngles[3], measuredEulerAngles[3];
    real attitudeError[3];

    // Update variables: S = HxPxHTranspose + R
    // DEBUG H is 3x16 or 9x16 (AHRS vs INS)
    real nu[9];

    real S[3][3], SInverse[3][3];
    real H[3][NUMBER_OF_EKF_STATES];
    real R[9][9];
    real K[NUMBER_OF_EKF_STATES][3];

    double llaDeg[3];
//    double Position_E[3];

    real wTrueTimesDtOverTwo[3];

    real turnSwitchMultiplier;
} KalmanFilterStruct;

extern KalmanFilterStruct gKalmanFilter;

/* Global Algorithm structure  */
typedef struct {
    // Sensor readings in the body-frame (B)
    double            accel_B[3];      // [g]
    double            angRate_B[3];    // [rad/s]
    double            magField_B[3];   // [G]

    // Used to generate the system ICs
    // GPS stuff still needed
} EKFInputDataStruct;

extern EKFInputDataStruct gEKFInputData; // 


/* Global Algorithm structure  */
typedef struct {
    // Algorithm states (15 states)
    double            position_N[3];
    double            velocity_N[3];
    double            quaternion_BinN[4];
    double            angRateBias_B[3];
    double            accelBias_B[3];

    // Derived variables
    double            eulerAngs_BinN[3];
    double            corrAngRates_B[3];
    double            corrAccel_B[3];

    // Operational states
    uint8_t           opMode;
    uint8_t           turnSwitchFlag;
    uint8_t           linAccelSwitch;
} EKFOutputDataStruct;

extern EKFOutputDataStruct gEKFOutputData;

void EKF_Algorithm(void);
void enableFreeIntegration(BOOL enable);

// Getters for data extraction from the EKF output data structure
void GetEKF_Attitude_EA(real *EulerAngles);
void GetEKF_Attitude_Q(real *Quaternions);
void GetEKF_CorrectedAngRates(real *CorrAngRates_B);
void GetEKF_CorrectedAccels(real *CorrAccels_B);
void GetEKF_EstimatedAngRateBias(real *AngRateBias_B);
void GetEKF_EstimatedAccelBias(real *AccelBias_B);
void GetEKF_EstimatedPosition(real *Position_N);
void GetEKF_EstimatedVelocity(real *Velocity_N);

void GetEKF_OperationalMode(uint8_t *EKF_OperMode);
void GetEKF_OperationalSwitches(uint8_t *EKF_LinAccelSwitch, uint8_t *EKF_TurnSwitch);


#endif /* _EKF_ALGORITHM_H_ */

