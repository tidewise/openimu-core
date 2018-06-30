/*
 * File:   EKF_PredictionStage.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:23 AM
 */

#include "GlobalConstants.h"

#include <string.h>   // memset
#include <math.h>      // pow
#include "VectorMath.h"
#include "TransformationMath.h"

#include "EKF_Algorithm.h"

#include "algorithm.h"
#include "PredictFunctions.h"

#include "MatrixMath.h"
#include "QuaternionMath.h"  // QuatNormalize

#include "Indices.h"       // IND
#include "StateIndices.h"  // STATE_IND
#include "TimingVars.h"    // timer
#include "AlgorithmLimits.h"

#include "SensorNoiseParameters.h"

#include "WorldMagneticModel.h"

#include "algorithm.h"
//#include "gyroscope.h"
#include "platformAPI.h"


#ifdef INS_OFFLINE
#include "c:\Projects\software\sim\INS380_Offline\INS380_Offline\SimulationParameters.h"
#endif

//#include "RunLengthEncoding.h"
// F is sparse and has elements in the following locations...
// There may be some more efficient ways of implementing this as this method
//   still performs multiplication with zero values.  (Ask Andrey)
uint8_t RLE_F[ROWS_IN_F][2] = { {  STATE_RX, STATE_VX  },     // Row  0: cols 0,3
                                {  STATE_RY, STATE_VY  },     // Row  1: cols 1,4
                                {  STATE_RZ, STATE_VZ  },     // Row  2: cols 2,5
                                {  STATE_VX, STATE_ABZ },     // Row  3: cols 3,6:9,13:15
                                {  STATE_VY, STATE_ABZ },     // Row  4: cols 4,6:9,13:15
                                {  STATE_VZ, STATE_ABZ },     // Row  5: cols 5,6:9,13:15
                                {  STATE_Q0, STATE_WBZ },     // Row  6: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  7: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  8: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  9: cols 6:12
                                { STATE_WBX, STATE_WBX },     // Row 10: cols 10
                                { STATE_WBY, STATE_WBY },     // Row 11: cols 11
                                { STATE_WBZ, STATE_WBZ },     // Row 12: cols 12
                                { STATE_ABX, STATE_ABX },     // Row 13: cols 13
                                { STATE_ABY, STATE_ABY },     // Row 14: cols 14
                                { STATE_ABZ, STATE_ABZ } };   // Row 15: cols 15

// Q is sparse and has elements in the following locations...
uint8_t RLE_Q[ROWS_IN_F][2] = { {  STATE_RX, STATE_RX  },
                                {  STATE_RY, STATE_RY  },
                                {  STATE_RZ, STATE_RZ  },
                                {  STATE_VX, STATE_VX  },
                                {  STATE_VY, STATE_VY  },
                                {  STATE_VZ, STATE_VZ  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                { STATE_WBX, STATE_WBX },
                                { STATE_WBY, STATE_WBY },
                                { STATE_WBZ, STATE_WBZ },
                                { STATE_ABX, STATE_ABX },
                                { STATE_ABY, STATE_ABY },
                                { STATE_ABZ, STATE_ABZ } };


// Local functions
static void _PredictStateEstimate(void);
static void _PredictCovarianceEstimate(void);

static void _UpdateProcessJacobian(void);
static void _UpdateProcessCovariance(void);

// todo tm20160603 - use filters from filter.h, or move this filter there  (Ask Andrey)
void _FirstOrderLowPass(real *output, real input);

static void _PopulateFilterCoefficients(void);

// 16 States: [ STATE_RX,  STATE_RY,  STATE_RZ, ...
//              STATE_VX,  STATE_VY,  STATE_VZ, ...
//              STATE_Q0,  STATE_Q1,  STATE_Q2,  STATE_Q3, ...
//              STATE_WBX, STATE_WBY, STATE_WBZ, ...
//              STATE_ABX, STATE_ABY, STATE_ABZ ]

// Filter variables (Third-Order BWF w/ default 5 Hz Cutoff)
#define FILTER_ORDER 3

#define CURRENT 0
#define PASTx1  1
#define PASTx2  2
#define PASTx3  3

static real b_AccelFilt[FILTER_ORDER+1];
static real a_AccelFilt[FILTER_ORDER+1];

// Replace this with a fuction that will compute the coefficients so the
//   input is the cutoff frequency in Hertz
#define  NO_LPF              0
#define  TWO_HZ_LPF          1
#define  FIVE_HZ_LPF         2
#define  TEN_HZ_LPF          3
#define  TWENTY_HZ_LPF       4
#define  TWENTY_FIVE_HZ_LPF  5
#define  N_LPF               6

// Floating-point filter variables
static real accelFilt[FILTER_ORDER+1][NUM_AXIS];
static real accelReading[FILTER_ORDER+1][NUM_AXIS];

//EKF_PredictionStage.m
void EKF_PredictionStage(void)
{
    real magFieldVector[3];

    // Propagate the state and covariance estimates
    _PredictStateEstimate();        // x(k+1) = x(k) + f(x(k), u(k))
    _PredictCovarianceEstimate();   // P = F*P*FTrans + Q

    // Extract the predicted Euler angles from the predicted quaternion
    QuaternionToEulerAngles( gKalmanFilter.eulerAngles,
                             gKalmanFilter.quaternion );

    // Filter the yaw-rate at 200 Hz for the TURN-SWITCH (used in the
    //   update stage only -- since that is a ten-hertz routine).  The way this
    //   is coded, the filter function can only be used for filtering yaw-rate
    //   data as the previous input state is saved as a static in the function.
    gAlgorithm.filteredYawRate = gAlgorithm.filteredYawRatePast;
    _FirstOrderLowPass( &gAlgorithm.filteredYawRate,
                        gKalmanFilter.correctedRate_B[Z_AXIS] );
    gAlgorithm.filteredYawRatePast = gAlgorithm.filteredYawRate;

    // ------- Compute the measured euler angles at the desired rate -------
    static BOOL initAccelFilt = true;
    if (initAccelFilt) {
        initAccelFilt = false;

        // Set the filter coefficients based on selected cutoff frequency and sampling rate
        _PopulateFilterCoefficients();

        // Initialize the filter variables (do not need to populate the 0th element
        //   as it is never used)
        for( int i = PASTx1; i <= PASTx3; i++ ) {
            accelReading[i][X_AXIS] = (real)gEKFInputData.accel_B[X_AXIS];
            accelReading[i][Y_AXIS] = (real)gEKFInputData.accel_B[Y_AXIS];
            accelReading[i][Z_AXIS] = (real)gEKFInputData.accel_B[Z_AXIS];

            accelFilt[i][X_AXIS] = (real)gEKFInputData.accel_B[X_AXIS];
            accelFilt[i][Y_AXIS] = (real)gEKFInputData.accel_B[Y_AXIS];
            accelFilt[i][Z_AXIS] = (real)gEKFInputData.accel_B[Z_AXIS];
        }
    }

    // Filter accelerometer readings (Note: a[0] = 1.0 and the filter
    //   coefficients are symmetric)
    //   y = filtered output; x = raw input;
    //
    // a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
    //    b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
    //    b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
    accelFilt[CURRENT][X_AXIS] = b_AccelFilt[CURRENT] * (real)gEKFInputData.accel_B[X_AXIS] +
                                 b_AccelFilt[PASTx1] * ( accelReading[PASTx1][X_AXIS] +
                                                         accelReading[PASTx2][X_AXIS] ) +
                                 b_AccelFilt[PASTx3] * accelReading[PASTx3][X_AXIS] -
                                 a_AccelFilt[PASTx1] * accelFilt[PASTx1][X_AXIS] -
                                 a_AccelFilt[PASTx2] * accelFilt[PASTx2][X_AXIS] -
                                 a_AccelFilt[PASTx3] * accelFilt[PASTx3][X_AXIS];
    accelFilt[CURRENT][Y_AXIS] = b_AccelFilt[CURRENT] * (real)gEKFInputData.accel_B[Y_AXIS] +
                                 b_AccelFilt[PASTx1] * ( accelReading[PASTx1][Y_AXIS] +
                                                         accelReading[PASTx2][Y_AXIS] ) +
                                 b_AccelFilt[PASTx3] * accelReading[PASTx3][Y_AXIS] -
                                 a_AccelFilt[PASTx1] * accelFilt[PASTx1][Y_AXIS] -
                                 a_AccelFilt[PASTx2] * accelFilt[PASTx2][Y_AXIS] -
                                 a_AccelFilt[PASTx3] * accelFilt[PASTx3][Y_AXIS];
    accelFilt[CURRENT][Z_AXIS] = b_AccelFilt[CURRENT] * (real)gEKFInputData.accel_B[Z_AXIS] +
                                 b_AccelFilt[PASTx1] * ( accelReading[PASTx1][Z_AXIS] +
                                                         accelReading[PASTx2][Z_AXIS] ) +
                                 b_AccelFilt[PASTx3] * accelReading[PASTx3][Z_AXIS] -
                                 a_AccelFilt[PASTx1] * accelFilt[PASTx1][Z_AXIS] -
                                 a_AccelFilt[PASTx2] * accelFilt[PASTx2][Z_AXIS] -
                                 a_AccelFilt[PASTx3] * accelFilt[PASTx3][Z_AXIS];

    // Update past readings
    accelReading[PASTx3][X_AXIS] = accelReading[PASTx2][X_AXIS];
    accelReading[PASTx2][X_AXIS] = accelReading[PASTx1][X_AXIS];
    accelReading[PASTx1][X_AXIS] = (real)gEKFInputData.accel_B[X_AXIS];

    accelReading[PASTx3][Y_AXIS] = accelReading[PASTx2][Y_AXIS];
    accelReading[PASTx2][Y_AXIS] = accelReading[PASTx1][Y_AXIS];
    accelReading[PASTx1][Y_AXIS] = (real)gEKFInputData.accel_B[Y_AXIS];

    accelReading[PASTx3][Z_AXIS] = accelReading[PASTx2][Z_AXIS];
    accelReading[PASTx2][Z_AXIS] = accelReading[PASTx1][Z_AXIS];
    accelReading[PASTx1][Z_AXIS] = (real)gEKFInputData.accel_B[Z_AXIS];

    accelFilt[PASTx3][X_AXIS] = accelFilt[PASTx2][X_AXIS];
    accelFilt[PASTx2][X_AXIS] = accelFilt[PASTx1][X_AXIS];
    accelFilt[PASTx1][X_AXIS] = accelFilt[CURRENT][X_AXIS];

    accelFilt[PASTx3][Y_AXIS] = accelFilt[PASTx2][Y_AXIS];
    accelFilt[PASTx2][Y_AXIS] = accelFilt[PASTx1][Y_AXIS];
    accelFilt[PASTx1][Y_AXIS] = accelFilt[CURRENT][Y_AXIS];

    accelFilt[PASTx3][Z_AXIS] = accelFilt[PASTx2][Z_AXIS];
    accelFilt[PASTx2][Z_AXIS] = accelFilt[PASTx1][Z_AXIS];
    accelFilt[PASTx1][Z_AXIS] = accelFilt[CURRENT][Z_AXIS];

    // Calculate the acceleration magnitude for use with the linear-acceleration
    //   switch
    gAlgorithm.aMag = (real)sqrt( gEKFInputData.accel_B[X_AXIS] * gEKFInputData.accel_B[X_AXIS] +
                                  gEKFInputData.accel_B[Y_AXIS] * gEKFInputData.accel_B[Y_AXIS] +
                                  gEKFInputData.accel_B[Z_AXIS] * gEKFInputData.accel_B[Z_AXIS] );

    // Check for times when the acceleration is 'close' to 1 [g].  When this
    //   occurs, increment a counter.  When it exceeds a threshold (indicating
    //   that the system has been at rest for a given period) then decrease the
    //   R-values (in the update stage of the EKF), effectively increasing the
    //   Kalman gain.
    if (fabs( 1.0 - gAlgorithm.aMag ) < gAlgorithm.Limit.accelSwitch ) {
        gAlgorithm.linAccelSwitchCntr++;
        if ( gAlgorithm.linAccelSwitchCntr >= gAlgorithm.Limit.linAccelSwitchDelay ) {
            gAlgorithm.linAccelSwitch = TRUE;
        } else {
            gAlgorithm.linAccelSwitch = FALSE;
        }
    } else {
        gAlgorithm.linAccelSwitchCntr = 0;
        gAlgorithm.linAccelSwitch     = FALSE;
    }

    // Extract the magnetometer readings (set to zero if the magnetometer is not
    //   present or unused).
    if(platformHasMag())    
    {
        magFieldVector[X_AXIS] = (real)gEKFInputData.magField_B[X_AXIS];
        magFieldVector[Y_AXIS] = (real)gEKFInputData.magField_B[Y_AXIS];
        magFieldVector[Z_AXIS] = (real)gEKFInputData.magField_B[Z_AXIS];
    } else {
        magFieldVector[X_AXIS] = (real)0.0;
        magFieldVector[Y_AXIS] = (real)0.0;
        magFieldVector[Z_AXIS] = (real)0.0;
    }

    // Compute the measured Euler angles from gravity and magnetic field data
    //   ( phiMeas, thetaMeas, psiMeas ) = f( g_B, mMeas_B ).  Adjust for
    //   declination.
    FieldVectorsToEulerAngles( &accelFilt[CURRENT][0],
                               magFieldVector,
                               gAlgorithm.state == LOW_GAIN_AHRS,  // use pred to level when in LG
                               gKalmanFilter.measuredEulerAngles );

    // Adjust for declination if the GPS signal is good
    if( gAlgorithm.applyDeclFlag ) {
        gKalmanFilter.measuredEulerAngles[YAW] = gKalmanFilter.measuredEulerAngles[YAW] +
                                                 gWorldMagModel.decl_rad;
    }
}


// PredictStateEstimate.m
static void _PredictStateEstimate(void)
{
    real aCorr_N[3];
    real deltaQuaternion[4];

    static real GRAVITY_VECTOR_N_FRAME[3] = { (real)0.0, (real)0.0, (real)(-GRAVITY) };

    // Predict the EKF states at 100 Hz based on readings from the:
    //  - accelerometer
    //  - angular-rate sensors

    // ================= NED Position (r_N) =================
    // r_N(k+1) = r_N(k) + dV_N = r_N(k) + v_N*DT
    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] +
                                       gKalmanFilter.Velocity_N[X_AXIS] * gAlgorithm.dt;
    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] +
                                       gKalmanFilter.Velocity_N[Y_AXIS] * gAlgorithm.dt;
    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] +
                                       gKalmanFilter.Velocity_N[Z_AXIS] * gAlgorithm.dt;

    // ================= NED Velocity (v_N) =================
    // Generate the transformation matrix (R_BinN) based on the past value of
    //   the attitude quaternion (prior to prediction at the new time-step)
    QuaternionToR321(gKalmanFilter.quaternion, &gKalmanFilter.R_BinN[0][0]);

    // aCorr_B = aMeas_B - aBias_B
    // gEKFInputData.accel_B in g's, convert to m/s^2 for integration
    gKalmanFilter.correctedAccel_B[X_AXIS] = (real)(gEKFInputData.accel_B[X_AXIS] * GRAVITY) -
                                             gKalmanFilter.accelBias_B[X_AXIS];
    gKalmanFilter.correctedAccel_B[Y_AXIS] = (real)(gEKFInputData.accel_B[Y_AXIS] * GRAVITY) -
                                             gKalmanFilter.accelBias_B[Y_AXIS];
    gKalmanFilter.correctedAccel_B[Z_AXIS] = (real)(gEKFInputData.accel_B[Z_AXIS] * GRAVITY) -
                                             gKalmanFilter.accelBias_B[Z_AXIS];

    // Transform the corrected acceleration vector from the body to the NED-frame
    // a_N = R_BinN * a_B
    aCorr_N[X_AXIS] = gKalmanFilter.R_BinN[X_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                      gKalmanFilter.R_BinN[X_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                      gKalmanFilter.R_BinN[X_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
    aCorr_N[Y_AXIS] = gKalmanFilter.R_BinN[Y_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                      gKalmanFilter.R_BinN[Y_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                      gKalmanFilter.R_BinN[Y_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
    aCorr_N[Z_AXIS] = gKalmanFilter.R_BinN[Z_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                      gKalmanFilter.R_BinN[Z_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                      gKalmanFilter.R_BinN[Z_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];

    // Determine the acceleration of the system by removing the gravity vector
    // v_N(k+1) = v_N(k) + dV = v_N(k) + aMotion_N*DT = v_N(k) + ( a_N - g_N )*DT
    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] +
                                       ( aCorr_N[X_AXIS] - GRAVITY_VECTOR_N_FRAME[X_AXIS] ) * gAlgorithm.dt;
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] +
                                       ( aCorr_N[Y_AXIS] - GRAVITY_VECTOR_N_FRAME[Y_AXIS] ) * gAlgorithm.dt;
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] +
                                       ( aCorr_N[Z_AXIS] - GRAVITY_VECTOR_N_FRAME[Z_AXIS] ) * gAlgorithm.dt;

    // ================= Attitude quaternion =================
    // Find the 'true' angular rate (wTrue_B = wCorr_B = wMeas_B - wBias_B)
    gKalmanFilter.correctedRate_B[X_AXIS] = (real)gEKFInputData.angRate_B[X_AXIS] -
                                            gKalmanFilter.rateBias_B[X_AXIS];
    gKalmanFilter.correctedRate_B[Y_AXIS] = (real)gEKFInputData.angRate_B[Y_AXIS] -
                                            gKalmanFilter.rateBias_B[Y_AXIS];
    gKalmanFilter.correctedRate_B[Z_AXIS] = (real)gEKFInputData.angRate_B[Z_AXIS] -
                                            gKalmanFilter.rateBias_B[Z_AXIS];

    // Placed in gKalmanFilter as wTrueTimesDtOverTwo is used to compute the Jacobian (F)
    gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] = gKalmanFilter.correctedRate_B[X_AXIS] * gAlgorithm.dtOverTwo;
    gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] = gKalmanFilter.correctedRate_B[Y_AXIS] * gAlgorithm.dtOverTwo;
    gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] = gKalmanFilter.correctedRate_B[Z_AXIS] * gAlgorithm.dtOverTwo;

    // Save the past attitude quaternion before updating (for use in the
    //   covariance estimation calculations)
    gKalmanFilter.quaternion_Past[Q0] = gKalmanFilter.quaternion[Q0];
    gKalmanFilter.quaternion_Past[Q1] = gKalmanFilter.quaternion[Q1];
    gKalmanFilter.quaternion_Past[Q2] = gKalmanFilter.quaternion[Q2];
    gKalmanFilter.quaternion_Past[Q3] = gKalmanFilter.quaternion[Q3];

    // Find the attitude change based on angular rate data
    deltaQuaternion[Q0] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q1] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q2] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q1] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q0] +
                           gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q2] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q2] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q0] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q1] +
                           gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q3] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q0] +
                           gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q1] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q2];

    // Update the attitude
    // q_BinN(k+1) = q_BinN(k) + dq = q_BinN(k) + OMEGA*q_BinN(k)
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + deltaQuaternion[Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + deltaQuaternion[Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + deltaQuaternion[Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + deltaQuaternion[Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    // ================= Angular-rate bias: wBias(k+1) = wBias(k) =================
    // N/A (predicted state is same as past state)

    // ================= Linear-acceleration bias: aBias(k+1) = aBias(k) =================
    // N/A (predicted state is same as past state)
}


// Define variables that reside on the heap
real PxFTranspose[ROWS_IN_P][ROWS_IN_F], FxPxFTranspose[ROWS_IN_F][ROWS_IN_F];

// PredictCovarianceEstimate.m
static void _PredictCovarianceEstimate(void)
{
    uint8_t rowNum, colNum, multIndex;

    // Compute the F and Q matrices used in the prediction stage (only certain
    //   elements in the process-covariance, Q, change with each time-step)
    _UpdateProcessJacobian();     // gKF.F  (16x16)
    _UpdateProcessCovariance();   // gKF.Q  (16x16)

    // Update P from the P, F, and Q matrices: P = FxPxFTranspose + Q
    //   1) PxFTranspose is computed first
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        for (colNum = 0; colNum < ROWS_IN_F; colNum++) {
            PxFTranspose[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[colNum][0]; multIndex <= RLE_F[colNum][1]; multIndex++) {
                PxFTranspose[rowNum][colNum] = PxFTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.F[colNum][multIndex];
            }
        }
    }

    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < 16; rowNum++) {
        for (colNum = 0; colNum < 16; colNum++) {
            gKalmanFilter.P[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[rowNum][0]; multIndex <= RLE_F[rowNum][1]; multIndex++) {
                gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] +
                    gKalmanFilter.F[rowNum][multIndex] * PxFTranspose[multIndex][colNum];
            }
        }
    }

    //   3) Finally, add Q to FxPxFTranspose (P) to get the final value for
    //    gKalmanFilter.P (only the quaternion elements of Q have nonzero off-
    //    diagonal terms)
    gKalmanFilter.P[STATE_RX][STATE_RX] = gKalmanFilter.P[STATE_RX][STATE_RX] + gKalmanFilter.Q[STATE_RX][STATE_RX];
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RY][STATE_RY] + gKalmanFilter.Q[STATE_RY][STATE_RY];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + gKalmanFilter.Q[STATE_RZ][STATE_RZ];

    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] + gKalmanFilter.Q[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VY][STATE_VY] + gKalmanFilter.Q[STATE_VY][STATE_VY];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + gKalmanFilter.Q[STATE_VZ][STATE_VZ];

    gKalmanFilter.P[STATE_Q0][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q0] + gKalmanFilter.Q[STATE_Q0][STATE_Q0];
    gKalmanFilter.P[STATE_Q0][STATE_Q1] = gKalmanFilter.P[STATE_Q0][STATE_Q1] + gKalmanFilter.Q[STATE_Q0][STATE_Q1];
    gKalmanFilter.P[STATE_Q0][STATE_Q2] = gKalmanFilter.P[STATE_Q0][STATE_Q2] + gKalmanFilter.Q[STATE_Q0][STATE_Q2];
    gKalmanFilter.P[STATE_Q0][STATE_Q3] = gKalmanFilter.P[STATE_Q0][STATE_Q3] + gKalmanFilter.Q[STATE_Q0][STATE_Q3];

    gKalmanFilter.P[STATE_Q1][STATE_Q0] = gKalmanFilter.P[STATE_Q1][STATE_Q0] + gKalmanFilter.Q[STATE_Q1][STATE_Q0];
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q1] + gKalmanFilter.Q[STATE_Q1][STATE_Q1];
    gKalmanFilter.P[STATE_Q1][STATE_Q2] = gKalmanFilter.P[STATE_Q1][STATE_Q2] + gKalmanFilter.Q[STATE_Q1][STATE_Q2];
    gKalmanFilter.P[STATE_Q1][STATE_Q3] = gKalmanFilter.P[STATE_Q1][STATE_Q3] + gKalmanFilter.Q[STATE_Q1][STATE_Q3];

    gKalmanFilter.P[STATE_Q2][STATE_Q0] = gKalmanFilter.P[STATE_Q2][STATE_Q0] + gKalmanFilter.Q[STATE_Q2][STATE_Q0];
    gKalmanFilter.P[STATE_Q2][STATE_Q1] = gKalmanFilter.P[STATE_Q2][STATE_Q1] + gKalmanFilter.Q[STATE_Q2][STATE_Q1];
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q2] + gKalmanFilter.Q[STATE_Q2][STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q3] = gKalmanFilter.P[STATE_Q2][STATE_Q3] + gKalmanFilter.Q[STATE_Q2][STATE_Q3];

    gKalmanFilter.P[STATE_Q3][STATE_Q0] = gKalmanFilter.P[STATE_Q3][STATE_Q0] + gKalmanFilter.Q[STATE_Q3][STATE_Q0];
    gKalmanFilter.P[STATE_Q3][STATE_Q1] = gKalmanFilter.P[STATE_Q3][STATE_Q1] + gKalmanFilter.Q[STATE_Q3][STATE_Q1];
    gKalmanFilter.P[STATE_Q3][STATE_Q2] = gKalmanFilter.P[STATE_Q3][STATE_Q2] + gKalmanFilter.Q[STATE_Q3][STATE_Q2];
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = gKalmanFilter.P[STATE_Q3][STATE_Q3] + gKalmanFilter.Q[STATE_Q3][STATE_Q3];

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = gKalmanFilter.P[STATE_WBX][STATE_WBX] + gKalmanFilter.Q[STATE_WBX][STATE_WBX];
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = gKalmanFilter.P[STATE_WBY][STATE_WBY] + gKalmanFilter.Q[STATE_WBY][STATE_WBY];
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = gKalmanFilter.P[STATE_WBZ][STATE_WBZ] + gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];

    gKalmanFilter.P[STATE_ABX][STATE_ABX] = gKalmanFilter.P[STATE_ABX][STATE_ABX] + gKalmanFilter.Q[STATE_ABX][STATE_ABX];
    gKalmanFilter.P[STATE_ABY][STATE_ABY] = gKalmanFilter.P[STATE_ABY][STATE_ABY] + gKalmanFilter.Q[STATE_ABY][STATE_ABY];
    gKalmanFilter.P[STATE_ABZ][STATE_ABZ] = gKalmanFilter.P[STATE_ABZ][STATE_ABZ] + gKalmanFilter.Q[STATE_ABZ][STATE_ABZ];

    // P is a fully populated matrix (nominally) so all the elements of the matrix have to be
    //   considered when working with it.
    LimitValuesAndForceMatrixSymmetry_noAvg(&gKalmanFilter.P[0][0], (real)LIMIT_P, ROWS_IN_P, COLS_IN_P);
}


// GenerateProcessJacobian.m: Set the elements of F that DO NOT change with each time-step
void GenerateProcessJacobian(void)
{
    // Initialize the Process Jacobian matrix (F)
    memset(gKalmanFilter.F, 0, sizeof(gKalmanFilter.F));

    // Form the process Jacobian

    // ---------- Rows corresponding to POSITION ----------
    gKalmanFilter.F[STATE_RX][STATE_VX] = gAlgorithm.dt;
    gKalmanFilter.F[STATE_RY][STATE_VY] = gAlgorithm.dt;
    gKalmanFilter.F[STATE_RZ][STATE_VZ] = gAlgorithm.dt;

    // ---------- Rows corresponding to VELOCITY ----------
    // N/A (other than diagonal, set below, all other terms changes with attitude)

    // ---------- Rows corresponding to ATTITUDE ----------
    // N/A (other than diagonal, set below, all other terms changes with attitude)

    // ---------- Rows corresponding to RATE-BIAS ----------
    // N/A (no terms changes with attitude)

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------
    // N/A (no terms changes with attitude)

    // ---------- Add to I16 to get final formulation of F ----------
    // Populate the diagonals of F with 1.0
    gKalmanFilter.F[STATE_RX][STATE_RX] = (real)1.0;
    gKalmanFilter.F[STATE_RY][STATE_RY] = (real)1.0;
    gKalmanFilter.F[STATE_RZ][STATE_RZ] = (real)1.0;

    gKalmanFilter.F[STATE_VX][STATE_VX] = (real)1.0;
    gKalmanFilter.F[STATE_VY][STATE_VY] = (real)1.0;
    gKalmanFilter.F[STATE_VZ][STATE_VZ] = (real)1.0;

    gKalmanFilter.F[STATE_Q0][STATE_Q0] = (real)1.0;
    gKalmanFilter.F[STATE_Q1][STATE_Q1] = (real)1.0;
    gKalmanFilter.F[STATE_Q2][STATE_Q2] = (real)1.0;
    gKalmanFilter.F[STATE_Q3][STATE_Q3] = (real)1.0;

    gKalmanFilter.F[STATE_WBX][STATE_WBX] = (real)1.0;
    gKalmanFilter.F[STATE_WBY][STATE_WBY] = (real)1.0;
    gKalmanFilter.F[STATE_WBZ][STATE_WBZ] = (real)1.0;

    gKalmanFilter.F[STATE_ABX][STATE_ABX] = (real)1.0;
    gKalmanFilter.F[STATE_ABY][STATE_ABY] = (real)1.0;
    gKalmanFilter.F[STATE_ABZ][STATE_ABZ] = (real)1.0;
}


// _UpdateProcessJacobian.m: Update the elements of F that change with each time-step
static void _UpdateProcessJacobian(void)
{
    real q0aXdT, q1aXdT, q2aXdT, q3aXdT;
    real q0aYdT, q1aYdT, q2aYdT, q3aYdT;
    real q0aZdT, q1aZdT, q2aZdT, q3aZdT;
    real q0DtOver2, q1DtOver2, q2DtOver2, q3DtOver2;

    // ---------- Rows corresponding to POSITION ----------
    // No updates

    // ---------- Rows corresponding to VELOCITY ----------
    // Columns corresponding to the attitude-quaternion states
    q0aXdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q1aXdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q2aXdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q3aXdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;

    q0aYdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q1aYdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q2aYdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q3aYdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;

    q0aZdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q1aZdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q2aZdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q3aZdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;

    //
    gKalmanFilter.F[STATE_VX][STATE_Q0] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT );
    gKalmanFilter.F[STATE_VX][STATE_Q1] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT );
    gKalmanFilter.F[STATE_VX][STATE_Q2] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT );
    gKalmanFilter.F[STATE_VX][STATE_Q3] = (real)2.0 * ( -q3aXdT - q0aYdT + q1aZdT );

    gKalmanFilter.F[STATE_VY][STATE_Q0] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT );
    gKalmanFilter.F[STATE_VY][STATE_Q1] = (real)2.0 * (  q2aXdT - q1aYdT - q0aZdT );
    gKalmanFilter.F[STATE_VY][STATE_Q2] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT );
    gKalmanFilter.F[STATE_VY][STATE_Q3] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT );

    gKalmanFilter.F[STATE_VZ][STATE_Q0] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT );
    gKalmanFilter.F[STATE_VZ][STATE_Q1] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT );
    gKalmanFilter.F[STATE_VZ][STATE_Q2] = (real)2.0 * ( -q0aXdT + q3aYdT - q2aZdT );
    gKalmanFilter.F[STATE_VZ][STATE_Q3] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT );

    // Columns corresponding to the acceleration-bias state (-R_BinN*DT)
    gKalmanFilter.F[STATE_VX][STATE_ABX] = -gKalmanFilter.R_BinN[0][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VX][STATE_ABY] = -gKalmanFilter.R_BinN[0][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VX][STATE_ABZ] = -gKalmanFilter.R_BinN[0][2] * gAlgorithm.dt;

    gKalmanFilter.F[STATE_VY][STATE_ABX] = -gKalmanFilter.R_BinN[1][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VY][STATE_ABY] = -gKalmanFilter.R_BinN[1][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VY][STATE_ABZ] = -gKalmanFilter.R_BinN[1][2] * gAlgorithm.dt;

    gKalmanFilter.F[STATE_VZ][STATE_ABX] = -gKalmanFilter.R_BinN[2][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VZ][STATE_ABY] = -gKalmanFilter.R_BinN[2][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VZ][STATE_ABZ] = -gKalmanFilter.R_BinN[2][2] * gAlgorithm.dt;

    // ---------- Rows corresponding to attitude-QUATERNION ----------
    // Columns corresponding to the attitude-quaternion state (0.5*Omega*DT)
    //gKalmanFilter.F[STATE_Q0][STATE_Q0] =     0;
    gKalmanFilter.F[STATE_Q0][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    gKalmanFilter.F[STATE_Q0][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q0][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];

    gKalmanFilter.F[STATE_Q1][STATE_Q0] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q1][STATE_Q1] =     0;
    gKalmanFilter.F[STATE_Q1][STATE_Q2] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q1][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];

    gKalmanFilter.F[STATE_Q2][STATE_Q0] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q2][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    //gKalmanFilter.F[STATE_Q2][STATE_Q2] =     0;
    gKalmanFilter.F[STATE_Q2][STATE_Q3] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];

    gKalmanFilter.F[STATE_Q3][STATE_Q0] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q1] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q3,STATE.Q3] =     0;

    // Columns corresponding to the rate-bias state (-0.5*Xi*DT)
    q0DtOver2 = gKalmanFilter.quaternion_Past[Q0] * gAlgorithm.dtOverTwo;
    q1DtOver2 = gKalmanFilter.quaternion_Past[Q1] * gAlgorithm.dtOverTwo;
    q2DtOver2 = gKalmanFilter.quaternion_Past[Q2] * gAlgorithm.dtOverTwo;
    q3DtOver2 = gKalmanFilter.quaternion_Past[Q3] * gAlgorithm.dtOverTwo;

    // 
    gKalmanFilter.F[STATE_Q0][STATE_WBX] =  q1DtOver2;
    gKalmanFilter.F[STATE_Q0][STATE_WBY] =  q2DtOver2;
    gKalmanFilter.F[STATE_Q0][STATE_WBZ] =  q3DtOver2;

    gKalmanFilter.F[STATE_Q1][STATE_WBX] = -q0DtOver2;
    gKalmanFilter.F[STATE_Q1][STATE_WBY] =  q3DtOver2;
    gKalmanFilter.F[STATE_Q1][STATE_WBZ] = -q2DtOver2;

    gKalmanFilter.F[STATE_Q2][STATE_WBX] = -q3DtOver2;
    gKalmanFilter.F[STATE_Q2][STATE_WBY] = -q0DtOver2;
    gKalmanFilter.F[STATE_Q2][STATE_WBZ] =  q1DtOver2;

    gKalmanFilter.F[STATE_Q3][STATE_WBX] =  q2DtOver2;
    gKalmanFilter.F[STATE_Q3][STATE_WBY] = -q1DtOver2;
    gKalmanFilter.F[STATE_Q3][STATE_WBZ] = -q0DtOver2;

    // ---------- Rows corresponding to RATE-BIAS ----------
    // All zeros

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------
    // All zeros
}


// 
static void _UpdateProcessCovariance(void)
{
    // Variables used to initially populate the Q-matrix
    real arw, biSq[3] = {(real)1.0e-10, (real)1.0e-10, (real)1.0e-10};
    real sigDriftDot;

    // Variables used to populate the Q-matrix each time-step
    static real multiplier_Q, multiplier_Q_Sq;

    static BOOL initQ_HG = TRUE;
    static BOOL initQ_LG = TRUE;

    // Only need to generate Q-Bias values once
    if (initQ_HG == TRUE) {
        initQ_HG = FALSE;

        // This value is set based on the version string loaded into the unit
        //   via the system configuration load
        uint8_t sysRange = platformGetSysRange(); // from system config

        // Set the matrix, Q, based on whether the system is a -200 or -400
        //   Q-values are based on the rate-sensor's ARW (noise) and BI values
        //   passed through the process model
        switch (sysRange) {
            case _200_DPS_RANGE: // same as default
                // Bias-stability value for the rate-sensors:
                //   1.508e-3 [deg/sec] = 2.63e-5 [rad/sec]
                biSq[X_AXIS] = (real)(6.92e-10);
                    biSq[Y_AXIS] = biSq[X_AXIS];
                    biSq[Z_AXIS] = biSq[X_AXIS];
                break;

            // -400 values
            case _400_DPS_RANGE:
                // Bias-stability value for the rate-sensors:
                //   1.27e-3 [deg/sec] = 2.21e-5 [rad/sec]
                biSq[X_AXIS] = (real)(4.88e-10);
                    biSq[Y_AXIS] = biSq[X_AXIS];
                    biSq[Z_AXIS] = biSq[X_AXIS];
                break;
        }

        // ARW is unaffected by range setting.  Value in [rad/rt-sec]
        //   4.39e-3 [deg/rt-sec] = 7.66e-5 [rad/rt-sec]
            arw = (real)7.66e-5;

        // Rate-bias terms (computed once as it does not change with attitude). 
        //   sigDriftDot = (2*pi/ln(2)) * BI^2 / ARW
        //   2*pi/ln(2) = 9.064720283654388
        sigDriftDot = (real)9.064720283654388 / arw;

        // Rate-bias terms (Q is ultimately the squared value, which is done in the second line of the assignment)
        gKalmanFilter.Q[STATE_WBX][STATE_WBX] = sigDriftDot * biSq[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBX][STATE_WBX] = gKalmanFilter.Q[STATE_WBX][STATE_WBX] * gKalmanFilter.Q[STATE_WBX][STATE_WBX];

        gKalmanFilter.Q[STATE_WBY][STATE_WBY] = gKalmanFilter.Q[STATE_WBX][STATE_WBX];

        //gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = sigDriftDot * biSq[Z_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = sigDriftDot * biSq[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] * gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];

        gKalmanFilter.Q[STATE_ABX][STATE_ABX] = (real)6.839373353251920e-10;
        gKalmanFilter.Q[STATE_ABY][STATE_ABY] = (real)6.839373353251920e-10;
        gKalmanFilter.Q[STATE_ABZ][STATE_ABZ] = (real)6.839373353251920e-10;

        // Precalculate the multiplier applied to the Q terms associated with
        //   attitude.  sigRate = ARW / sqrt( dt )
    //
        // mult = 0.5 * dt * sigRate
        //      = 0.5 * sqrt(dt) * sqrt(dt) * ( ARW / sqrt(dt) )
        //      = 0.5 * sqrt(dt) * ARW
        multiplier_Q = (real)(0.5) * gAlgorithm.sqrtDt * arw;
        multiplier_Q_Sq = multiplier_Q * multiplier_Q;
    }

    // Attempt to solve the rate-bias problem: Decrease the process covariance,
    //   Q, upon transition to low-gain mode to limit the change in the rate-bias
    //   estimate.
    if( initQ_LG == TRUE ) {
        if( gAlgorithm.state == LOW_GAIN_AHRS ) {
            initQ_LG = FALSE;

            // After running drive-test data through the AHRS simulation,
            //   reducing Q by 1000x reduced the changeability of the rate-bias
            //   estimate (the estimate was less affected by errors).  This
            //   seems to have improved the solution as much of the errors
            //   was due to rapid changes in the rate-bias estimate.  This
            //   seemed to result in a better than nominal solution (for the
            //   drive-test).  Note: this is only called upon the first-entry
            //   into low-gain mode.
            gKalmanFilter.Q[STATE_WBX][STATE_WBX] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBX][STATE_WBX];
            gKalmanFilter.Q[STATE_WBY][STATE_WBY] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBY][STATE_WBY];
            gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];

            gKalmanFilter.Q[STATE_ABX][STATE_ABX] = gKalmanFilter.Q[STATE_ABX][STATE_ABX] * (real)1.0e-1;
            gKalmanFilter.Q[STATE_ABY][STATE_ABY] = gKalmanFilter.Q[STATE_ABY][STATE_ABY] * (real)1.0e-1;
            gKalmanFilter.Q[STATE_ABZ][STATE_ABZ] = gKalmanFilter.Q[STATE_ABZ][STATE_ABZ] * (real)1.0e-1;
        }
    }

    // Update the elements of the process covariance matrix, Q, that change
    //   with each time-step (the elements that correspond to the quaternion-
    //   block of the Q-matrix).  The rest of the elements in the matrix are set
    //   during the transition into and between EKF states (high-gain, low-gain,
    //   etc) or above (upon first entry into this function).
    real q0q0 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q0];
    real q0q1 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q1];
    real q0q2 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q2];
    real q0q3 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q3];

    real q1q1 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q1];
    real q1q2 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q2];
    real q1q3 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q3];

    real q2q2 = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.quaternion_Past[Q2];
    real q2q3 = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.quaternion_Past[Q3];

    real q3q3 = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.quaternion_Past[Q3];

    // Note: this block of the covariance matrix is symmetric
    gKalmanFilter.Q[STATE_Q0][STATE_Q0] = ((real)1.0 - q0q0) * multiplier_Q_Sq;
    gKalmanFilter.Q[STATE_Q0][STATE_Q1] = (-q0q1) * multiplier_Q_Sq;
    gKalmanFilter.Q[STATE_Q0][STATE_Q2] = (-q0q2) * multiplier_Q_Sq;
    gKalmanFilter.Q[STATE_Q0][STATE_Q3] = (-q0q3) * multiplier_Q_Sq;

    gKalmanFilter.Q[STATE_Q1][STATE_Q0] = gKalmanFilter.Q[STATE_Q0][STATE_Q1];
    gKalmanFilter.Q[STATE_Q1][STATE_Q1] = ((real)1.0 - q1q1) * multiplier_Q_Sq;
    gKalmanFilter.Q[STATE_Q1][STATE_Q2] = (-q1q2) * multiplier_Q_Sq;
    gKalmanFilter.Q[STATE_Q1][STATE_Q3] = (-q1q3) * multiplier_Q_Sq;

    gKalmanFilter.Q[STATE_Q2][STATE_Q0] = gKalmanFilter.Q[STATE_Q0][STATE_Q2];
    gKalmanFilter.Q[STATE_Q2][STATE_Q1] = gKalmanFilter.Q[STATE_Q1][STATE_Q2];
    gKalmanFilter.Q[STATE_Q2][STATE_Q2] = ((real)1.0 - q2q2) * multiplier_Q_Sq;
    gKalmanFilter.Q[STATE_Q2][STATE_Q3] = (-q2q3) * multiplier_Q_Sq;

    gKalmanFilter.Q[STATE_Q3][STATE_Q0] = gKalmanFilter.Q[STATE_Q0][STATE_Q3];
    gKalmanFilter.Q[STATE_Q3][STATE_Q1] = gKalmanFilter.Q[STATE_Q1][STATE_Q3];
    gKalmanFilter.Q[STATE_Q3][STATE_Q2] = gKalmanFilter.Q[STATE_Q2][STATE_Q3];
    gKalmanFilter.Q[STATE_Q3][STATE_Q3] = ((real)1.0 - q3q3) * multiplier_Q_Sq;
}


//
// GenerateProcessCovarMatrix.m
void GenerateProcessCovariance(void)
{
    // Initialize the Process Covariance (Q) matrix with values that do not change
    memset(gKalmanFilter.Q, 0, sizeof(gKalmanFilter.Q));

    // THE FOLLOWING COVARIANCE VALUES AREN'T CORRECT, JUST SELECTED SO THE
    // PROGRAM COULD RUN

    // Acceleration based values
    real dtSigAccelSq = (real)(gAlgorithm.dt * SENSOR_NOISE_ACCEL_STD_DEV);
    dtSigAccelSq = dtSigAccelSq * dtSigAccelSq;

    // Position
    gKalmanFilter.Q[STATE_RX][STATE_RX] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RY][STATE_RY] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RZ][STATE_RZ] = gAlgorithm.dtSquared * dtSigAccelSq;

    // Velocity
    gKalmanFilter.Q[STATE_VX][STATE_VX] = dtSigAccelSq;//(real)1e-10; 
    gKalmanFilter.Q[STATE_VY][STATE_VY] = dtSigAccelSq;
    gKalmanFilter.Q[STATE_VZ][STATE_VZ] = dtSigAccelSq;

    // Acceleration - bias
    gKalmanFilter.Q[STATE_ABX][STATE_ABX] = (real)1e-10; // dtSigAccelSq; //%1e-8 %sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABY][STATE_ABY] = (real)1e-10; //dtSigAccelSq; //%sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABZ][STATE_ABZ] = (real)1e-10; //dtSigAccelSq; //%sigmaAccelBiasSq;
}


/** ****************************************************************************
 * @name: firstOrderLowPass_float  implements a low pass yaw axis filter
 * @brief floating point version
 * TRACE:
 * @param [out] - output pointer to the filtered value
 * @param [in] - input pointer to a new raw value
 * @retval N/A
 * @details  This is a replacement for 'lowPass2Pole' found in the 440 code.
 *           Note, the implementation in the 440 SW is not a second-order filter
 *           but is an implementation of a first-order filter.
 ******************************************************************************/
void _FirstOrderLowPass( real *output,   // <-- INITIALLY THIS IS OUTPUT PAST (FILTERED VALUED IS RETURNED HERE)
                         real input )   // <-- CURRENT VALUE OF SIGNAL TO BE FILTERED
{
    static real inputPast;

    // 0.25 Hz LPF
    if( gAlgorithm.callingFreq == FREQ_100_HZ ) {
        *output = (real)(0.984414127416097) * (*output) + 
                  (real)(0.007792936291952) * (input + inputPast);
    } else {
        *output = (real)(0.992176700177507) * (*output) + 
                  (real)(0.003911649911247) * (input + inputPast);
    }

    inputPast = input;
}


// Set the accelerometer filter coefficients, which are used to filter the 
//   accelerometer readings prior to determining the setting of the linear-
//   acceleration switch and computing the roll and pitch from accelerometer
//   readings.
static void _PopulateFilterCoefficients(void)
{
    switch( gAlgorithm.linAccelLPFType ) {
        case NO_LPF:
            b_AccelFilt[0] = (real)(1.0);
            b_AccelFilt[1] = (real)(0.0);
            b_AccelFilt[2] = (real)(0.0);
            b_AccelFilt[3] = (real)(0.0);

            a_AccelFilt[0] = (real)(0.0);
            a_AccelFilt[1] = (real)(0.0);
            a_AccelFilt[2] = (real)(0.0);
            a_AccelFilt[3] = (real)(0.0);
            break;
        case TWO_HZ_LPF:
            if( gAlgorithm.callingFreq == FREQ_100_HZ ) {
                b_AccelFilt[0] = (real)(2.19606211225382e-4);
                b_AccelFilt[1] = (real)(6.58818633676145e-4);
                b_AccelFilt[2] = (real)(6.58818633676145e-4);
                b_AccelFilt[3] = (real)(2.19606211225382e-4);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.748835809214676);
                a_AccelFilt[2] = (real)( 2.528231219142559);
                a_AccelFilt[3] = (real)(-0.777638560238080);
            } else {
                b_AccelFilt[0] = (real)(2.91464944656705e-5);
                b_AccelFilt[1] = (real)(8.74394833970116e-5);
                b_AccelFilt[2] = (real)(8.74394833970116e-5);
                b_AccelFilt[3] = (real)(2.91464944656705e-5);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.874356892677485);
                a_AccelFilt[2] = (real)( 2.756483195225695);
                a_AccelFilt[3] = (real)(-0.881893130592486);
            }
            break;
        case FIVE_HZ_LPF:
            if( gAlgorithm.callingFreq == FREQ_100_HZ ) {
                b_AccelFilt[0] = (real)( 0.002898194633721);
                b_AccelFilt[1] = (real)( 0.008694583901164);
                b_AccelFilt[2] = (real)( 0.008694583901164);
                b_AccelFilt[3] = (real)( 0.002898194633721);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.374094743709352);
                a_AccelFilt[2] = (real)( 1.929355669091215);
                a_AccelFilt[3] = (real)(-0.532075368312092);
            } else {
                b_AccelFilt[0] = (real)( 0.000416546139076);
                b_AccelFilt[1] = (real)( 0.001249638417227);
                b_AccelFilt[2] = (real)( 0.001249638417227);
                b_AccelFilt[3] = (real)( 0.000416546139076);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.686157396548143);
                a_AccelFilt[2] = (real)( 2.419655110966473);
                a_AccelFilt[3] = (real)(-0.730165345305723);
            }
            break;
        case TWENTY_HZ_LPF:
            if (gAlgorithm.callingFreq == FREQ_100_HZ) {
                // [B,A] = butter(3,20/(100/2))
                b_AccelFilt[0] = (real)( 0.098531160923927);
                b_AccelFilt[1] = (real)( 0.295593482771781);
                b_AccelFilt[2] = (real)( 0.295593482771781);
                b_AccelFilt[3] = (real)( 0.098531160923927);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-0.577240524806303);
                a_AccelFilt[2] = (real)( 0.421787048689562);
                a_AccelFilt[3] = (real)(-0.056297236491843);
            } else {
                // [B,A] = butter(3,20/(200/2))
                b_AccelFilt[0] = (real)( 0.018098933007514);
                b_AccelFilt[1] = (real)( 0.054296799022543);
                b_AccelFilt[2] = (real)( 0.054296799022543);
                b_AccelFilt[3] = (real)( 0.018098933007514);
                
                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-1.760041880343169);
                a_AccelFilt[2] = (real)( 1.182893262037831);
                a_AccelFilt[3] = (real)(-0.278059917634546);
            }
            break;
        case TWENTY_FIVE_HZ_LPF:
            if (gAlgorithm.callingFreq == FREQ_100_HZ) {
                b_AccelFilt[0] = (real)( 0.166666666666667);
                b_AccelFilt[1] = (real)( 0.500000000000000);
                b_AccelFilt[2] = (real)( 0.500000000000000);
                b_AccelFilt[3] = (real)( 0.166666666666667);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-0.000000000000000);
                a_AccelFilt[2] = (real)( 0.333333333333333);
                a_AccelFilt[3] = (real)(-0.000000000000000);
            } else {
                b_AccelFilt[0] = (real)( 0.031689343849711);
                b_AccelFilt[1] = (real)( 0.095068031549133);
                b_AccelFilt[2] = (real)( 0.095068031549133);
                b_AccelFilt[3] = (real)( 0.031689343849711);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-1.459029062228061);
                a_AccelFilt[2] = (real)( 0.910369000290069);
                a_AccelFilt[3] = (real)(-0.197825187264319);
            }
            break;
        case TEN_HZ_LPF:
        default:
            if( gAlgorithm.callingFreq == FREQ_100_HZ ) {
                b_AccelFilt[0] = (real)( 0.0180989330075144);
                b_AccelFilt[1] = (real)( 0.0542967990225433);
                b_AccelFilt[2] = (real)( 0.0542967990225433);
                b_AccelFilt[3] = (real)( 0.0180989330075144);

                a_AccelFilt[0] = (real)( 1.0000000000000000);
                a_AccelFilt[1] = (real)(-1.7600418803431690);
                a_AccelFilt[2] = (real)( 1.1828932620378310);
                a_AccelFilt[3] = (real)(-0.2780599176345460);
            } else {
                b_AccelFilt[0] = (real)( 0.002898194633721);
                b_AccelFilt[1] = (real)( 0.008694583901164);
                b_AccelFilt[2] = (real)( 0.008694583901164);
                b_AccelFilt[3] = (real)( 0.002898194633721);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.374094743709352);
                a_AccelFilt[2] = (real)( 1.929355669091215);
                a_AccelFilt[3] = (real)(-0.532075368312092);
            }
            break;
    }
}

