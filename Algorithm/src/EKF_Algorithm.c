/*
 * File:   EKF_Algorithms.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include "GlobalConstants.h"   // TRUE, FALSE, etc

#include <math.h>      // std::abs
#include <stdlib.h>   // EXIT_FAILURE

#include "algorithm.h"        // gAlgorithm
#include "BITStatus.h"

#include "Indices.h"     // IND

#include "SelectState.h"
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "UpdateFunctions.h"
#include "AlgorithmLimits.h"
//#include "TimingVars.h"
#include "platformAPI.h"

KalmanFilterStruct gKalmanFilter;
EKF_InputDataStruct  gEKFInputData;
EKF_OutputDataStruct gEKFOutputData;

// This routine is called at either 100 or 200 Hz (based upon the system
//   configuration):
//      -- Unaided soln: 200 Hz
//      -- Aided soln: 100 Hz
void EKF_Algorithm(void)
{
    static uint16_t freeIntegrationCounter = 0;

    // Compute the EKF solution if past the stabilization and initialization
    //   stages
    if( gAlgorithm.state > INITIALIZE_ATTITUDE )
    {
        // Increment the algorithm itow
        gAlgorithm.itow = gAlgorithm.itow + gAlgorithm.dITOW;

        // Perform EKF Prediction
        EKF_PredictionStage();

        // Update the predicted states if not freely integrating (NOTE: free-
        //   integration is not applicable in HG AHRS mode)
        if( gAlgorithm.Behavior.bit.freeIntegrate &&
            ( gAlgorithm.state > HIGH_GAIN_AHRS ) )
        {
            // Limit the free-integration time before reverting to the complete
            //   EKF solution (including updates)
            freeIntegrationCounter = freeIntegrationCounter + 1;   // [cycles]
            if( freeIntegrationCounter >= gAlgorithm.Limit.Free_Integration_Cntr ) {
                freeIntegrationCounter = 0;
                enableFreeIntegration(FALSE);

#ifdef DISPLAY_DIAGNOSTIC_MSG
                // Display the time at the end of the free-integration period
                TimingVars_DiagnosticMsg("Free integration period ended");
#endif
            }

            // Restart the system in LG AHRS after free integration is complete
            gAlgorithm.insFirstTime = TRUE;
            gAlgorithm.state        = LOW_GAIN_AHRS;
            gAlgorithm.stateTimer   = gAlgorithm.Duration.Low_Gain_AHRS;
        } else {
            enableFreeIntegration(FALSE);
            freeIntegrationCounter = 0;

            // Perform EKF Update
            EKF_UpdateStage();
        }
    }

    // Select the algorithm state based upon the present state as well as
    //   operational conditions (time, sensor health, etc).  Note: This is called
    //   after the the above code-block to prevent the transition from occuring
    //   until the next time step.
    switch( gAlgorithm.state ) {
        case STABILIZE_SYSTEM:
            StabilizeSystem();
            break;
        case INITIALIZE_ATTITUDE:
            InitializeAttitude();
            break;
        case HIGH_GAIN_AHRS:
            HG_To_LG_Transition_Test();
            break;
        case LOW_GAIN_AHRS:
            // Prevent INS transition by setting
            //   gConfiguration.userBehavior.bit.useGPS false (use in simulation
            //   in lieu of AHRS_ONLY)
            LG_To_INS_Transition_Test();
            break;
        case INS_SOLUTION:
            INS_To_AHRS_Transition_Test();
            break;
        default:
#ifdef DISPLAY_DIAGNOSTIC_MSG
            // Shouldn't be able to make it here
            TimingVars_DiagnosticMsg("Uh-oh! Invalid algorithm state in EKF_Algorithm.cpp");
            std::cout << "Press enter to finish ...";
            std::cin.get();
#endif
            return;
    }

    // Dynamic motion logic (to revert back to HG AHRS)
    DynamicMotion();
}

void enableFreeIntegration(BOOL enable)
{
    gAlgorithm.Behavior.bit.freeIntegrate = enable;
}

void setAlgorithmExeFreq(int freq)
{
    gAlgorithm.callingFreq = freq;

}

BOOL freeIntegrationEnabled()
{
    return gAlgorithm.Behavior.bit.freeIntegrate;
}   

void enableMagInAlgorithm(BOOL enable)
{
    if(platformHasMag()){
        gAlgorithm.Behavior.bit.useMag = enable;
    }else{
        gAlgorithm.Behavior.bit.useMag = FALSE;
    }
}

BOOL magUsedInAlgorithm()
{
    return gAlgorithm.Behavior.bit.useMag != 0;
}

BOOL gpsUsedInAlgorithm(void)
{
    return gAlgorithm.Behavior.bit.useGPS;
}

void enableGpsUsageInAlgorithm(BOOL enable)
{
    gAlgorithm.Behavior.bit.useGPS = enable;
}



// Getters based on results structure passed to WriteResultsIntoOutputStream()

// Extract the attitude (expressed as Euler-angles) of the body-frame (B)
//   in the NED-frame (N) in [deg]
void EKF_GetAttitude_EA(real *EulerAngles)
{
    // Euler-angles in [deg]
    EulerAngles[ROLL]  = (real)gEKFOutputData.eulerAngs_BinN[ROLL];
    EulerAngles[PITCH] = (real)gEKFOutputData.eulerAngs_BinN[PITCH];
    EulerAngles[YAW]   = (real)gEKFOutputData.eulerAngs_BinN[YAW];
}


// Extract the attitude (expressed by quaternion-elements) of the body-
//   frame (B) in the NED-frame (N)
void EKF_GetAttitude_Q(real *Quaternions)
{
    Quaternions[Q0] = (real)gEKFOutputData.quaternion_BinN[Q0];
    Quaternions[Q1] = (real)gEKFOutputData.quaternion_BinN[Q1];
    Quaternions[Q2] = (real)gEKFOutputData.quaternion_BinN[Q2];
    Quaternions[Q3] = (real)gEKFOutputData.quaternion_BinN[Q3];
}


// Extract the angular-rate of the body (corrected for estimated rate-bias)
//   measured in the body-frame (B)
void EKF_GetCorrectedAngRates(real *CorrAngRates_B)
{
    // Angular-rate in [deg/s]
    CorrAngRates_B[X_AXIS] = (real)gEKFOutputData.corrAngRates_B[X_AXIS];
    CorrAngRates_B[Y_AXIS] = (real)gEKFOutputData.corrAngRates_B[Y_AXIS];
    CorrAngRates_B[Z_AXIS] = (real)gEKFOutputData.corrAngRates_B[Z_AXIS];
}


// Extract the acceleration of the body (corrected for estimated
//   accelerometer-bias) measured in the body-frame (B)
void EKF_GetCorrectedAccels(real *CorrAccels_B)
{
    // Acceleration in [m/s^2]
    CorrAccels_B[X_AXIS] = (real)gEKFOutputData.corrAccel_B[X_AXIS];
    CorrAccels_B[Y_AXIS] = (real)gEKFOutputData.corrAccel_B[Y_AXIS];
    CorrAccels_B[Z_AXIS] = (real)gEKFOutputData.corrAccel_B[Z_AXIS];
}


// Extract the acceleration of the body (corrected for estimated
//   accelerometer-bias) measured in the body-frame (B)
void EKF_GetEstimatedAngRateBias(real *AngRateBias_B)
{
    // Angular-rate bias in [deg/sec]
    AngRateBias_B[X_AXIS] = (real)gEKFOutputData.angRateBias_B[X_AXIS];
    AngRateBias_B[Y_AXIS] = (real)gEKFOutputData.angRateBias_B[Y_AXIS];
    AngRateBias_B[Z_AXIS] = (real)gEKFOutputData.angRateBias_B[Z_AXIS];
}


// Extract the acceleration of the body (corrected for estimated
//   accelerometer-bias) measured in the body-frame (B)
void EKF_GetEstimatedAccelBias(real *AccelBias_B)
{
    // Acceleration-bias in [m/s^2]
    AccelBias_B[X_AXIS] = (real)gEKFOutputData.accelBias_B[X_AXIS];
    AccelBias_B[Y_AXIS] = (real)gEKFOutputData.accelBias_B[Y_AXIS];
    AccelBias_B[Z_AXIS] = (real)gEKFOutputData.accelBias_B[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedPosition(real *Position_N)
{
    // Position in [m]
    Position_N[X_AXIS] = (real)gEKFOutputData.position_N[X_AXIS];
    Position_N[Y_AXIS] = (real)gEKFOutputData.position_N[Y_AXIS];
    Position_N[Z_AXIS] = (real)gEKFOutputData.position_N[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedVelocity(real *Velocity_N)
{
    // Velocity in [m/s]
    Velocity_N[X_AXIS] = (real)gEKFOutputData.velocity_N[X_AXIS];
    Velocity_N[Y_AXIS] = (real)gEKFOutputData.velocity_N[Y_AXIS];
    Velocity_N[Z_AXIS] = (real)gEKFOutputData.velocity_N[Z_AXIS];
}


// Extract the Operational Mode of the Algorithm:
//   0: Stabilize
//   1: Initialize
//   2: High-Gain VG/AHRS mode
//   3: Low-Gain VG/AHRS mode
//   4: INS operation
void EKF_GetOperationalMode(uint8_t *EKF_OperMode)
{
    *EKF_OperMode = gEKFOutputData.opMode;
}


// Extract the linear-acceleration and turn-switch flags
void EKF_GetOperationalSwitches(uint8_t *EKF_LinAccelSwitch, uint8_t *EKF_TurnSwitch)
{
    *EKF_LinAccelSwitch = gEKFOutputData.linAccelSwitch;
    *EKF_TurnSwitch     = gEKFOutputData.turnSwitchFlag;
}


// SETTERS: for EKF input and output structures

// Populate the EKF input structure with sensor and GPS data (if used)
void EKF_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps)
{
    // Accelerometer signal is in [g]
    gEKFInputData.accel_B[X_AXIS]    = accels[X_AXIS];
    gEKFInputData.accel_B[Y_AXIS]    = accels[Y_AXIS];
    gEKFInputData.accel_B[Z_AXIS]    = accels[Z_AXIS];

    // Angular-rate signal is in [rad/s]
    gEKFInputData.angRate_B[X_AXIS]  = rates[X_AXIS];
    gEKFInputData.angRate_B[Y_AXIS]  = rates[Y_AXIS];
    gEKFInputData.angRate_B[Z_AXIS]  = rates[Z_AXIS];

    // Magnetometer signal is in [G]
    gEKFInputData.magField_B[X_AXIS] = mags[X_AXIS];
    gEKFInputData.magField_B[Y_AXIS] = mags[Y_AXIS];
    gEKFInputData.magField_B[Z_AXIS] = mags[Z_AXIS];
}


// Populate the EKF output structure with algorithm results
void EKF_SetOutputStruct(void)
{
    // ------------------ States ------------------

    // Position in [m]
    gEKFOutputData.position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS];
    gEKFOutputData.position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS];
    gEKFOutputData.position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS];

    // Velocity in [m/s]
    gEKFOutputData.velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS];
    gEKFOutputData.velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS];
    gEKFOutputData.velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS];

    // Position in [N/A]
    gEKFOutputData.quaternion_BinN[Q0] = gKalmanFilter.quaternion[Q0];
    gEKFOutputData.quaternion_BinN[Q1] = gKalmanFilter.quaternion[Q1];
    gEKFOutputData.quaternion_BinN[Q2] = gKalmanFilter.quaternion[Q2];
    gEKFOutputData.quaternion_BinN[Q3] = gKalmanFilter.quaternion[Q3];

    // Angular-rate bias in [deg/sec]
    gEKFOutputData.angRateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] * RAD_TO_DEG;
    gEKFOutputData.angRateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] * RAD_TO_DEG;
    gEKFOutputData.angRateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] * RAD_TO_DEG;

    // Acceleration-bias in [m/s^2]
    gEKFOutputData.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] * ACCEL_DUE_TO_GRAV;
    gEKFOutputData.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] * ACCEL_DUE_TO_GRAV;
    gEKFOutputData.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] * ACCEL_DUE_TO_GRAV;

    // ------------------ Derived variables ------------------

    // Euler-angles in [deg]
    gEKFOutputData.eulerAngs_BinN[ROLL]  = gKalmanFilter.eulerAngles[ROLL] * RAD_TO_DEG;
    gEKFOutputData.eulerAngs_BinN[PITCH] = gKalmanFilter.eulerAngles[PITCH] * RAD_TO_DEG;
    gEKFOutputData.eulerAngs_BinN[YAW]   = gKalmanFilter.eulerAngles[YAW] * RAD_TO_DEG;

    // Angular-rate in [deg/s]
    gEKFOutputData.corrAngRates_B[X_AXIS] = ( gEKFInputData.angRate_B[X_AXIS] -
                                              gKalmanFilter.rateBias_B[X_AXIS] ) * RAD_TO_DEG;
    gEKFOutputData.corrAngRates_B[Y_AXIS] = ( gEKFInputData.angRate_B[Y_AXIS] -
                                              gKalmanFilter.rateBias_B[Y_AXIS] ) * RAD_TO_DEG;
    gEKFOutputData.corrAngRates_B[Z_AXIS] = ( gEKFInputData.angRate_B[Z_AXIS] -
                                              gKalmanFilter.rateBias_B[Z_AXIS] ) * RAD_TO_DEG;

    // Acceleration in [m/s^2]
    gEKFOutputData.corrAccel_B[X_AXIS] = ( gEKFInputData.accel_B[X_AXIS] -
                                           gKalmanFilter.accelBias_B[X_AXIS] ) * ACCEL_DUE_TO_GRAV;
    gEKFOutputData.corrAccel_B[Y_AXIS] = ( gEKFInputData.accel_B[Y_AXIS] -
                                           gKalmanFilter.accelBias_B[Y_AXIS] ) * ACCEL_DUE_TO_GRAV;
    gEKFOutputData.corrAccel_B[Z_AXIS] = ( gEKFInputData.accel_B[Z_AXIS] -
                                           gKalmanFilter.accelBias_B[Z_AXIS] ) * ACCEL_DUE_TO_GRAV;

    // ------------------ Algorithm flags ------------------
    gEKFOutputData.opMode         = gAlgorithm.state;
    gEKFOutputData.linAccelSwitch = gAlgorithm.linAccelSwitch;
    gEKFOutputData.turnSwitchFlag = gBitStatus.swStatus.bit.turnSwitch;
}

