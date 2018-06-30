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
//#include "xbowsp_configuration.h"    // gConfiguration

#include "Indices.h"     // IND

#include "SelectState.h"
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "UpdateFunctions.h"
#include "AlgorithmLimits.h"
#include "TimingVars.h"
#include "platformAPI.h"

KalmanFilterStruct gKalmanFilter;
EKFInputDataStruct  gEKFInputData;
EKFOutputDataStruct gEKFOutputData;

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

//void GetSamplingTime( void )
//{
//
//}

// Extract the attitude (expressed in Euler-angles) of the body-frame (B)
//   in the NED-frame (N)
void GetEKF_Attitude_EA(real *EulerAngles)
{
    EulerAngles[ROLL]  = gEKFOutputData.eulerAngs_BinN[ROLL];
    EulerAngles[PITCH] = gEKFOutputData.eulerAngs_BinN[PITCH];
    EulerAngles[YAW]   = gEKFOutputData.eulerAngs_BinN[YAW];
}


// Extract the attitude (expressed by quaternion-elements) of the body-
//   frame (B) in the NED-frame (N)
void GetEKF_Attitude_Q(real *Quaternions)
{
    Quaternions[Q0] = gEKFOutputData.quaternion_BinN[Q0];
    Quaternions[Q1] = gEKFOutputData.quaternion_BinN[Q1];
    Quaternions[Q2] = gEKFOutputData.quaternion_BinN[Q2];
    Quaternions[Q3] = gEKFOutputData.quaternion_BinN[Q3];
}


// Extract the angular-rate of the body (corrected for estimated rate-bias)
//   measured in the body-frame (B)
void GetEKF_CorrectedAngRates(real *CorrAngRates_B)
{
    CorrAngRates_B[X_AXIS] = gEKFOutputData.corrAngRates_B[X_AXIS];
    CorrAngRates_B[Y_AXIS] = gEKFOutputData.corrAngRates_B[Y_AXIS];
    CorrAngRates_B[Z_AXIS] = gEKFOutputData.corrAngRates_B[Z_AXIS];
}


// Extract the acceleration of the body (corrected for estimated
//   accelerometer-bias) measured in the body-frame (B)
void GetEKF_CorrectedAccels(real *CorrAccels_B)
{
    CorrAccels_B[X_AXIS] = gEKFOutputData.corrAccel_B[X_AXIS];
    CorrAccels_B[Y_AXIS] = gEKFOutputData.corrAccel_B[Y_AXIS];
    CorrAccels_B[Z_AXIS] = gEKFOutputData.corrAccel_B[Z_AXIS];
}


// Extract the acceleration of the body (corrected for estimated
//   accelerometer-bias) measured in the body-frame (B)
void GetEKF_EstimatedAngRateBias(real *AngRateBias_B)
{
    AngRateBias_B[X_AXIS] = gEKFOutputData.angRateBias_B[X_AXIS];
    AngRateBias_B[Y_AXIS] = gEKFOutputData.angRateBias_B[Y_AXIS];
    AngRateBias_B[Z_AXIS] = gEKFOutputData.angRateBias_B[Z_AXIS];
}


// Extract the acceleration of the body (corrected for estimated
//   accelerometer-bias) measured in the body-frame (B)
void GetEKF_EstimatedAccelBias(real *AccelBias_B)
{
    AccelBias_B[X_AXIS] = gEKFOutputData.accelBias_B[X_AXIS];
    AccelBias_B[Y_AXIS] = gEKFOutputData.accelBias_B[Y_AXIS];
    AccelBias_B[Z_AXIS] = gEKFOutputData.accelBias_B[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void GetEKF_EstimatedPosition(real *Position_N)
{
    Position_N[X_AXIS] = gEKFOutputData.position_N[X_AXIS];
    Position_N[Y_AXIS] = gEKFOutputData.position_N[Y_AXIS];
    Position_N[Z_AXIS] = gEKFOutputData.position_N[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void GetEKF_EstimatedVelocity(real *Velocity_N)
{
    Velocity_N[X_AXIS] = gEKFOutputData.velocity_N[X_AXIS];
    Velocity_N[Y_AXIS] = gEKFOutputData.velocity_N[Y_AXIS];
    Velocity_N[Z_AXIS] = gEKFOutputData.velocity_N[Z_AXIS];
}


// Extract the Operational Mode of the Algorithm:
//   0: Stabilize
//   1: Initialize
//   2: High-Gain mode
//   3: Low-Gain mode
//   4: INS
void GetEKF_OperationalMode(uint8_t *EKF_OperMode)
{
    *EKF_OperMode = gEKFOutputData.opMode;
}


// Extract the linear-acceleration and turn-switch flags
void GetEKF_OperationalSwitches(uint8_t *EKF_LinAccelSwitch, uint8_t *EKF_TurnSwitch)
{
    *EKF_LinAccelSwitch = gEKFOutputData.linAccelSwitch;
    *EKF_TurnSwitch     = gEKFOutputData.turnSwitchFlag;
}



