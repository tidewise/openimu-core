/*
 * File:   UpdateFunctions.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include <math.h>
#include <string.h>

#include "UpdateMatrixSizing.h"  // used to specify the size of the update vectors
#include "GlobalConstants.h"   // TRUE, FALSE, NUMBER_OF_EKF_STATES, ...
#include "UpdateFunctions.h"
#include "algorithm.h"
#include "TimingVars.h"
#include "Indices.h"
#include "AlgorithmLimits.h"
#include "EKF_Algorithm.h"
#include "VectorMath.h"
#include "MatrixMath.h"
#include "QuaternionMath.h"
#include "TransformationMath.h"
#include "StateIndices.h"
#include "SensorNoiseParameters.h"
#include "GpsData.h"
#include "WorldMagneticModel.h"
#include "platformAPI.h"
#include "BITStatus.h"

#include "driverGPS.h"

//#include "RunLengthEncoding.h"
// H is sparse and has elements in the following locations...
uint8_t RLE_H[ROWS_IN_H][2] = { {STATE_Q0, STATE_Q3},
                                {STATE_Q0, STATE_Q3},
                                {STATE_Q0, STATE_Q3} };

// KxH is sparse with elements only in cols 6 through 9
uint8_t RLE_KxH[ROWS_IN_K][2] = { {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3},
                                  {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3}, {STATE_Q0, STATE_Q3} };

// Local functions
static void _TurnSwitch(void);

static real _UnwrapAttitudeError( real attitudeError );
static real _LimitValue( real value, real limit );
static BOOL _CheckForUpdateTrigger(uint8_t updateRate);


// Update rates
#define  TEN_HERTZ_UPDATE          10
#define  TWENTY_HERTZ_UPDATE       20
#define  TWENTY_FIVE_HERTZ_UPDATE  25
#define  FIFTY_HERTZ_UPDATE        50
#define  ONE_HUNDRED_HERTZ_UPDATE  100

static uint32_t updateCntr[2] = { 0, 0 };
static BOOL newGPSDataFlag;
static BOOL useMagHeading = 1;

// Uncomment to run only AHRS-type updates
//#define ATT_UPDATE_ONLY

// EKF_UpdateStage.m
void EKF_UpdateStage(void)
{
    // Perform a VG/AHRS update, regardless of GPS availability or health,
    //   when the state is HG AHRS or LG AHRS.  Once GPS becomes healthy
    //   (and the right conditions are met) perform an INS or reduced-
    //   order GPS update.
    if( gAlgorithm.state <= LOW_GAIN_AHRS )
    {
#if 0
        // Switch the update rate based on the status of the linear-
        //   acceleration switch (solution will converge faster when
        //   static)
        if( gAlgorithm.linAccelSwitch == TRUE ) {
            updateRate = TWENTY_FIVE_HERTZ_UPDATE;
        } else {
            updateRate = TEN_HERTZ_UPDATE;
        }
#endif

        // Only allow the algorithm to be called on 100 Hz marks
        if(timer.oneHundredHertzFlag == 1) {
        // Update the AHRS solution at a 10 Hz update rate
            // Subframe counter counts to 10 before it is reset
            if( _CheckForUpdateTrigger(TEN_HERTZ_UPDATE) )
            {
                // The AHRS/VG solution is handled inside FieldVectorsToEulerAngles
                //   (called from the prediction function EKF_PredictionStage)

                // nu = z - h(xPred)
                ComputeSystemInnovation_Att();
                Update_Att();
            }
        }
    } else {
        // GPS-type Updates (with magnetometers: true-heading = mag-heading + mag-decl)
        if ( magUsedInAlgorithm() )
        {
            // The following variable enables the update to be broken up into
            //   two sequential calculations in two sucessive 100 Hz periods
            static int runInsUpdate = 0;

            // Perform the EKF update at 10 Hz (split nine mag-only updates for
            //   for every GPS/mag update)
            // 
            // Check for 'new GPS data'.  If new, and GPS is valid, perform a
            //   GPS-Based update and reset timer values to resync the attitude
            //   updates.
            if( gEKFInputData.updateFlag && gEKFInputData.gpsValid )
            {
                // Resync timer
                timer.tenHertzCntr = 0;
                timer.subFrameCntr = 0;

// Debugging counter: updateCntr[0] should be about 9x less than updateCntr[1]
updateCntr[0] = updateCntr[0] + 1;
                // At 1 Hz mark, update when GPS data is valid, else do an AHRS-update
                runInsUpdate = 1;

                // Reset 'new GPS data' flag
                gEKFInputData.updateFlag = false;

                // Sync the algorithm itow to the GPS value (may place this elsewhere)
                gAlgorithm.itow = gEKFInputData.itow;

                // reset the "last good reading" time
                gAlgorithm.timeOfLastGoodGPSReading = gEKFInputData.itow;

                // Save off the Lat/Lon in radians and altitude in meters
                gAlgorithm.llaRad[X_AXIS] = (gEKFInputData.lat) * DEG_TO_RAD; // high precision from GPS
                gAlgorithm.llaRad[Y_AXIS] = (gEKFInputData.lon) * DEG_TO_RAD;
                gAlgorithm.llaRad[Z_AXIS] = gEKFInputData.alt;

                // Extract what's common between the following function and the
                //   routines below so we aren't repeating calculations
                GPS_PosVel_To_NED();

                // Use gGpsDataPtr->rawGroundSpeed instead to save math
                real vSq = (real)(gEKFInputData.vNed[X_AXIS] * gEKFInputData.vNed[X_AXIS] +
                                  gEKFInputData.vNed[Y_AXIS] * gEKFInputData.vNed[Y_AXIS]);

                // Set in LG_To_INS_Transition_Test for AHRS operation
                if (vSq >= LIMIT_GPS_VELOCITY_SQ) {
                    gAlgorithm.timeOfLastSufficientGPSVelocity = (int32_t)gEKFInputData.itow;
                }

                // When the system is at rest, use the magnetometer readings for
                //   heading updates.  DEBUG: 0.1 is arbitrary.
                useMagHeading =
                    gAlgorithm.Behavior.bit.useCourseHeading ?
                    ( gEKFInputData.rawGroundSpeed <= 0.45 ) : 1;  // 0.45 m/s ~= 1.0 mph
                gBitStatus.swStatus.bit.useCourseHeading = !useMagHeading;

                // This Sequential-Filter (three-stage approach) is nearly as
                //   good as the full implementation -- we also can split it
                //   across multiple iterations to not exceed 10 ms execution on
                //   the embedded 380
#ifndef ATT_UPDATE_ONLY
                // Compute the system error: z = meas, h = pred = q, nu - z - h
                //   Do this at the same time even if the update is spread
                //   across time-steps
                ComputeSystemInnovation_Pos();
                ComputeSystemInnovation_Vel();
                ComputeSystemInnovation_Att();

                // Calculate the R-values for the INS measurements
                _GenerateObservationCovariance_INS();

                Update_Pos();
                Update_Vel();
            // Comment out next line to run all update functions in one loop
            //   (doesn't work on M3 processor because of how the code is
            //   implemented - FAST_MATH not set up)
            } else if( runInsUpdate ) {
                Update_Att();
                runInsUpdate = 0;  // set up for next pass
#else
                ComputeSystemInnovation_Att();
                Update_Att();
#endif
            } else if ((timer.subFrameCntr == 0) && (timer.oneHundredHertzFlag == 1)) { // AHRS at all the other 10 Hz marks
                // AHRS update
// Debugging counter: updateCntr[0] should be about 9x less than updateCntr[1]
updateCntr[1] = updateCntr[1] + 1;
                ComputeSystemInnovation_Att();
                Update_Att();
            }

            // FIXME: MOVE THIS TO THE EKF CALL, AFTER THE UPDATE, SO THE LAT/LON
            //        IS COMPUTED FROM INTEGRATED DATA IF AN UPDATE IS NOT PERFORMED
            // Update LLA at 100/200 Hz
            if ( gEKFInputData.gpsValid && ( gAlgorithm.insFirstTime == FALSE ) ) {
                //r_E = Base_To_ECEF( &gKalmanFilter.Position_N[0], &gAlgorithm.rGPS0_E[0], &R_NinE[0][0] );    //
                double r_E[NUM_AXIS];
                PosNED_To_PosECEF( &gKalmanFilter.Position_N[0], &gAlgorithm.rGPS0_E[0], &gAlgorithm.R_NinE[0][0], &r_E[0] );
                //                 100 Hz                        generated once          1 Hz                      100 Hz

                //gKalmanFilter.llaDeg[LAT_IDX] = ECEF2LLA( r_E );   // output variable (ned used for anything else); this is in [ deg, deg, m ]
                ECEF_To_LLA(&gKalmanFilter.llaDeg[LAT], &r_E[X_AXIS]);
                //          100 Hz                    100 Hz
            }

    // Don't have this logic in the current code base
    //
    // //currently in navmode and need to make update
    // if ( (algorithm.timerITOW -lastEnoughGPSVelTime)>LONGER_STOP_FREE_HEADING_TIME &&
    //      !(calibration.productConfiguration.bit.mags && configuration.userBehavior.bit.useMags) &&
    //      velPacket &&
    //      speedSquared>YAW_TRACK_SPEED_SQ)
    //          initNavFilter(); // if stop period is a bit long and not mag is used,
    //                           //alogrithm heading needs to be re-initialized with GPS
    //                           // when getting engough GPS velocity

        } else {
            // Magnetometers are unavailable or unused

        }
//        % Magnetometers not available or unused
//
//        % Check for valid GPS signal
//        if( gEKFInputData.gpsValid ),
//            % Compute vSq and respond accordingly
//            vSq = gGpsDataPtr.vel(X_AXIS) * gGpsDataPtr.vel(X_AXIS) + ...
//                  gGpsDataPtr.vel(Y_AXIS) * gGpsDataPtr.vel(Y_AXIS);
//
//            % Update the GPS-Stuff at 1Hz
//            if( ( timing.subFrameCntr == 0 ) && ( timing.tenHertzCntr == 0 ) ),
//                GPSPosVelToNED;
//            end
//
//            if( vSq >= LIMIT.GPS_VELOCITY_SQ ),
//                % Sufficient GPS velocity, can use velocity to compute
//                %   heading (heading is also part of message)
//                timeOfLastSufficientGPSVelocity = itow;
//                yawLock_1stTimeFlag = TRUE;
//
//                % Update at the rate of GPS data availability (presently
//                %   1Hz)
//                if( ( timing.subFrameCntr == 0 ) && ( timing.tenHertzCntr == 0 ) ),
//                    % 1 Hz update -- Virtually no difference between the
//                    %   first two solutions
//                    if( 1 ),
//                        % Use GPS position, velocity, and yaw from track
//                        ComputeSystemInnovation_PosVelYaw;
//                        Update_PosVelYaw;
//                    elseif( 0 ),
//                        % Use GPS position, velocity, yaw from track, and
//                        %   accelerometer readings
//                        ComputeSystemInnovation_PosVelTiltYaw;
//                        Update_PosVelTiltYaw; % Need to use yaw from track!
//                    elseif( 0 ),
//                        % Doesn't seem to work
//                        ComputeSystemInnovation_Yaw;
//                        Update_Yaw; % Use yaw from GPS track
//                    elseif( 0 ),
//                        % Not promising either
//                        ComputeSystemInnovation_PosVel;
//                        Update_PosVel; % Need to use yaw from track!
//                    else
//                        ComputeSystemInnovation_Vel;
//                        nu = zeros(3,1);
//                        Update_Vel; % Need to use yaw from track!
//
//%                       ComputeSystemInnovation_Yaw;
//%                       Update_Yaw; % Use yaw from GPS track
//                    end
//                end
//            else
//                % GPS velocity is below threshold to perform update
//                yawRateSq = ( gKF.correctedRate_B(YAW) )^2;
//
//                if( gConfiguration.userBehavior.bit.stationaryYawLock && ...
//                    yawRateSq < LIMIT.YAW_RATE_SQ ),
//                    if( yawLock_1stTimeFlag ),
//                        yawLock_1stTimeFlag = FALSE;
//                        psi_yawLock = gKF.predictedEulerAngles(YAW);
//                    else
//                        % Perform a 10 Hz update split between Pos/Vel and
//                        %   Yaw
//                        if( timing.subFrameCntr == 0 ),
//                            if( ( timing.tenHertzCntr == 0 ) && ( gGpsDataPtr.GPSValid ) ),   % 1 Hz mark with valid GPS
//                                % 1 Hz update
//                                ComputeSystemInnovation_PosVel;
//                                Update_PosVel;
//                            else
//                                % 10 Hz update
//                                ComputeSystemInnovation_Yaw;
//                                Update_Yaw;  % need to use psi_yawLock
//                            end
//                        end
//                    end
//                else
//                    % Yaw-lock not selected OR Yaw-Rate sufficiently high
//                    %   (not stopped).  Perform a 1 Hz update (when GPS
//                    %   updated).
//                    if( ( timing.subFrameCntr == 0 ) && ( timing.tenHertzCntr == 0 ) ),
//                        ComputeSystemInnovation_PosVel;
//                        Update_PosVel;
//                    end
//
//                    yawLock_1stTimeFlag = 1;
//                end
//            end
//
//            % Update at what rate (100 Hz?)
//            tmpLLA = [ gGpsDataPtr.pos(X_AXIS) * DEG_TO_RAD;
//                       gGpsDataPtr.pos(Y_AXIS) * DEG_TO_RAD;
//                       gGpsDataPtr.pos(Z_AXIS) ];
//            RNinE = InvRneFromLLA( tmpLLA );   % matches the inverse calculation
//            r_E = Base2ECEF( gKF.Position_N, rGPS0_E, RNinE );    %
//            lla = ECEF2LLA( r_E );   % output variable (ned used for anything else)
//        else
//            % GPS is invalid (if GPS is invalid for longer than a database
//            %   limit (~3 seconds) then drop to AHRS, see
//            %   INS_To_AHRS_Transition_Test.m)
//            if( gConfiguration.userBehavior.bit.stationaryYawLock ),
//                % if stationaryYawLock === TRUE then 'hold' heading;
//                if( yawLock_1stTimeFlag ),
//                    % rationale: it doesn't make sense to do an update the
//                    %            first time as the error will be zero
//                    yawLock_1stTimeFlag = 0;
//                    psi_yawLock = gKF.predictedEulerAngles(YAW);
//                else
//                    % Update yaw at ten hertz (GPS readings unavailable)
//                    if( timing.subFrameCntr == 0 ),
//                        %ComputeSystemInnovation_Yaw;  % <-- uses gps velocity, wrong!
//                        % Compute the innovation (based on angle prior to yaw-lock)
//                        z = psi_yawLock;
//                        h = gKF.predictedEulerAngles(YAW);
//                        nu = z - h;
//
//                        Update_Yaw;
//                    end
//                end
//            else
//                % Yaw-Lock not enabled, let yaw drift until GPS recovers or
//                %   transition to AHRS occurs
//                yawLock_1stTimeFlag = 1;
//            end
//        end
//    end
    }
}


//
void ComputeSystemInnovation_Pos(void)
{
    // ----Compute the innovation vector, nu----
    // Position error
    gKalmanFilter.nu[STATE_RX] = gAlgorithm.rGPS_N[X_AXIS] - gKalmanFilter.Position_N[X_AXIS];
    gKalmanFilter.nu[STATE_RY] = gAlgorithm.rGPS_N[Y_AXIS] - gKalmanFilter.Position_N[Y_AXIS];
    gKalmanFilter.nu[STATE_RZ] = gAlgorithm.rGPS_N[Z_AXIS] - gKalmanFilter.Position_N[Z_AXIS];

    gKalmanFilter.nu[STATE_RX] = _LimitValue(gKalmanFilter.nu[STATE_RX], gAlgorithm.Limit.Innov.positionError);
    gKalmanFilter.nu[STATE_RY] = _LimitValue(gKalmanFilter.nu[STATE_RY], gAlgorithm.Limit.Innov.positionError);
    gKalmanFilter.nu[STATE_RZ] = _LimitValue(gKalmanFilter.nu[STATE_RZ], gAlgorithm.Limit.Innov.positionError);
}


//
void ComputeSystemInnovation_Vel(void)
{
    // ----Compute the innovation vector, nu----
    // Velocity error
    gKalmanFilter.nu[STATE_VX] = (real)gEKFInputData.vNed[X_AXIS] - gKalmanFilter.Velocity_N[X_AXIS];
    gKalmanFilter.nu[STATE_VY] = (real)gEKFInputData.vNed[Y_AXIS] - gKalmanFilter.Velocity_N[Y_AXIS];
    gKalmanFilter.nu[STATE_VZ] = (real)gEKFInputData.vNed[Z_AXIS] - gKalmanFilter.Velocity_N[Z_AXIS];

    gKalmanFilter.nu[STATE_VX] = _LimitValue(gKalmanFilter.nu[STATE_VX], gAlgorithm.Limit.Innov.velocityError);
    gKalmanFilter.nu[STATE_VY] = _LimitValue(gKalmanFilter.nu[STATE_VY], gAlgorithm.Limit.Innov.velocityError);
    gKalmanFilter.nu[STATE_VZ] = _LimitValue(gKalmanFilter.nu[STATE_VZ], gAlgorithm.Limit.Innov.velocityError);
}


// 
void ComputeSystemInnovation_Att(void)
{
    // Compute the innovation, nu, between measured and predicted attitude.
    //   Correct for wrap-around. Then limit the error.
    // ----- Roll -----
    gKalmanFilter.nu[STATE_ROLL]  = gKalmanFilter.measuredEulerAngles[ROLL] -
                                    gKalmanFilter.eulerAngles[ROLL];
    gKalmanFilter.nu[STATE_ROLL]  = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_ROLL]);
    gKalmanFilter.nu[STATE_ROLL] = _LimitValue(gKalmanFilter.nu[STATE_ROLL], gAlgorithm.Limit.Innov.attitudeError);

    // ----- Pitch -----
    gKalmanFilter.nu[STATE_PITCH] = gKalmanFilter.measuredEulerAngles[PITCH] -
                                    gKalmanFilter.eulerAngles[PITCH];
    gKalmanFilter.nu[STATE_PITCH] = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_PITCH]);
    gKalmanFilter.nu[STATE_PITCH] = _LimitValue(gKalmanFilter.nu[STATE_PITCH], gAlgorithm.Limit.Innov.attitudeError);

    // ----- Yaw -----
    // CHANGED TO SWITCH BETWEEN GPS AND MAG UPDATES
    if( useMagHeading ) {
        // ORIG
        // When using VG or GPS but the velocity is too low, use the
        //   magnetometer-derived heading.
        gKalmanFilter.nu[STATE_YAW]   = gKalmanFilter.measuredEulerAngles[YAW] -
                                        gKalmanFilter.eulerAngles[YAW];
    } else {
        // When updating using GPS readings, newGPSDataFlag will set the error
        //   to zero when performing the VG-type updates as it is reset after
        //   each pass through the gps logic (above).
        if( newGPSDataFlag ) {
            gKalmanFilter.nu[STATE_YAW] = (real)gEKFInputData.trueCourse * (real)DEG_TO_RAD -
                                          gKalmanFilter.eulerAngles[YAW];
        } else {
            // ORIG
        gKalmanFilter.nu[STATE_YAW] = (real)0.0;
    }
    }
    gKalmanFilter.nu[STATE_YAW] = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_YAW]);
    gKalmanFilter.nu[STATE_YAW] = _LimitValue(gKalmanFilter.nu[STATE_YAW], gAlgorithm.Limit.Innov.attitudeError);

    // When the filtered yaw-rate is above certain thresholds then reduce the
    //   attitude-errors used to update roll and pitch.
    _TurnSwitch();

    //
    gKalmanFilter.nu[STATE_ROLL]  = gKalmanFilter.turnSwitchMultiplier * gKalmanFilter.nu[STATE_ROLL];
    gKalmanFilter.nu[STATE_PITCH] = gKalmanFilter.turnSwitchMultiplier * gKalmanFilter.nu[STATE_PITCH];
    //gKalmanFilter.nu[STATE_YAW]   = gKalmanFilter.turnSwitchMultiplier * gKalmanFilter.nu[STATE_YAW];
}


/** ****************************************************************************
* @name: _GenerateObservationJacobian_RollAndPitch roll and pitch elements of
*        the measurement Jacobian (H)
* @brief
* TRACE:
* @param N/A
* @retval 1
******************************************************************************/
uint8_t _GenerateObservationJacobian_AHRS(void)
{
    // 
    real xPhi, yPhi;
    real uTheta;
    real xPsi, yPsi;
    real denom = 1.0;
    real multiplier = 1.0;

    // Set the values in DP to zero
    static BOOL initH = TRUE;
    if( initH ) {
        initH = FALSE;
        memset(gKalmanFilter.H, 0, sizeof(gKalmanFilter.H));
    }

    /// Note: H is 3x7
    /// Roll
    yPhi = (real)2.0 * ( gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q3] +
                         gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q1] );
    xPhi =  gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q0] + 
           -gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q1] +
           -gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q2] +
            gKalmanFilter.quaternion[Q3] * gKalmanFilter.quaternion[Q3];

    denom = yPhi*yPhi + xPhi*xPhi;
    if (denom < 1e-3) {
        // Based on drive-test data, the minimum value seen was 0.98 but the minimum
        //   values based on Matlab analysis is 1e-7.
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    /// Derivative of the roll-angle wrt quaternions
    gKalmanFilter.H[ROLL][STATE_Q0] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q1] +
                                                    -yPhi*gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[ROLL][STATE_Q1] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q0] + 
                                                     yPhi*gKalmanFilter.quaternion[Q1]);
    gKalmanFilter.H[ROLL][STATE_Q2] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q3] + 
                                                     yPhi*gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[ROLL][STATE_Q3] = multiplier * ( xPhi*gKalmanFilter.quaternion[Q2] +
                                                    -yPhi*gKalmanFilter.quaternion[Q3]);

    /// Pitch (including modifications for |q| = 1 constraint)
    uTheta = (real)2.0 * ( gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q3] -
                      gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q2] );

    denom = sqrt((real)1.0 - uTheta*uTheta);
    if (denom < 1e-3) {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    gKalmanFilter.H[PITCH][STATE_Q0] = multiplier * ( gKalmanFilter.quaternion[Q2] + 
                                                      uTheta * gKalmanFilter.quaternion[Q0] );
    gKalmanFilter.H[PITCH][STATE_Q1] = multiplier * (-gKalmanFilter.quaternion[Q3] + 
                                                      uTheta * gKalmanFilter.quaternion[Q1] );
    gKalmanFilter.H[PITCH][STATE_Q2] = multiplier * ( gKalmanFilter.quaternion[Q0] + 
                                                      uTheta * gKalmanFilter.quaternion[Q2] );
    gKalmanFilter.H[PITCH][STATE_Q3] = multiplier * (-gKalmanFilter.quaternion[Q1] + 
                                                      uTheta * gKalmanFilter.quaternion[Q3] );

    /// Yaw
    yPsi = (real)2.0 * ( gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q2] +
                         gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q3] );
    xPsi = (real)1.0 - (real)2.0 * ( gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q2] +
                                     gKalmanFilter.quaternion[Q3] * gKalmanFilter.quaternion[Q3] );
    denom = yPsi*yPsi + xPsi*xPsi;
    if (denom < 1e-3) {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    /// Derivative of the yaw-angle wrt quaternions
    gKalmanFilter.H[YAW][STATE_Q0] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q3] +
                                                   -yPsi*gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[YAW][STATE_Q1] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q2] +
                                                   -yPsi*gKalmanFilter.quaternion[Q1]);
    gKalmanFilter.H[YAW][STATE_Q2] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q1] + 
                                                    yPsi*gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[YAW][STATE_Q3] = multiplier * ( xPsi*gKalmanFilter.quaternion[Q0] + 
                                                    yPsi*gKalmanFilter.quaternion[Q3]);

    return 1;
}


// 
void _GenerateObservationCovariance_AHRS(void)
{
    //
    static real Rnom;

    // Only need to compute certain elements of R once
    static BOOL initR = TRUE;
    if (initR) {
        initR = FALSE;

        // Clear the values in R (in AHRS mode, there are 3 rows in the Jacobian)
        // Initialize the Process Covariance (Q) matrix
        // DEBUG: Probably not needed
        memset(gKalmanFilter.R, 0, sizeof(gKalmanFilter.R));

#ifdef INS_OFFLINE
        // This value is set based on the version string specified in the 
        //   simulation configuration file, ekfSim.cfg
        uint8_t sysRange = gSimulation.sysRange;
#else
        // This value is set based on the version string loaded into the unit
        //   via the system configuration load
        uint8_t sysRange = platformGetSysRange(); // from system config
#endif

        // Set the matrix, R, based on whether the system is operating in high
        //   or low-gain (is the acceleration above or below the acceleration
        //   threshold)
        //
        //   R-values are based on the variance of the roll and pitch angles
        //   generated from the sensor noise passed through the measurement
        //   model.  The values are multiplied by dt^2 as well as vary with the
        //   angle.  The value can be made large enough to work with all angles
        //   but it may slow the response.
        //
        // These values are found by passing the accelerometer VRW values
        //   (determined from a very limited data set) through a Matlab
        //   script which generates the roll and pitch noise based on the
        //   sensor noise.  The value below is the 1-sigma value at 0
        //   degrees.  The quadratic correction below is meant to increase
        //   R as the angle increases (due to the geometry and
        //   mathematical function used to compute the angle).
        //
        //   Matlab script: R_Versus_Theta.m
        switch (sysRange) {
            case _200_DPS_RANGE:
                // -200 VRW value (average x/y/z): 7.2e-4 [(m/sec)/rt-sec]
                Rnom = (real)(1.0e-06);  // (1.0e-3 [rad])^2
                Rnom = (real)9.82e-07;  // (9.91e-4)^2
                break;
            case _400_DPS_RANGE:
                    // -400 VRW value (average x/y/z): 8.8e-4 [(m/sec)/rt-sec]
                    Rnom = (real)(1.6e-06);  // (1.3e-3 [rad])^2
                Rnom = (real)1.54e-06;  // (1.24e-3)^2
                break;
        }
    }

    // High/low-gain switching to increase the EKF gain when the system is
    //   static
    //
    // Low-gain concept -- Increasing R reduces the gain and results in less of
    //                     the measurement being used to update the state.
    //
    //                     By passing drive-test data through the AHRS
    //                     simulation, a multiplier of 400.0 on R (when in low-
    //                     gain operation) seems to result in a better than
    //                     nominal solution (for the drive-test).
    if( (gAlgorithm.state == HIGH_GAIN_AHRS) ||
        (gAlgorithm.linAccelSwitch == TRUE) )
    {
        // High-Gain -- The process by which the values were selected is described above.
        gKalmanFilter.R[STATE_ROLL][STATE_ROLL]   = Rnom;
        gKalmanFilter.R[STATE_PITCH][STATE_PITCH] = gKalmanFilter.R[STATE_ROLL][STATE_ROLL];
    } else {
        // Low-Gain -- Increase R to reduce gain
        gKalmanFilter.R[STATE_ROLL][STATE_ROLL]   = (real)400.0 * Rnom;
        gKalmanFilter.R[STATE_PITCH][STATE_PITCH] = (real)400.0 * Rnom;
#ifdef INS_OFFLINE
        // Modify simulation values
        gKalmanFilter.R[STATE_ROLL][STATE_ROLL]   = gSimulation.measCovarMult_roll  * gKalmanFilter.R[STATE_ROLL][STATE_ROLL];
        gKalmanFilter.R[STATE_PITCH][STATE_PITCH] = gSimulation.measCovarMult_pitch * gKalmanFilter.R[STATE_PITCH][STATE_PITCH];
#endif
    }

    // Adjust the roll R-value based on the predicted pitch.  The multiplier is
    //   due to the fact that R changes with phi and theta.  Either make it big
    //   enough to work with all angles or vary it with the angle.
    real mult;
    real pitchSq = gKalmanFilter.eulerAngles[PITCH] * gKalmanFilter.eulerAngles[PITCH];
    mult = (real)(1.0 + 0.65 * pitchSq);

    //real mult = (real)1.0 + (real)0.65 * gKalmanFilter.eulerAngles[PITCH] * gKalmanFilter.eulerAngles[PITCH];
    gKalmanFilter.R[STATE_ROLL][STATE_ROLL] = mult * mult * gKalmanFilter.R[STATE_ROLL][STATE_ROLL];

    // Yaw
    // ------ From NovAtel's description of BESTVEL: ------
    //   Velocity (speed and direction) calculations are computed from either
    //   Doppler or carrier phase measurements rather than from pseudorange
    //   measurements. Typical speed accuracies are around 0.03m/s (0.07 mph,
    //   0.06 knots).
    //
    //   Direction accuracy is derived as a function of the vehicle speed. A
    //   simple approach would be to assume a worst case 0.03 m/s cross-track
    //   velocity that would yield a direction error function something like:
    //
    //   d (speed) = tan-1(0.03/speed)
    //
    //   For example, if you are flying in an airplane at a speed of 120 knots
    //   or 62 m/s, the approximate directional error will be:
    //
    //   tan-1 (0.03/62) = 0.03 degrees
    //
    //   Consider another example applicable to hiking at an average walking
    //   speed of 3 knots or 1.5 m/s. Using the same error function yields a
    //   direction error of about 1.15 degrees.
    //
    //   You can see from both examples that a faster vehicle speed allows for a
    //   more accurate heading indication. As the vehicle slows down, the
    //   velocity information becomes less and less accurate. If the vehicle is
    //   stopped, a GNSS receiver still outputs some kind of movement at speeds
    //   between 0 and 0.5 m/s in random and changing directions. This
    //   represents the noise and error of the static position.

    // ----- Yaw -----
    // CHANGED TO SWITCH BETWEEN GPS AND MAG UPDATES
    if( useMagHeading || newGPSDataFlag == 0 ) {
        // MAGNETOMETERS
        if( (gAlgorithm.state == HIGH_GAIN_AHRS) ||
            (gAlgorithm.linAccelSwitch == TRUE) )
        {
            // --- High-Gain ---
            //gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]   = (real)1.0e-3;  // v14.6 values
            //gKalmanFilter.R[STATE_YAW][STATE_YAW]   = (real)2.0e-05; //(for sig = 1e-3)
            gKalmanFilter.R[STATE_YAW][STATE_YAW] = (real)1.0e-2;  // jun4
        } else {
            // --- Low-Gain ---
            gKalmanFilter.R[STATE_YAW][STATE_YAW]   = (real)1.0e-1; // v14.6 values
            //gKalmanFilter.R[STATE_YAW][STATE_YAW]   = (real)0.02; //(for sig = 1e-3)
        }

        // For 'large' roll/pitch angles, increase R-yaw to decrease the effect
        //   of update due to potential uncompensated z-axis magnetometer
        //   readings from affecting the yaw-update.
        if( ( gKalmanFilter.eulerAngles[ROLL]  > TEN_DEGREES_IN_RAD ) ||
            ( gKalmanFilter.eulerAngles[PITCH] > TEN_DEGREES_IN_RAD ) )
        {
            gKalmanFilter.R[STATE_YAW][STATE_YAW]   = (real)0.2;
        }
    } else {
        // When updating using GPS readings, newGPSDataFlag will set the error
        //   to zero when performing the VG-type updates as it is reset after
        //   each pass through the gps logic (above).
        if( newGPSDataFlag ) {
            float temp = (float)atan( 0.05 / gEKFInputData.rawGroundSpeed );
            gKalmanFilter.R[STATE_YAW][STATE_YAW] = temp; // * temp;
        }
    }
}


// 
void _GenerateObservationCovariance_INS(void)
{
    // Only need to compute certain elements of R once
    static BOOL initR = TRUE;
    if (initR) {
        initR = FALSE;

        gKalmanFilter.R[STATE_RX][STATE_RX] = (real)R_VALS_GPS_POS_X;
        gKalmanFilter.R[STATE_RY][STATE_RY] = gKalmanFilter.R[STATE_RX][STATE_RX];
        gKalmanFilter.R[STATE_RZ][STATE_RZ] = gKalmanFilter.R[STATE_RX][STATE_RX];

        gKalmanFilter.R[STATE_VX][STATE_VX] = (real)R_VALS_GPS_VEL_X;
        gKalmanFilter.R[STATE_VY][STATE_VY] = gKalmanFilter.R[STATE_VX][STATE_VX];
        gKalmanFilter.R[STATE_VZ][STATE_VZ] = gKalmanFilter.R[STATE_VX][STATE_VX];
    }

    // Use the GPS-provided horizontal and vertical accuracy values to populate
    //   the covariance values.
    gKalmanFilter.R[STATE_RX][STATE_RX] = gEKFInputData.GPSHorizAcc * gEKFInputData.GPSHorizAcc;
    gKalmanFilter.R[STATE_RY][STATE_RY] = gKalmanFilter.R[STATE_RX][STATE_RX];
    gKalmanFilter.R[STATE_RZ][STATE_RZ] = gEKFInputData.GPSVertAcc * gEKFInputData.GPSVertAcc;

    // Scale the best velocity error by HDOP then multiply by the z-axis angular
    //   rate PLUS one (to prevent the number from being zero) so the velocity
    //   update during high-rate turns is reduced.
    float stddev_vx = gEKFInputData.GPSHVelAcc * ((real)1.0 + fabs(gAlgorithm.filteredYawRate) * (real)RAD_TO_DEG);
    gKalmanFilter.R[STATE_VX][STATE_VX] = stddev_vx * stddev_vx;
    gKalmanFilter.R[STATE_VY][STATE_VY] = gKalmanFilter.R[STATE_VX][STATE_VX];

    // z-axis velocity isn't really a function of yaw-rate and hdop
    //gKalmanFilter.R[STATE_VZ][STATE_VZ] = gKalmanFilter.R[STATE_VX][STATE_VX];
    gKalmanFilter.R[STATE_VZ][STATE_VZ] = gEKFInputData.GPSVVelAcc * gEKFInputData.GPSVVelAcc;
}


//
uint8_t rowNum, colNum, multIndex;

real S_3x3[3][3], SInverse_3x3[3][3];
real PxHTranspose[ROWS_IN_P][ROWS_IN_H], HxPxHTranspose[ROWS_IN_H][ROWS_IN_H];
real KxH[NUMBER_OF_EKF_STATES][COLS_IN_H] = {{ 0.0 }};
real deltaP_tmp[ROWS_IN_P][COLS_IN_P];
real stateUpdate[NUMBER_OF_EKF_STATES];

void Update_Att(void)
{
    // Calculate the elements in the H and R matrices
    //                                             Matrix sizes for an Euler-angle based AHRS solution:
    _GenerateObservationJacobian_AHRS();     // gKF.H: 3x16
    _GenerateObservationCovariance_AHRS();   // gKF.R: 3x3

    // This solution consists of an integrated roll/pitch/yaw solution
    // S = H*P*HTrans + R (However the matrix math can be simplified since
    //                     H is very sparse!  P is fully populated)
    // Update P from the P, H, and R matrices: P = HxPxHTranspose + R
    //   1) PxHTranspose is computed first
    memset(PxHTranspose, 0, sizeof(PxHTranspose));
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        for (colNum = 0; colNum < ROWS_IN_H; colNum++) {
            for (multIndex = RLE_H[colNum][0]; multIndex <= RLE_H[colNum][1]; multIndex++) {
                PxHTranspose[rowNum][colNum] = PxHTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.H[colNum][multIndex];
            }
        }
    }

    //   2) Use gKalmanFilter.P as a temporary variable to hold HxPxHTranspose
    //      to reduce the number of "large" variables on the heap
#if 0
    for (rowNum = 0; rowNum < 3; rowNum++) {
        for (colNum = 0; colNum < 3; colNum++) {
            HxPxHTranspose[rowNum][colNum] = 0.0;
            for (multIndex = RLE_H[rowNum][0]; multIndex <= RLE_H[rowNum][1]; multIndex++) {
                HxPxHTranspose[rowNum][colNum] = HxPxHTranspose[rowNum][colNum] +
                    gKalmanFilter.H[rowNum][multIndex] * PxHTranspose[multIndex][colNum];
            }
        }
    }
#else
    // HPH' is symmetric so only need to multiply one half and reflect the values
    //   across the diagonal
    for (rowNum = 0; rowNum < 3; rowNum++) {
        for (colNum = rowNum; colNum < 3; colNum++) {
            HxPxHTranspose[rowNum][colNum] = 0.0;
            for (multIndex = RLE_H[rowNum][0]; multIndex <= RLE_H[rowNum][1]; multIndex++) {
                HxPxHTranspose[rowNum][colNum] = HxPxHTranspose[rowNum][colNum] +
                    gKalmanFilter.H[rowNum][multIndex] * PxHTranspose[multIndex][colNum];
            }
            HxPxHTranspose[colNum][rowNum] = HxPxHTranspose[rowNum][colNum];
        }
    }
#endif

    // S = HxPxHTranspose + R (rows 7:10 and cols 7:10 of P PLUS diagonal of R)
    S_3x3[ROLL][ROLL]   = HxPxHTranspose[ROLL][ROLL]   + gKalmanFilter.R[STATE_ROLL][STATE_ROLL];
    S_3x3[ROLL][PITCH]  = HxPxHTranspose[ROLL][PITCH];
    S_3x3[ROLL][YAW]    = HxPxHTranspose[ROLL][YAW];

    S_3x3[PITCH][ROLL]  = HxPxHTranspose[PITCH][ROLL];
    S_3x3[PITCH][PITCH] = HxPxHTranspose[PITCH][PITCH] + gKalmanFilter.R[STATE_PITCH][STATE_PITCH];
    S_3x3[PITCH][YAW]   = HxPxHTranspose[PITCH][YAW];

    S_3x3[YAW][ROLL]    = HxPxHTranspose[YAW][ROLL];
    S_3x3[YAW][PITCH]   = HxPxHTranspose[YAW][PITCH];
    S_3x3[YAW][YAW]     = HxPxHTranspose[YAW][YAW]     + gKalmanFilter.R[STATE_YAW][STATE_YAW];

    // Invert the S-Matrix (replace with sequential update)
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute the Kalman gain: K = P*HTrans*SInv
#if 1
    AxB( &PxHTranspose[0][0],
         &SInverse_3x3[0][0],
         ROWS_IN_P, ROWS_IN_H, ROWS_IN_H,
         &gKalmanFilter.K[0][0] );
#else
    // This doesn't work.  Need to figure out why.
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        for (colNum = 0; colNum < ROWS_IN_H; colNum++) {
            gKalmanFilter.K[rowNum][colNum] = 0.0;
            for (multIndex = 0; multIndex < ROWS_IN_H; multIndex++) {
                gKalmanFilter.K[rowNum][colNum] = gKalmanFilter.K[rowNum][colNum] +
                                                     PxHTranspose[rowNum][multIndex] * 
                                                     SInverse_3x3[multIndex][colNum];
            }
        }
    }
#endif

    // Compute attitude-quaternion updates: Dx = K*nu
    // NOTE: Can access nu in the elements that the attitude error is stored BUT the
    //       value of ROWS_IN_H must be correct or the multiplication will be wrong
    AxV( &gKalmanFilter.K[0][0],
         &gKalmanFilter.nu[STATE_ROLL],
         NUMBER_OF_EKF_STATES, ROWS_IN_H,
         &stateUpdate[0] );

    // Update states based on computed deltas
    // --- attitude quaternions (q = q + Dq) ---
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate[STATE_Q3];

    // Normalize q
    QuatNormalize(&gKalmanFilter.quaternion[0]);

    // --- Angular-rate bias (wBias = wBias = DwBias) ---
    //     If magnetometers are not used then set the rate bias to zero???
    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate[STATE_WBY];
// FIXME: what if external magnetometers are used?
    if (magUsedInAlgorithm()) {
        gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate[STATE_WBZ];
    } else {
        gKalmanFilter.rateBias_B[Z_AXIS] = (real)0.0;
    }

    // Update covariance: P = P + DP = P - K*H*P
    // KxH = gKF.K * gKF.H;
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    memset(KxH, 0, sizeof(KxH));
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) {
        for (colNum = RLE_KxH[rowNum][0]; colNum <= RLE_KxH[rowNum][1]; colNum++) {
            for (multIndex = 0; multIndex < ROWS_IN_H; multIndex++) {
                KxH[rowNum][colNum] = KxH[rowNum][colNum] +
                    gKalmanFilter.K[rowNum][multIndex] * gKalmanFilter.H[multIndex][colNum];
            }
        }
    }

// KxH is sparse too.  Only cols 6 to 9 are populated.
    // Only modify the rows and cols that correspond to q and wb (the remainder of DP
    //   will be zero).  This is an attempt to fix the slow response in roll and pitch
    //   due to 'fast' motions that Parvez brought to my attention.  If this does not
    //   fix it then maybe the values of R are too large when the system is at rest
    //   --> change with aMag???
    // deltaP = KxH * gKF.P;
    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
#if 0
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) {
        for (colNum = 0; colNum < COLS_IN_P; colNum++) {
            for (multIndex = RLE_KxH[rowNum][0]; multIndex <= RLE_KxH[rowNum][1]; multIndex++) {
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    KxH[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#else
    // deltaP is symmetric so only need to multiply one half and reflect the values
    //   across the diagonal
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) {
        for (colNum = rowNum; colNum < COLS_IN_P; colNum++) {
            for (multIndex = RLE_KxH[rowNum][0]; multIndex <= RLE_KxH[rowNum][1]; multIndex++) {
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    KxH[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
            deltaP_tmp[colNum][rowNum] = deltaP_tmp[rowNum][colNum];
        }
    }
#endif

    // Commenting out the following doesn't really save any time
//    // Zero out P values not affected by attitude information
//    memset(deltaP, 0, sizeof(deltaP));
//    for (rowNum = STATE_Q0; rowNum <= STATE_WBZ; rowNum++) {
//        for (colNum = STATE_Q0; colNum <= STATE_WBZ; colNum++) {
//            deltaP[rowNum][colNum] = deltaP_tmp[rowNum][colNum];
//        }
//    }

#if 0
    // gKF.P = gKF.P - deltaP;
    AMinusB( &gKalmanFilter.P[0][0],
             &deltaP_tmp[0][0], //&deltaP[0][0],
             ROWS_IN_P, COLS_IN_P,
             &gKalmanFilter.P[0][0]);

    // Ensure P is symmetric
    ForceMatrixSymmetry( &gKalmanFilter.P[0][0],
                         ROWS_IN_P, COLS_IN_P );
#else
    // P is symmetric so only need to multiply one half and reflect the values
    //   across the diagonal
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        for (colNum = rowNum; colNum < COLS_IN_P; colNum++) {
            gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] -
                                              deltaP_tmp[rowNum][colNum];
        }
        gKalmanFilter.P[colNum][rowNum] = gKalmanFilter.P[rowNum][colNum];
    }
#endif
}


real stateUpdate_pos[NUMBER_OF_EKF_STATES], stateUpdate_vel[NUMBER_OF_EKF_STATES];
real K_pos[NUMBER_OF_EKF_STATES][3], K_vel[NUMBER_OF_EKF_STATES][3];
real KxHxP[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];

// The position update only allows us to update the position and velocity states (along
//   with Pr and Pv).  Want to verify this...
void Update_Pos(void)
{
    real nu_pos[NUM_AXIS];
    nu_pos[0] = gKalmanFilter.nu[STATE_RX];
    nu_pos[1] = gKalmanFilter.nu[STATE_RY];
    nu_pos[2] = gKalmanFilter.nu[STATE_RZ];

    // ++++++++++++++++++++++ POSITION ++++++++++++++++++++++
    // Step 1) Position% Step 1) Position
    // S1 = H1*gKF.P*H1' + R1;   // Top 3 rows of the first 3 cols of P + R
    // K1 = gKF.P*H1'*inv(S1);   // ( first 3 cols of P ) * S1Inverse
    // P1 = (eye(16) - K1*H1) * gKF.P;

    // S1 = H1*gKF.P*H1' + R1;
    S_3x3[0][0] = gKalmanFilter.P[STATE_RX][STATE_RX] + gKalmanFilter.R[STATE_RX][STATE_RX];
    S_3x3[0][1] = gKalmanFilter.P[STATE_RX][STATE_RY];
    S_3x3[0][2] = gKalmanFilter.P[STATE_RX][STATE_RZ];

    S_3x3[1][0] = gKalmanFilter.P[STATE_RY][STATE_RX];
    S_3x3[1][1] = gKalmanFilter.P[STATE_RY][STATE_RY] + gKalmanFilter.R[STATE_RY][STATE_RY];
    S_3x3[1][2] = gKalmanFilter.P[STATE_RY][STATE_RZ];

    S_3x3[2][0] = gKalmanFilter.P[STATE_RZ][STATE_RX];
    S_3x3[2][1] = gKalmanFilter.P[STATE_RZ][STATE_RY];
    S_3x3[2][2] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + gKalmanFilter.R[STATE_RZ][STATE_RZ];

    // S1_Inverse
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute K1 = ( gKF.P*H1' ) * S1Inverse = ( first 3 cols of P ) * S1Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) {
            K_pos[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the position states are used
            //   in the calculation
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++) {
                K_pos[rowNum][colNum] = K_pos[rowNum][colNum] +
                                        gKalmanFilter.P[rowNum][multIndex] * SInverse_3x3[multIndex - STATE_RX][colNum];
            }
        }
    }

    // Compute the intermediate state update, stateUpdate_pos
    AxB(&K_pos[0][0], &nu_pos[0], NUMBER_OF_EKF_STATES, 3, 1, &stateUpdate_pos[0]);

    memset(KxHxP, 0, sizeof(KxHxP));
    // Update the intermediate covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = 0; colNum < NUMBER_OF_EKF_STATES; colNum++) {
//            KxHxP[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the position states are used
            //   in the calculation
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_pos[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }

    AMinusB(&gKalmanFilter.P[0][0], &KxHxP[0][0], NUMBER_OF_EKF_STATES, NUMBER_OF_EKF_STATES, &gKalmanFilter.P[0][0]);
    // ++++++++++++++++++++++ END OF POSITION ++++++++++++++++++++++

    // Update states
    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + stateUpdate_pos[STATE_RX];
    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + stateUpdate_pos[STATE_RY];
    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + stateUpdate_pos[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + stateUpdate_pos[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + stateUpdate_pos[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + stateUpdate_pos[STATE_VZ];

    //gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate_pos[STATE_Q0];
    //gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate_pos[STATE_Q1];
    //gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate_pos[STATE_Q2];
    //gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate_pos[STATE_Q3];
    //
    //// Normalize quaternion and force q0 to be positive
    //QuatNormalize(gKalmanFilter.quaternion);
    //
    //gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate_pos[STATE_WBX];
    //gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate_pos[STATE_WBY];
    //gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate_pos[STATE_WBZ];
    //
    //gKalmanFilter.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] + stateUpdate_pos[STATE_ABX];
    //gKalmanFilter.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] + stateUpdate_pos[STATE_ABY];
    //gKalmanFilter.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] + stateUpdate_pos[STATE_ABZ];
}


// The velocity update only allows us to update the velocity, attitude, and acceleration-
//   bias states (along with Pv, Pq, and Pab).  Wwant to verify this...
void Update_Vel(void)
{
    real nu_vel[NUM_AXIS];
    nu_vel[0] = gKalmanFilter.nu[STATE_VX];
    nu_vel[1] = gKalmanFilter.nu[STATE_VY];
    nu_vel[2] = gKalmanFilter.nu[STATE_VZ];

    // ++++++++++++++++++++++ VELOCITY ++++++++++++++++++++++
    // Step 2) Velocity
    //S2 = H2*P1*H2' + R2; (4th, 5th, and 6th rows of the 4th, 5th, and 6th cols of P1)
    //K2 = P1*H2'*inv(S2);
    //P2 = (eye(16) - K2*H2) * P1;

    // S2 = H2*P1*H2' + R2;
    S_3x3[0][0] = gKalmanFilter.P[STATE_VX][STATE_VX] + gKalmanFilter.R[STATE_VX][STATE_VX];
    S_3x3[0][1] = gKalmanFilter.P[STATE_VX][STATE_VY];
    S_3x3[0][2] = gKalmanFilter.P[STATE_VX][STATE_VZ];

    S_3x3[1][0] = gKalmanFilter.P[STATE_VY][STATE_VX];
    S_3x3[1][1] = gKalmanFilter.P[STATE_VY][STATE_VY] + gKalmanFilter.R[STATE_VY][STATE_VY];
    S_3x3[1][2] = gKalmanFilter.P[STATE_VY][STATE_VZ];

    S_3x3[2][0] = gKalmanFilter.P[STATE_VZ][STATE_VX];
    S_3x3[2][1] = gKalmanFilter.P[STATE_VZ][STATE_VY];
    S_3x3[2][2] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + gKalmanFilter.R[STATE_VZ][STATE_VZ];

    // S2_Inverse
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute K2 = ( P1*H2' ) * S2Inverse = ( 4th, 5th, and 6th cols of P1 ) * S2Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) {
            K_vel[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the velocity states are used
            //   in the calculation
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) {
                K_vel[rowNum][colNum] = K_vel[rowNum][colNum] +
                                        gKalmanFilter.P[rowNum][multIndex] * SInverse_3x3[multIndex - STATE_VX][colNum];
            }
        }
    }

    // Compute the intermediate state update
    AxB(&K_vel[0][0], &nu_vel[0], NUMBER_OF_EKF_STATES, 3, 1, &stateUpdate_vel[0]);

    memset(KxHxP, 0, sizeof(KxHxP));

    // Update the intermediate covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = 0; colNum < NUMBER_OF_EKF_STATES; colNum++) {
            // H is sparse so only the columns of P associated with the velocity states are used
            //   in the calculation
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_vel[rowNum][multIndex - STATE_VX] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }

    // P2 = P2 - KxHxP2
    AMinusB(&gKalmanFilter.P[0][0], &KxHxP[0][0], NUMBER_OF_EKF_STATES, NUMBER_OF_EKF_STATES, &gKalmanFilter.P[0][0]);
    // ++++++++++++++++++++++ END OF VELOCITY ++++++++++++++++++++++

    // Update states
    //gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + stateUpdate_vel[STATE_RX];
    //gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + stateUpdate_vel[STATE_RY];
    //gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + stateUpdate_vel[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + stateUpdate_vel[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + stateUpdate_vel[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + stateUpdate_vel[STATE_VZ];

    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate_vel[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate_vel[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate_vel[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate_vel[STATE_Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    //gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate_vel[STATE_WBX];
    //gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate_vel[STATE_WBY];
    //gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate_vel[STATE_WBZ];

    gKalmanFilter.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] + stateUpdate_vel[STATE_ABX];
    gKalmanFilter.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] + stateUpdate_vel[STATE_ABY];
    gKalmanFilter.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] + stateUpdate_vel[STATE_ABZ];
}


// Conversion from turn-rate threshold (values loaded into gConfiguration) to
//   decimal value in [rad/sec]:
//
//   thresh_rad = ( 10.0 * pi/180 );   % 0.1745
//   thresh_counts = floor( thresh_rad * ( 2^16 / (2*pi) ) );   % 1820
//   thresh_rad = thresh_counts * ( 2*pi / 2^16 )   % 1820 * (2*pi) / 2^16 = 0.1745

real TILT_YAW_SWITCH_GAIN = (real)0.05;

// TurnSwitch.m
static void _TurnSwitch(void)
{
    static real minSwitch = (real)0.0, maxSwitch = (real)0.0;
    static real turnSwitchThresholdPast = (real)0.0;
    static real linInterpSF;

    real absYawRate;
    real turnSwitchScaleFactor;

    // gKF.filteredYawRate (calculated in the prediction stage)
    absYawRate = fabs(gAlgorithm.filteredYawRate);

    // In case the user changes the TST during operation
    if (gAlgorithm.turnSwitchThreshold != turnSwitchThresholdPast) {
        turnSwitchThresholdPast = gAlgorithm.turnSwitchThreshold;

        // Example conversion: ( 1820*12868 / 2^27 ) * ( 180/pi )
        minSwitch = gAlgorithm.turnSwitchThreshold * (real)(DEG_TO_RAD);   // angle in radians
        maxSwitch = (real)2.0 * minSwitch;   // angle in radians

        linInterpSF = ((real)1.0 - TILT_YAW_SWITCH_GAIN) / (maxSwitch - minSwitch);
    }

    // Linear interpolation if the yawRate is above the specified threshold
    if ((gAlgorithm.state > HIGH_GAIN_AHRS) && (absYawRate > minSwitch))
    {
        gBitStatus.swStatus.bit.turnSwitch = TRUE;
        //        std::cout << "TurnSwitch (INS): Activated\n";

        // When the rate is below the maximum rate defined by
        //   turnSwitchThreshold, then generate a scale-factor that is between
        //   ( 1.0 - G ) and 0.0 (based on absYawRate).  If it is above
        //   'maxSwitch' then the SF is zero.
        if (absYawRate < maxSwitch) {
            turnSwitchScaleFactor = linInterpSF * (maxSwitch - absYawRate);
        } else {
            // yaw-rate is above maxSwitch ==> no gain
            turnSwitchScaleFactor = (real)0.0;
        }

        // Specify the multiplier so it is between G and 1.0
        gKalmanFilter.turnSwitchMultiplier = TILT_YAW_SWITCH_GAIN + turnSwitchScaleFactor;
    } else {
        gBitStatus.swStatus.bit.turnSwitch = FALSE;
        gKalmanFilter.turnSwitchMultiplier = (real)1.0;
    }
}


static real _UnwrapAttitudeError(real attitudeError)
{
    while (fabs(attitudeError) > PI)
    {
        if (attitudeError > PI) {
            attitudeError = attitudeError - (real)TWO_PI;
        } else if (attitudeError < -PI) {
            attitudeError = attitudeError + (real)TWO_PI;
        }
    }

    return attitudeError;
}


static real _LimitValue(real value, real limit)
{
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    }

    return value;
}


// Returns true when the system is ready to update (based on the timer values
//   and the desired update rate)
static BOOL _CheckForUpdateTrigger(uint8_t updateRate)
{
    //
    uint8_t oneHundredHzCntr;
    uint8_t updateFlag = 0;

    //
    switch( updateRate ){
        // ten-hertz update
        case 10:
            if( timer.subFrameCntr == 0 ) {
                updateFlag = 1;
            }
            break;

        // twenty-hertz update
        case 20:
            if( timer.subFrameCntr == 0 ||
                timer.subFrameCntr == 5 )
            {
                updateFlag = 1;
            }
            break;

        // twenty-hertz update
        case 25:
            oneHundredHzCntr = 10 * timer.tenHertzCntr +
                                    timer.subFrameCntr;

            // 
            if( oneHundredHzCntr ==  0 ||
                oneHundredHzCntr ==  4 ||
                oneHundredHzCntr ==  8 ||
                oneHundredHzCntr == 12 ||
                oneHundredHzCntr == 16 ||
                oneHundredHzCntr == 20 ||
                oneHundredHzCntr == 24 ||
                oneHundredHzCntr == 28 ||
                oneHundredHzCntr == 32 ||
                oneHundredHzCntr == 36 ||
                oneHundredHzCntr == 40 ||
                oneHundredHzCntr == 44 ||
                oneHundredHzCntr == 48 ||
                oneHundredHzCntr == 52 ||
                oneHundredHzCntr == 56 ||
                oneHundredHzCntr == 60 ||
                oneHundredHzCntr == 64 ||
                oneHundredHzCntr == 68 ||
                oneHundredHzCntr == 72 ||
                oneHundredHzCntr == 76 ||
                oneHundredHzCntr == 80 ||
                oneHundredHzCntr == 84 ||
                oneHundredHzCntr == 88 ||
                oneHundredHzCntr == 92 ||
                oneHundredHzCntr == 96 )
            {
                updateFlag = 1;
            }
            break;

        // fifty-hertz update
        case 50:
            if( timer.subFrameCntr == 0 ||
                timer.subFrameCntr == 2 ||
                timer.subFrameCntr == 4 ||
                timer.subFrameCntr == 6 ||
                timer.subFrameCntr == 8 )
            {
                updateFlag = 1;
            }
            break;

        // fifty-hertz update
        case 100:
            updateFlag = 1;
            break;
    }

    return updateFlag;
}

