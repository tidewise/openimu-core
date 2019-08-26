/*
 * File:   TransformationMath.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include <math.h>
#include "algorithm.h"
#include "TransformationMath.h"
#include "MatrixMath.h"
#include "VectorMath.h"
#include "MagAlign.h"
#include "Indices.h"
#include "EKF_Algorithm.h"   // for gKalmanFilter
#include "GpsData.h"
#include "WorldMagneticModel.h"

#include "arm_math.h"
#include "FastInvTrigFuncs.h"

static void _TransformMagFieldToPerpFrame( real* MagFieldVector,
                                           real* nedMagFieldVector,
                                           real* EulerAngles );

#define  CORRECT_IN_BODY_FRAME

/** ****************************************************************************
* @name: _FieldVectorsToEulerAngles Compute roll and pitch from the gravity
*        vector. The yaw is computed from the  magnetic field vector. If the
*        system does not have magnetometers yaw = zero. Correction for
*        hard/soft-iron is only used to adjust the yaw-angle, knowledge of the
*        hard/soft-iron is not enough to correct magnetic-field.
* TRACE:
* @param [in] GravityUnitVector_q30 unit vector components of the body to gravity
* @param [in] MagFieldVector_q27
* @param [out] EulerAngles_q29
* @retval 1 if magnetometers are used, otherwise it returns a zero.
******************************************************************************/
BOOL FieldVectorsToEulerAngles( real* GravityVector,
                                real* magFieldVector,
                                uint8_t usePredFlag,
                                real* eulerAngles )
{
    real nedMagFieldVector[NUM_AXIS] = {0.0};
    real tmp[2];
    real xMag, yMag;
    real gravityUnitVector[NUM_AXIS] = {0.0};

    // At rest, gravity is the negative of the acceleration vector: gHat = -a_Body
    VectorNormalize(GravityVector, gravityUnitVector);
    gravityUnitVector[0] = -gravityUnitVector[0];
    gravityUnitVector[1] = -gravityUnitVector[1];
    gravityUnitVector[2] = -gravityUnitVector[2];

    /// calculate roll and pitch from the gravity-vector
#ifndef FAST_MATH
    eulerAngles[ROLL]  = (real)( atan2( -GravityVector[Y_AXIS],
                                        -GravityVector[Z_AXIS] ) );
    eulerAngles[PITCH] = (real)( -asin( gravityUnitVector[X_AXIS] ) );
#else
    eulerAngles[ROLL]  = (real)( fatan2_rad( -GravityVector[Y_AXIS],
                                             -GravityVector[Z_AXIS] ) );
    eulerAngles[PITCH] = (real)( -fasin_rad( gravityUnitVector[X_AXIS] ) );
#endif

    /// yaw angle (unit magnetic field) in the plane normal to the gravity vector
    if ( magUsedInAlgorithm() )
    {
#ifdef CORRECT_IN_BODY_FRAME
        // Correct for hard/soft-iron effects here (in body-frame)
        // Hard-iron/Soft-iron corrections was _calcMagHeading()
        tmp[X_AXIS] = magFieldVector[X_AXIS] - gMagAlign.hardIronBias[X_AXIS];
        tmp[Y_AXIS] = magFieldVector[Y_AXIS] - gMagAlign.hardIronBias[Y_AXIS];

        magFieldVector[0] = gMagAlign.SF[0] * tmp[X_AXIS] + gMagAlign.SF[1] * tmp[Y_AXIS];
        magFieldVector[1] = gMagAlign.SF[2] * tmp[X_AXIS] + gMagAlign.SF[3] * tmp[Y_AXIS];
#endif

        // Transform the magnetic field vector from the body-frame to the plane normal to the gravity vector
        if( usePredFlag ) {
            _TransformMagFieldToPerpFrame( magFieldVector, nedMagFieldVector, gKalmanFilter.eulerAngles );
        } else {
            _TransformMagFieldToPerpFrame( magFieldVector, nedMagFieldVector, eulerAngles );
        }

#ifndef CORRECT_IN_BODY_FRAME
        // Hard-iron/Soft-iron corrections was _calcMagHeading()
        tmp[X_AXIS] = nedMagFieldVector[X_AXIS] - gMagAlign.hardIronBias[X_AXIS];
        tmp[Y_AXIS] = nedMagFieldVector[Y_AXIS] - gMagAlign.hardIronBias[Y_AXIS];

        xMag = gMagAlign.SF[0] * tmp[X_AXIS] + gMagAlign.SF[1] * tmp[Y_AXIS];
        yMag = gMagAlign.SF[2] * tmp[X_AXIS] + gMagAlign.SF[3] * tmp[Y_AXIS];
#else
        xMag = nedMagFieldVector[X_AXIS];
        yMag = nedMagFieldVector[Y_AXIS];
#endif

        // The negative of the angle the vector makes with the unrotated (psi = 0)
        //   frame is the yaw-angle of the initial frame.
#ifndef FAST_MATH
        eulerAngles[YAW] = (real)( -atan2( yMag, xMag ) );
#else
        eulerAngles[YAW] = (real)( -fatan2_rad( yMag, xMag ) );
#endif
    } else {
        // For VG, set the measured heading to the predicted heading (this
        //   forces the error to zero)
        eulerAngles[YAW] = gKalmanFilter.eulerAngles[YAW];
    }

    return(magUsedInAlgorithm());
}


// Transform the magnetic field from the body-frame to the 'perpendicular' frame
//   (whose z-axis is aligned with the NED-frame but the x and y-axes are unaligned).
static void _TransformMagFieldToPerpFrame( real* magFieldVector,
                                           real* nedMagFieldVector,
                                           real* eulerAngles )
{
    real sinRoll, cosRoll;
    real sinPitch, cosPitch;
    real temp;

#ifndef FAST_MATH
    sinRoll  = (real)(sin( eulerAngles[ROLL] ));
    cosRoll  = (real)(cos( eulerAngles[ROLL] ));
    sinPitch = (real)(sin( eulerAngles[PITCH] ));
    cosPitch = (real)(cos( eulerAngles[PITCH] ));
#else
    sinRoll  = (real)(arm_sin_f32( eulerAngles[ROLL] ));
    cosRoll  = (real)(arm_cos_f32( eulerAngles[ROLL] ));
    sinPitch = (real)(arm_sin_f32( eulerAngles[PITCH] ));
    cosPitch = (real)(arm_cos_f32( eulerAngles[PITCH] ));
#endif

    temp = sinRoll * magFieldVector[Y_AXIS] + cosRoll * magFieldVector[Z_AXIS];

    nedMagFieldVector[X_AXIS] =  cosPitch * magFieldVector[X_AXIS] + sinPitch * temp;
    nedMagFieldVector[Y_AXIS] =  cosRoll  * magFieldVector[Y_AXIS] - sinRoll  * magFieldVector[Z_AXIS];
    nedMagFieldVector[Z_AXIS] = -sinPitch * magFieldVector[X_AXIS] + cosPitch * temp;
}


///** ***************************************************************************
//* @name LLA_To_R_EinN returns the rotation matrix that converts from the Earth-
//*                     Centered, Earth-Fixed [m] to the North/East/Down-Frame [m]
//*
//* @details Pre calculated all non-changing constants and unfolded the matrices
//* @param [in] LLA - array with the Latitude, Longitude and Altitude [rad]
//* @param [out] R_EinN - rotation matrix from ECEF to NED
//* @retval always 1
//******************************************************************************/
BOOL LLA_To_R_EinN( double* llaRad,
                    real*  R_EinN )
{
    real sinLat, cosLat;
    real sinLon, cosLon;

    sinLat = (real)sin(llaRad[LAT]);
    cosLat = (real)cos(llaRad[LAT]);
    sinLon = (real)sin(llaRad[LON]);
    cosLon = (real)cos(llaRad[LON]);

    // First row
    *(R_EinN + 0 * 3 + 0) = -sinLat * cosLon;
    *(R_EinN + 0 * 3 + 1) = -sinLat * sinLon;
    *(R_EinN + 0 * 3 + 2) =  cosLat;

    // Second row
    *(R_EinN + 1 * 3 + 0) = -sinLon;
    *(R_EinN + 1 * 3 + 1) =  cosLon;
    *(R_EinN + 1 * 3 + 2) =  0.0;

    // Third row
    *(R_EinN + 2 * 3 + 0) = -cosLat * cosLon;
    *(R_EinN + 2 * 3 + 1) = -cosLat * sinLon;
    *(R_EinN + 2 * 3 + 2) = -sinLat;

    return 1;
}


///** ***************************************************************************
//* @name LLA_To_R_NinE returns a rotation matrix North [m], East [m] down [m] to
//*       Earth Centered Earth Fixed [m] coordinates
//* @details Pre calculated all non-changing constants and unfolded the matrices
//* @param [in] LLA - array with the Latitude, Longitude and Altitude [rad]
//* @param [out] InvRne - rotation matrix from NED to ECEF
//* @retval always 1
//******************************************************************************/
BOOL LLA_To_R_NinE( double* llaRad,
                    real* R_NinE )
{
    real sinLat, cosLat;
    real sinLon, cosLon;

    sinLat = (real)(sin((real)llaRad[LAT]));
    cosLat = (real)(cos((real)llaRad[LAT]));
    sinLon = (real)(sin((real)llaRad[LON]));
    cosLon = (real)(cos((real)llaRad[LON]));

    *(R_NinE + 0 * 3 + 0) = -sinLat * cosLon;
    *(R_NinE + 0 * 3 + 1) = -sinLon;
    *(R_NinE + 0 * 3 + 2) = -cosLat * cosLon;

    *(R_NinE + 1 * 3 + 0) = -sinLat * sinLon;
    *(R_NinE + 1 * 3 + 1) =  cosLon;
    *(R_NinE + 1 * 3 + 2) = -cosLat * sinLon;

    *(R_NinE + 2 * 3 + 0) =  cosLat;
    *(R_NinE + 2 * 3 + 1) =  0.0;
    *(R_NinE + 2 * 3 + 2) = -sinLat;

    return 1;
}


///** ***************************************************************************
//* @name LLA2Base Express LLA in a local NED Base Frame
//* @details Pre calculated all non-changing constants and unfolded the matrices
//* @param [in]  LLA - array with the current Latitude, Longitude and Altitude [rad]
//* @param [in]  BaseECEF - start of frame position
//* @param [in]  Rne - rotation matrix from ECEF to NED
//* @param [out] NED - output of the position in North East and Down coords
//* @param [out] newECEF - current position in ECEF from LLA
//* @retval always 1
//******************************************************************************/
BOOL LLA_To_Base( double* llaRad,  // in
                  double* rECEF_Init,  // in
                  real* dr_N,  //NED,
                  real* R_NinE,
                  double* rECEF)  // out
{
    real dr_E[NUM_AXIS];

    double N;
    double sinLat = sin(llaRad[LAT]);
    double cosLat = cos(llaRad[LAT]);
    double sinLon = sin(llaRad[LON]);
    double cosLon = cos(llaRad[LON]);

    real sinLat_r = (real)sinLat;
    real cosLat_r = (real)cosLat;
    real sinLon_r = (real)sinLon;
    real cosLon_r = (real)cosLon;

    N = E_MAJOR / sqrt(1.0 - (E_ECC_SQ * sinLat * sinLat)); // radius of Curvature [meters]

    //LLA_To_ECEF(llaRad, rECEF);
    double temp_d = (N + llaRad[ALT]) * cosLat;
    rECEF[X_AXIS] = temp_d * cosLon;
    rECEF[Y_AXIS] = temp_d * sinLon;
    rECEF[Z_AXIS] = ((E_MINOR_OVER_MAJOR_SQ * N) + llaRad[ALT]) * sinLat;

    dr_E[X_AXIS] = (real)( rECEF[X_AXIS] - *(rECEF_Init + X_AXIS) );
    dr_E[Y_AXIS] = (real)( rECEF[Y_AXIS] - *(rECEF_Init + Y_AXIS) );
    dr_E[Z_AXIS] = (real)( rECEF[Z_AXIS] - *(rECEF_Init + Z_AXIS) );

    // Form R_NinE
    // First row
    *(R_NinE + 0 * 3 + 0) = -sinLat_r * cosLon_r;
    *(R_NinE + 0 * 3 + 1) = -sinLon_r;
    *(R_NinE + 0 * 3 + 2) = -cosLat_r * cosLon_r;

    // Second row
    *(R_NinE + 1 * 3 + 0) = -sinLat_r * sinLon_r;
    *(R_NinE + 1 * 3 + 1) =  cosLon_r;
    *(R_NinE + 1 * 3 + 2) = -cosLat_r * sinLon_r;

    // Third row
    *(R_NinE + 2 * 3 + 0) =  cosLat_r;
    *(R_NinE + 2 * 3 + 1) =       0.0;
    *(R_NinE + 2 * 3 + 2) = -sinLat_r;

    // Convert from delta-position in the ECEF-frame to the NED-frame (the transpose
    //   in the equations that followed is handled in the formulation)
    //
    //       N E          ( E N )T
    // dr_N = R  * dr_E = (  R  )  * dr_E
    //                    (     )
    dr_N[X_AXIS] = *(R_NinE + X_AXIS * 3 + X_AXIS) * dr_E[X_AXIS] +
                   *(R_NinE + Y_AXIS * 3 + X_AXIS) * dr_E[Y_AXIS] +
                   *(R_NinE + Z_AXIS * 3 + X_AXIS) * dr_E[Z_AXIS];
    dr_N[Z_AXIS] = *(R_NinE + X_AXIS * 3 + Z_AXIS) * dr_E[X_AXIS] +
                   *(R_NinE + Y_AXIS * 3 + Z_AXIS) * dr_E[Y_AXIS] +
                   *(R_NinE + Z_AXIS * 3 + Z_AXIS) * dr_E[Z_AXIS];
    dr_N[Y_AXIS] = *(R_NinE + X_AXIS * 3 + Y_AXIS) * dr_E[X_AXIS] +
                   *(R_NinE + Y_AXIS * 3 + Y_AXIS) * dr_E[Y_AXIS] +
                   *(R_NinE + Z_AXIS * 3 + Y_AXIS) * dr_E[Z_AXIS];

    return 1;
}


/** ***************************************************************************
* @name LLA2ECEF Lat [rad], Lon [rad] to Earth Centered Earth Fixed coordinates
*        [m]
* @details pre calculated all non-changing constants
* @param [in] LLA - array [rad] with the Latitude, Lonigtude and altitude
* @param [out] ECEF - array cartresian coordinate [m]
* @retval always 1
******************************************************************************/
BOOL LLA_To_ECEF( double* lla_Rad,
                  double* ecef_m )
{
    double N;
    double cosLat = cos( lla_Rad[LAT] );
    double sinLat = sin( lla_Rad[LAT] );
    double cosLon = cos( lla_Rad[LON] );
    double sinLon = sin( lla_Rad[LON] );

    N = E_MAJOR / sqrt(1.0 - (E_ECC_SQ * sinLat * sinLat)); // radius of Curvature [meters]

    double temp = (N + lla_Rad[ALT]) * cosLat;
    ecef_m[X_AXIS] = temp * cosLon;
    ecef_m[Y_AXIS] = temp * sinLon;
    ecef_m[Z_AXIS] = ((E_MINOR_OVER_MAJOR_SQ * N) + lla_Rad[ALT]) * sinLat;

    return 1;
}


/** ***************************************************************************
* @name Base2ECEF Express tangent (NED) coords in ECEF coordinates
* @details adds delta postion fro start of frame in NED to the ECEF postion
* at the current time step and returns the results in meters ECEF.
* @param [in] NED - input tangent position [m] in North East and Down coords
* @param {in} BaseECEF - start of frame (gps) position [m]
* @param {in} invRne - inverse of the rotation matrix from ECEF to NED
* @param [out] ECEF - current frame position [m]
* @retval always 1
******************************************************************************/
BOOL PosNED_To_PosECEF( real*  r_N,
                        double* rECEF_Init, //BaseECEF,
                        real* R_NinE,
                        double* rECEF)
{
    // ECEF = Base + delta
    *(rECEF + 0) = *(rECEF_Init + 0) + (double)(*(R_NinE + 0 * 3 + 0) * r_N[X_AXIS] +
                                                *(R_NinE + 0 * 3 + 1) * r_N[Y_AXIS] +
                                                *(R_NinE + 0 * 3 + 2) * r_N[Z_AXIS] ); // X
    *(rECEF + 1) = *(rECEF_Init + 1) + (double)(*(R_NinE + 1 * 3 + 0) * r_N[X_AXIS] +
                                                *(R_NinE + 1 * 3 + 1) * r_N[Y_AXIS] +
                                                *(R_NinE + 1 * 3 + 2) * r_N[Z_AXIS] ); // Y
    *(rECEF + 2) = *(rECEF_Init + 2) + (double)(*(R_NinE + 2 * 3 + 0) * r_N[X_AXIS] +
                                                *(R_NinE + 2 * 3 + 1) * r_N[Y_AXIS] +
                                                *(R_NinE + 2 * 3 + 2) * r_N[Z_AXIS] ); // Z

    return 1;
}


/** ***************************************************************************
* @name ECEF2LLA Earth Centered Earth Fixed [m] to Lat [rad], Lon [rad] coordinates
* @details pre calculated all non-changing constants
* @param [out] LLA - array  the Latitude, Lonigtude [deg] and Altitude [m]
* @param [in] ECEF - cartresian coordiante [m]
* @retval always 1
******************************************************************************/
BOOL ECEF_To_LLA(double* llaDeg, double* ecef_m)
{
    double P;
    double theta;
    double sinLat;
    double sinTheta, cosTheta;
    double Lat;

    P = sqrt( ecef_m[X_AXIS] * ecef_m[X_AXIS] + ecef_m[Y_AXIS] * ecef_m[Y_AXIS] );

    //    theta = atan( ( ecef_m[Z_AXIS] * E_MAJOR ) / ( P * E_MINOR ) );   // sqrt( ecef(2) * const )
    theta = atan2(ecef_m[Z_AXIS] * E_MAJOR_OVER_MINOR, P);

    sinTheta = sin(theta);
    cosTheta = cos(theta);

    Lat = atan2((ecef_m[Z_AXIS] + EP_SQ * sinTheta * sinTheta * sinTheta), (P - E_ECC_SQxE_MAJOR * cosTheta * cosTheta * cosTheta));
    *(llaDeg + LAT) = Lat * R2D;
    *(llaDeg + LON) = atan2(ecef_m[Y_AXIS], ecef_m[X_AXIS]) * R2D; // arctan(Y/X)

    sinLat = sin(Lat);
    *(llaDeg + ALT) = P / cos(Lat) - E_MAJOR / sqrt(1.0 - E_ECC_SQ * sinLat * sinLat); // alt

    return 1;
}


/** ***************************************************************************
* @name ECEF2NED Earth Centered Earth Fixed [m] to North [m], East [m] down [m]
* coordinates
* @details Pre calculated all non-changing constants and unfolded the matricies
* @param [in] LLA - Latitude, Longitude and Altitude [rad]
* @param [in] VelECEF - Earth centered earth fixed [m/s]
* @param [out] VelNED - North East Down [m/s]
* @retval always 1
******************************************************************************/
BOOL VelECEF_To_VelNED( double* LLA,
                        real* VelECEF,
                        real* VelNED )
{
    real cosLat, sinLat;
    real cosLon, sinLon;

#ifndef FAST_MATH
    cosLat = (real)(cos( (real)*(LLA + LAT) ));
    sinLat = (real)(sin( (real)*(LLA + LAT) ));
    cosLon = (real)(cos( (real)*(LLA + LON) ));
    sinLon = (real)(sin( (real)*(LLA + LON) ));
#else
    cosLat = (real)(arm_cos_f32( (real)*(LLA + LAT) ));
    sinLat = (real)(arm_sin_f32( (real)*(LLA + LAT) ));
    cosLon = (real)(arm_cos_f32( (real)*(LLA + LON) ));
    sinLon = (real)(arm_sin_f32( (real)*(LLA + LON) ));
#endif

    // North
    *(VelNED+X_AXIS) = -sinLat * cosLon * *(VelECEF + X_AXIS) +
                           -sinLon * sinLat * *(VelECEF + Y_AXIS) +
                                     cosLat * *(VelECEF + Z_AXIS);
    // East
    *(VelNED+Y_AXIS) =          -sinLon * *(VelECEF + X_AXIS) +
                                     cosLon * *(VelECEF + Y_AXIS);
    // Down
    *(VelNED+Z_AXIS) = -cosLat * cosLon * *(VelECEF + X_AXIS) +
                           -cosLat * sinLon * *(VelECEF + Y_AXIS) +
                                    -sinLat * *(VelECEF + Z_AXIS);

    return 1;
}


#include "TimingVars.h"
//
void GPS_PosVel_To_NED(void)
{
    static BOOL oneTimeOnly = TRUE;

    LLA_To_Base(&gAlgorithm.llaRad[0],
        &gAlgorithm.rGPS0_E[0],
        &gAlgorithm.rGPS_N[0],
        &gAlgorithm.R_NinE[0][0],
        &gAlgorithm.rGPS_E[0]);

    // Upon the first entry into INS, save off the base position and reset the
    //   Kalman filter variables.
    if (gAlgorithm.insFirstTime) {
        gAlgorithm.insFirstTime = FALSE;

        // Save off the base ECEF location
        gAlgorithm.rGPS0_E[X_AXIS] = gAlgorithm.rGPS_E[X_AXIS];
        gAlgorithm.rGPS0_E[Y_AXIS] = gAlgorithm.rGPS_E[Y_AXIS];
        gAlgorithm.rGPS0_E[Z_AXIS] = gAlgorithm.rGPS_E[Z_AXIS];

        // Reset the gps position (as position is relative to starting location)
        gAlgorithm.rGPS_N[X_AXIS] = 0.0;
        gAlgorithm.rGPS_N[Y_AXIS] = 0.0;
        gAlgorithm.rGPS_N[Z_AXIS] = 0.0;

        // ResetINS;  % <--need to reset H and R for each case (INS, MagOnly, ...)

        // Reset prediction values
        gKalmanFilter.Position_N[LAT] = 0.0;
        gKalmanFilter.Position_N[LON] = 0.0;
        gKalmanFilter.Position_N[ALT] = 0.0;

        // Should use the input structure (JSM)
        gKalmanFilter.Velocity_N[X_AXIS] = (real)gGpsDataPtr->vNed[X_AXIS];
        gKalmanFilter.Velocity_N[Y_AXIS] = (real)gGpsDataPtr->vNed[Y_AXIS];
        gKalmanFilter.Velocity_N[Z_AXIS] = (real)gGpsDataPtr->vNed[Z_AXIS];

        if (oneTimeOnly) {
            oneTimeOnly = FALSE;
            gAlgorithm.applyDeclFlag = TRUE;

#ifdef DISPLAY_DIAGNOSTIC_MSG
            TimingVars_DiagnosticMsg("GPS_PosVel_To_NED: First time GPS");
#endif

            // add declination to prediction;

            // Adjust prediction when GPS goes healthy and WMM obtains the first
            //   solution.  rotate the quat by the declination at the current
            // position.
            real qDecl[4];
#ifndef FAST_MATH
            qDecl[Q0] = cos((real)0.5 * gWorldMagModel.decl_rad);
            qDecl[Q1] = (real)0.0;
            qDecl[Q2] = (real)0.0;
            qDecl[Q3] = sin((real)0.5 * gWorldMagModel.decl_rad);
#else
            qDecl[Q0] = arm_cos_f32((real)0.5 * gWorldMagModel.decl_rad);
            qDecl[Q1] = (real)0.0;
            qDecl[Q2] = (real)0.0;
            qDecl[Q3] = arm_sin_f32((real)0.5 * gWorldMagModel.decl_rad);
#endif

            real Q[4][4] = { { qDecl[Q0], -qDecl[Q1], -qDecl[Q2], -qDecl[Q3] },
            { qDecl[Q1],  qDecl[Q0], -qDecl[Q3],  qDecl[Q2] },
            { qDecl[Q2],  qDecl[Q3],  qDecl[Q0], -qDecl[Q1] },
            { qDecl[Q3], -qDecl[Q2],  qDecl[Q1],  qDecl[Q0] } };

            real q[4];
            AxV(&Q[0][0], &gKalmanFilter.quaternion[0], 4, 4, &q[0]);

            //
            gKalmanFilter.quaternion[Q0] = q[Q0];
            gKalmanFilter.quaternion[Q1] = q[Q1];
            gKalmanFilter.quaternion[Q2] = q[Q2];
            gKalmanFilter.quaternion[Q3] = q[Q3];

            AxV(&Q[0][0], &gKalmanFilter.quaternion_Past[0], 4, 4, &q[0]);
            gKalmanFilter.quaternion_Past[Q0] = q[Q0];
            gKalmanFilter.quaternion_Past[Q1] = q[Q1];
            gKalmanFilter.quaternion_Past[Q2] = q[Q2];
            gKalmanFilter.quaternion_Past[Q3] = q[Q3];
        }
    }
}


