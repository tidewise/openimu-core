/*
* File:   xbowsp_algorithm.c
* Author: t. malerich
*
* Created on 2016-06-06
*
* Purpose is to provide the functions to initialize the AlgorithmStruct
*/

//#include "xbowsp_algorithm.h"    // gAlgorithm
#include "GlobalConstants.h"     // TRUE, FALSE, etc
#include "scaling.h"      // DEGREES_TO_RADS

#include "AlgorithmLimits.h"
#include "TimingVars.h"

#include <math.h>
#include "platformAPI.h"

#ifdef INS_OFFLINE
#include "C:\Projects\software\sim\INS380_Offline\INS380_Offline\SimulationParameters.h"
#endif
/*
void    SetAlgorithmUseDgps(BOOL d) 
{
     gAlgorithm.bitStatus.hwStatus.bit.noDGPS = d; 
}




void setAlgorithmStateStabilizeSystem()
{
    gAlgorithm.state = STABILIZE_SYSTEM;

}

uint16_t getAlgorithmCounter(void)
{
    return gAlgorithm.counter;
}
*/
