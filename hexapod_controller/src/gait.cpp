
// ROS Hexapod Controller Node
// Copyright (c) 2016, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Kevin Ochs nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL KEVIN OCHS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs

// xxxxx Phoenix xxxxxx


#define DEFINE_HEX_GLOBALS

#include "Hex_Cfg.h"
#include "Phoenix.h"

#define BalanceDivFactor CNT_LEGS

static const byte GetACos[] =
{
    255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225,
    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193,
    192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,
    157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117,
    115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70,
    70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
    59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
    46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
    28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
    16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0
};

//Sin table 90 deg, persision 0.5 deg [180 values]
static const word GetSin[] =
{
    0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564,
    1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007,
    3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383,
    4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
    5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
    6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826,
    7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660,
    8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304,
    9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743,
    9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969,
  9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000
};

//Build tables for Leg configuration like I/O and MIN/imax values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

#ifndef QUADMODE
// Standard Hexapod...
// Servo Horn offsets
#ifdef cRRFemurHornOffset1                        // per leg configuration
static const short cFemurHornOffset1[] PROGMEM =
{
    cRRFemurHornOffset1, cRMFemurHornOffset1, cRFFemurHornOffset1, cLRFemurHornOffset1, cLMFemurHornOffset1, cLFFemurHornOffset1
};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else                                             // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif

#ifdef c4DOF
#ifdef cRRTarsHornOffset1                         // per leg configuration
static const short cTarsHornOffset1[] PROGMEM =
{
    cRRTarsHornOffset1,  cRMTarsHornOffset1,  cRFTarsHornOffset1,  cLRTarsHornOffset1,  cLMTarsHornOffset1,  cLFTarsHornOffset1
};
#define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
#else                                             // Fixed per leg, if not defined 0
#ifndef cTarsHornOffset1
#define cTarsHornOffset1  0
#endif
#define CTARSHORNOFFSET1(LEGI)  cTarsHornOffset1
#endif
#endif

//Min / imax values
const short cCoxaMin1[] PROGMEM =
{
    cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1
};
const short cCoxaMax1[] PROGMEM =
{
    cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1
};
const short cFemurMin1[] PROGMEM =
{
    cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1
};
const short cFemurMax1[] PROGMEM =
{
    cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1
};
const short cTibiaMin1[] PROGMEM =
{
    cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1
};
const short cTibiaMax1[] PROGMEM =
{
    cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1
};

#ifdef c4DOF
const short cTarsMin1[] PROGMEM =
{
    cRRTarsMin1, cRMTarsMin1, cRFTarsMin1, cLRTarsMin1, cLMTarsMin1, cLFTarsMin1
};
const short cTarsMax1[] PROGMEM =
{
    cRRTarsMax1, cRMTarsMax1, cRFTarsMax1, cLRTarsMax1, cLMTarsMax1, cLFTarsMax1
};
#endif

//Leg Lengths
const byte cCoxaLength[] PROGMEM =
{
    cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength
};
const byte cFemurLength[] PROGMEM =
{
    cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength
};
const byte cTibiaLength[] PROGMEM =
{
    cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength
};
#ifdef c4DOF
const byte cTarsLength[] PROGMEM =
{
    cRRTarsLength, cRMTarsLength, cRFTarsLength, cLRTarsLength, cLMTarsLength, cLFTarsLength
};
#endif

//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM =
{
    cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX
};
const short cOffsetZ[] PROGMEM =
{
    cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ
};

//Default leg angle
const short cCoxaAngle1[] PROGMEM =
{
    cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1
};

//Start positions for the leg
const short cInitPosX[] PROGMEM =
{
    cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX
};
const short cInitPosY[] PROGMEM =
{
    cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY
};
const short cInitPosZ[] PROGMEM =
{
    cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ
};

//=============================================================================
#else
// Quads...
// Servo Horn offsets
#ifdef cRRFemurHornOffset1                        // per leg configuration
static const short cFemurHornOffset1[] PROGMEM =
{
    cRRFemurHornOffset1, cRFFemurHornOffset1, cLRFemurHornOffset1, cLFFemurHornOffset1
};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else                                             // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif

#ifdef c4DOF
#ifdef cRRTarsHornOffset1                         // per leg configuration
static const short cTarsHornOffset1[] PROGMEM =
{
    cRRTarsHornOffset1, cRFTarsHornOffset1,  cLRTarsHornOffset1, cLFTarsHornOffset1
};
#define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
#else                                             // Fixed per leg, if not defined 0
#ifndef cTarsHornOffset1
#define cTarsHornOffset1  0
#endif
#define CTARSHORNOFFSET1(LEGI)  cTarsHornOffset1
#endif
#endif

//Min / imax values
const short cCoxaMin1[] PROGMEM =
{
    cRRCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLFCoxaMin1
};
const short cCoxaMax1[] PROGMEM =
{
    cRRCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLFCoxaMax1
};
const short cFemurMin1[] PROGMEM =
{
    cRRFemurMin1, cRFFemurMin1, cLRFemurMin1, cLFFemurMin1
};
const short cFemurMax1[] PROGMEM =
{
    cRRFemurMax1, cRFFemurMax1, cLRFemurMax1, cLFFemurMax1
};
const short cTibiaMin1[] PROGMEM =
{
    cRRTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLFTibiaMin1
};
const short cTibiaMax1[] PROGMEM =
{
    cRRTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLFTibiaMax1
};

#ifdef c4DOF
const short cTarsMin1[] PROGMEM =
{
    cRRTarsMin1, cRFTarsMin1, cLRTarsMin1, cLFTarsMin1
};
const short cTarsMax1[] PROGMEM =
{
    cRRTarsMax1, cRFTarsMax1, cLRTarsMax1, cLFTarsMax1
};
#endif

//Leg Lengths
const byte cCoxaLength[] PROGMEM =
{
    cRRCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLFCoxaLength
};
const byte cFemurLength[] PROGMEM =
{
    cRRFemurLength, cRFFemurLength, cLRFemurLength, cLFFemurLength
};
const byte cTibiaLength[] PROGMEM =
{
    cRRTibiaLength, cRFTibiaLength, cLRTibiaLength, cLFTibiaLength
};
#ifdef c4DOF
const byte cTarsLength[] PROGMEM =
{
    cRRTarsLength, cRFTarsLength, cLRTarsLength, cLFTarsLength
};
#endif

//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM =
{
    cRROffsetX, cRFOffsetX, cLROffsetX, cLFOffsetX
};
const short cOffsetZ[] PROGMEM =
{
    cRROffsetZ, cRFOffsetZ, cLROffsetZ, cLFOffsetZ
};

//Default leg angle
const short cCoxaAngle1[] PROGMEM =
{
    cRRCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLFCoxaAngle1
};

//Start positions for the leg
const short cInitPosX[] PROGMEM =
{
    cRRInitPosX, cRFInitPosX, cLRInitPosX, cLFInitPosX
};
const short cInitPosY[] PROGMEM =
{
    cRRInitPosY, cRFInitPosY, cLRInitPosY, cLFInitPosY
};
const short cInitPosZ[] PROGMEM =
{
    cRRInitPosZ, cRFInitPosZ, cLRInitPosZ, cLFInitPosZ
};

boolean g_fQuadDynamicShift;
#endif

// Define some globals for debug information
boolean g_fShowDebugPrompt;
boolean g_fDebugOutput;
boolean g_fEnableServos = true;













// xxxxx Phoenix xxxxxx

#include <gait.h>

static const double PI = atan(1.0)*4.0;

//==============================================================================
//  Constructor: Initialize gait variables
//==============================================================================

Gait::Gait( void )
{
    ros::param::get( "CYCLE_LENGTH", CYCLE_LENGTH );
    ros::param::get( "LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT );
    ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
    ros::param::get( "GAIT_STYLE", GAIT_STYLE);
    cycle_period_ = 25;
    is_travelling_ = false;
    in_cycle_ = false;
    extra_gait_cycle_ = 1;
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    gait_factor = 1.0;
    cycle_leg_number_ = {1,0,1,0,1,0};
    if(GAIT_STYLE == "RIPPLE")
    {
      gait_factor = 0.5;
      cycle_leg_number_ = {1,0,2,0,2,1};
    }
    period_distance = 0;
    period_height = 0;
}

//=============================================================================
// step calculation
//=============================================================================

void Gait::cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet, geometry_msgs::Twist *gait_vel )
{
    period_height = sin( cycle_period_ * PI / CYCLE_LENGTH );

    // Calculate current velocities for this period of the gait
    // This factors in the sinusoid of the step for accurate odometry
    current_time_ = ros::Time::now();
    double dt = ( current_time_ - last_time_ ).toSec();
    gait_vel->linear.x = ( ( PI*base.x ) /  CYCLE_LENGTH ) * period_height * ( 1.0 / dt );
    gait_vel->linear.y = ( ( -PI*base.y ) /  CYCLE_LENGTH ) * period_height * ( 1.0 / dt );
    gait_vel->angular.z = ( ( PI*base.theta ) /  CYCLE_LENGTH ) * period_height * ( 1.0 / dt );
    last_time_ = current_time_;

    for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
    {
        // Lifts the leg and move it forward
        if( cycle_leg_number_[leg_index] == 0 && is_travelling_ == true )
        {
            period_distance = cos( cycle_period_ * PI / CYCLE_LENGTH );
            feet->foot[leg_index].position.x = base.x * period_distance;
            feet->foot[leg_index].position.y = base.y * period_distance;
            feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
            feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
        }
        // Moves legs backward pushing the body forward
        if( cycle_leg_number_[leg_index] == 1 )
        {
            period_distance = cos( cycle_period_ * PI * gait_factor / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
        }
        if( cycle_leg_number_[leg_index] == 2 )
        {
            period_distance = cos((CYCLE_LENGTH + cycle_period_) * PI * gait_factor / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
        }
    }
}

//=============================================================================
// Gait Sequencing
//=============================================================================

void Gait::gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet, geometry_msgs::Twist *gait_vel )
{
    // Convert velocities into actual distance for gait/foot positions
    geometry_msgs::Pose2D base;
    base.x = cmd_vel.linear.x / PI * CYCLE_LENGTH;
    base.y = cmd_vel.linear.y / PI * CYCLE_LENGTH;
    base.theta = cmd_vel.angular.z / PI * CYCLE_LENGTH;

    // Low pass filter on the values to avoid jerky movements due to rapid value changes
    smooth_base_.x = base.x * 0.05 + ( smooth_base_.x * ( 1.0 - 0.05 ) );
    smooth_base_.y = base.y * 0.05 + ( smooth_base_.y * ( 1.0 - 0.05 ) );
    smooth_base_.theta = base.theta * 0.05 + ( smooth_base_.theta * ( 1.0 - 0.05 ) );

    // Check to see if we are actually travelling
    if( ( std::abs( smooth_base_.y ) > 0.001 ) || // 1 mm
        ( std::abs( smooth_base_.x ) > 0.001 ) || // 1 mm
        ( std::abs( smooth_base_.theta ) > 0.00436332313 ) ) // 0.25 degree
    {
        is_travelling_ = true;
    }
    else
    {
        is_travelling_ = false;
        // Check to see if the legs are in a non rest state
        for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
        {
            if( ( std::abs( feet->foot[leg_index].position.x ) > 0.001 ) || // 1 mm
                ( std::abs( feet->foot[leg_index].position.y ) > 0.001 ) || // 1 mm
                ( std::abs( feet->foot[leg_index].orientation.yaw ) > 0.034906585 ) || // 2 degrees
                  std::abs( feet->foot[leg_index].position.z) > 0.001 ) // 1 mm
            {
                // If so calculate the rest of the cycle and add another complete cycle
                // This forces another cycle to allow all legs to set down after travel is stopped
                extra_gait_cycle_ = CYCLE_LENGTH - cycle_period_ + CYCLE_LENGTH;
                break;
            }
            else
            {
                extra_gait_cycle_ = 1;
            }
        }

        // countdown for in_cycle state
        if( extra_gait_cycle_ > 1 )
        {
            extra_gait_cycle_--;
            in_cycle_ = !( extra_gait_cycle_ == 1 );
        }
    }

    // If either is true we consider the gait active
    if( is_travelling_ == true || in_cycle_ == true  )
    {
        cyclePeriod( smooth_base_, feet, gait_vel );
        cycle_period_++;
    }
    else
    {
        // Reset period to start just to be sure. ( It should be here anyway )
        cycle_period_ = 0;
    }

    // Loop cycle and switch the leg groupings for cycle
    if( cycle_period_ == CYCLE_LENGTH )
    {
        cycle_period_ = 0;
        sequence_change( cycle_leg_number_ ); //sequence change
    }
}

//=============================================================================
// Gait Sequence Change
//=============================================================================

void Gait::sequence_change( std::vector<int> &vec )
{
    for( int i = 0 ; i < vec.size(); i++ )
    {
        if( vec[i] == 0 ) vec[i] = 1;
        else if( vec[i] == 1 && GAIT_STYLE == "RIPPLE" ) vec[i] = 2;
        else vec[i] = 0;
    }
}
