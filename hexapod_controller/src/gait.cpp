
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
    cycle_period_ = 1;
    is_travelling_ = false;
    in_cycle_ = false;
    extra_gait_cycle_ = 1;
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    gait_factor = 1.0;
    gait_select(0);
    cycle_leg_number_ = {1,0,1,0,1,0};
    if(GAIT_STYLE == "RIPPLE")
    {
      gait_select(1);
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
        {
    
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
        short int LegStep = cycle_period_ - cycle_leg_number_[leg_index];
        // // Lifts the leg and move it forward
        // if( cycle_leg_number_[leg_index] == 0 && is_travelling_ == true )
        // {
        //     period_distance = cos( cycle_period_ * PI / CYCLE_LENGTH );
        //     feet->foot[leg_index].position.x = base.x * period_distance;
        //     feet->foot[leg_index].position.y = base.y * period_distance;
        //     feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
        //     feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
        // }
        if ((is_travelling_ && (NrLiftedPos & 1) &&
             LegStep == 0) ||
            (!is_travelling_ && LegStep == 0 && ((abs(foot[leg_index].position.x) > 2) ||
                                                 //Up
                                                 (abs(foot[leg_index].position.y) > 2) || (abs(foot[leg_index].orientation.yaw) > 2))))
        {
            foot[leg_index].position.x = 0;
            foot[leg_index].position.z = -LEG_LIFT_HEIGHT * period_height;
            foot[leg_index].position.y = 0;
            foot[leg_index].orientation.yaw = 0;
            GaitLegInAir[GaitCurrentLegNr] = true;
        }
        //Optional Half heigth Rear (2, 3, 5 lifted positions)
        else if (((NrLiftedPos == 2 && LegStep == 0) || (NrLiftedPos >= 3 &&
                                                         (LegStep == -1 || LegStep == (CYCLE_LENGTH - 1)))) &&
                 is_travelling_)
        {
            foot[leg_index].position.x = -base.x * period_distance / LiftDivFactor;
            //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            foot[leg_index].position.z = -3 * LEG_LIFT_HEIGHT * period_height / (3 + HalfLiftHeigth);
            foot[leg_index].position.y = -base.y * period_distance / LiftDivFactor;
            foot[leg_index].orientation.yaw  = -base.theta * period_distance / LiftDivFactor;
            GaitLegInAir[GaitCurrentLegNr] = true;
        }
        // _A_
        // Optional Half heigth front (2, 3, 5 lifted positions)
        else if ((NrLiftedPos >= 2) && (LegStep == 1 || LegStep == -(CYCLE_LENGTH - 1)) && is_travelling_)
        {
            foot[leg_index].position.x = base.x * period_distance / LiftDivFactor;
            // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            foot[leg_index].position.z = -3 * LEG_LIFT_HEIGHT * period_height / (3 + HalfLiftHeigth);
            foot[leg_index].position.y = base.y * period_distance / LiftDivFactor;
            foot[leg_index].orientation.yaw  = base.theta * period_distance / LiftDivFactor;
            GaitLegInAir[GaitCurrentLegNr] = true;
        }

        //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
        else if (((NrLiftedPos == 5 && (LegStep == -2))) && is_travelling_)
        {
            foot[leg_index].position.x = -base.x * period_distance / 2;
            foot[leg_index].position.z = -LEG_LIFT_HEIGHT * period_height / 2;
            foot[leg_index].position.y = -base.y * period_distance / 2;
            foot[leg_index].orientation.yaw  = -base.theta * period_distance / 2;
            GaitLegInAir[GaitCurrentLegNr] = true;
        }

        //Optional Half heigth Front 5 LiftedPos (5 lifted positions)
        else if ((NrLiftedPos == 5) && (LegStep == 2 || LegStep == -(CYCLE_LENGTH - 2)) && is_travelling_)
        {
            foot[leg_index].position.x = base.x * period_distance / 2;
            foot[leg_index].position.z = -LEG_LIFT_HEIGHT * period_height / 2;
            foot[leg_index].position.y = base.y * period_distance / 2;
            foot[leg_index].orientation.yaw  = base.theta * period_distance / 2;
            GaitLegInAir[GaitCurrentLegNr] = true;
        }
        //_B_
        //Leg front down position //bug here?  From _A_ to _B_ there should only be one gaitstep, not 2!
        //For example, where is the case of LegStep==0+2 executed when NRLiftedPos=3?
        else if ((LegStep == FrontDownPos || LegStep == -(CYCLE_LENGTH - FrontDownPos)) && foot[leg_index].position.z < 0)
        {
            foot[leg_index].position.x = base.x * period_distance / 2;
            foot[leg_index].position.y = base.y * period_distance / 2;
            foot[leg_index].orientation.yaw  = base.theta * period_distance / 2;
            foot[leg_index].position.z = 0;
            GaitLegInAir[GaitCurrentLegNr] = true;

            // Moves legs backward pushing the body forward
            if (cycle_leg_number_[leg_index] == 1)
            {
                period_distance = cos(cycle_period_ * PI * gait_factor / CYCLE_LENGTH);
                feet->foot[leg_index].position.x = -base.x * period_distance;
                feet->foot[leg_index].position.y = -base.y * period_distance;
                feet->foot[leg_index].position.z = 0;
                feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
            }
            if (cycle_leg_number_[leg_index] == 2)
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

//=============================================================================
// Gait Select
//=============================================================================

void Gait::gait_select(void)
{
     switch (gait_type_)
    {
        case 0:
            //Ripple Gait 12 steps
            cycle_leg_number_ = {7,11,3,1,5,9};//['RR', 'RM', 'RF', 'LR', 'LM', 'LF']
            NrLiftedPos = 3;
            FrontDownPos = 2;
            LiftDivFactor = 2;
            HalfLiftHeigth = 3;
            TLDivFactor = 8;
            CYCLE_LENGTH = 12;  // ex CYCLE_LENGTH
            NomGaitSpeed = DEFAULT_SLOW_GAIT;
            break;
        case 1:
            //Tripod 8 steps
            cycle_leg_number_ = {1,5,1,5,1,5};//['RR', 'RM', 'RF', 'LR', 'LM', 'LF']
            NrLiftedPos = 3;
            FrontDownPos = 2;
            LiftDivFactor = 2;
            HalfLiftHeigth = 3;
            TLDivFactor = 4;
            CYCLE_LENGTH = 8;
            NomGaitSpeed = DEFAULT_SLOW_GAIT;
            break;
        case 2:
            //Triple Tripod 12 step
            cycle_leg_number_ = {5,10,3,11,4,9};//['RR', 'RM', 'RF', 'LR', 'LM', 'LF']
            NrLiftedPos = 3;
            FrontDownPos = 2;
            LiftDivFactor = 2;
            HalfLiftHeigth = 3;
            TLDivFactor = 8;
            CYCLE_LENGTH = 12;
            NomGaitSpeed = DEFAULT_GAIT_SPEED;
            break;
        // case 3:
        //     // Triple Tripod 16 steps, use 5 lifted positions

        //     GaitLegNr[cRF] = 4;
        //     GaitLegNr[cLM] = 5;
        //     GaitLegNr[cRR] = 6;
        //     GaitLegNr[cLF] = 12;
        //     GaitLegNr[cRM] = 13;
        //     GaitLegNr[cLR] = 14;

        //     NrLiftedPos = 5;
        //     FrontDownPos = 3;
        //     LiftDivFactor = 4;
        //     HalfLiftHeigth = 1;
        //     TLDivFactor = 10;
        //     CYCLE_LENGTH = 16;
        //     NomGaitSpeed = DEFAULT_GAIT_SPEED;
        //     break;
        // case 4:
        //     //Wave 24 steps
        //     GaitLegNr[cLR] = 1;
        //     GaitLegNr[cRF] = 21;
        //     GaitLegNr[cLM] = 5;

        //     GaitLegNr[cRR] = 13;
        //     GaitLegNr[cLF] = 9;
        //     GaitLegNr[cRM] = 17;

        //     NrLiftedPos = 3;
        //     FrontDownPos = 2;
        //     LiftDivFactor = 2;
        //     HalfLiftHeigth = 3;
        //     TLDivFactor = 20;
        //     CYCLE_LENGTH = 24;
        //     NomGaitSpeed = DEFAULT_SLOW_GAIT;
        //     break;
        // case 5:
        //     //Tripod 6 steps
        //     GaitLegNr[cLR] = 4;
        //     GaitLegNr[cRF] = 1;
        //     GaitLegNr[cLM] = 1;

        //     GaitLegNr[cRR] = 1;
        //     GaitLegNr[cLF] = 4;
        //     GaitLegNr[cRM] = 4;

        //     NrLiftedPos = 2;
        //     FrontDownPos = 1;
        //     LiftDivFactor = 2;
        //     HalfLiftHeigth = 1;
        //     TLDivFactor = 4;
        //     CYCLE_LENGTH = 6;
        //     NomGaitSpeed = DEFAULT_GAIT_SPEED;
        //     break;
    }
}