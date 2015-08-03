/**
 * \file        Motor.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        02/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "Motor.h"

#include <iostream>

// CTOR
Motor::Motor(const __uint8_t id, const float lowAngleLimit, const float highAngleLimit, const int defaultBaurate, const int speedRpmLimit)
:motor_id(id),
 lowStepLimit(lowAngleLimit),
 highStepLimit(highAngleLimit),
 speedRpmLimit(speedRpmLimit),
 defaultBaudrate(defaultBaurate),
 opened(false)
{}

//DTOR

Motor::~Motor() {
}

void Motor::initializeMotor(){
    // Check if current angle is within limits
    auto currentAngle = getCurrentAngle();
    if(currentAngle!=-1) {
        ROS_INFO("Motor:%d at current Angle: %f", motor_id, currentAngle);
        if( currentAngle < (lowStepLimit*STEP_PRECISION-BONDING_BOX_SIZE_FOR_INITIALIZATION) || currentAngle > (highStepLimit*STEP_PRECISION+BONDING_BOX_SIZE_FOR_INITIALIZATION) ) {
            ROS_INFO("Not within boundaries L:%f  H%f",(lowStepLimit*STEP_PRECISION),(highStepLimit*STEP_PRECISION));
            opened = false;
        } else {
            ROS_INFO("Motor:%d is opened", motor_id);
            opened = true;
        }
    }

    if( opened ) {

        ROS_INFO("Settings Motor:%d limits", motor_id);

        // Clockwise angle limit
        dxl_write_word(motor_id, CW_ANGLE_LIMIT_L, static_cast<int>(lowStepLimit));

        // Counter-Clockwise angle limit
        dxl_write_word(motor_id, CCW_ANGLE_LIMIT_L, static_cast<int>(highStepLimit));

        // Speed
        dxl_write_word( motor_id, MOVING_SPEED_L, speedRpmLimit);
    }
}

void Motor::PrintCommStatus( const int CommStatus) const
{
    switch(CommStatus)
    {
        case COMM_TXFAIL:
            printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
            break;

        case COMM_TXERROR:
            printf("COMM_TXERROR: Incorrect instruction packet!\n");
            break;

        case COMM_RXFAIL:
            printf("COMM_RXFAIL: Failed get status packet from device!\n");
            break;

        case COMM_RXWAITING:
            printf("COMM_RXWAITING: Now recieving status packet!\n");
            break;

        case COMM_RXTIMEOUT:
            printf("COMM_RXTIMEOUT: There is no status packet!\n");
            break;

        case COMM_RXCORRUPT:
            printf("COMM_RXCORRUPT: Incorrect status packet!\n");
            break;

        default:
            printf("This is unknown error code!\n");
            break;
    }
}

// Print error bit of status packet
void Motor::PrintErrorCode() const
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        printf("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        printf("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        printf("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        printf("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        printf("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        printf("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        printf("Instruction code error!\n");
}