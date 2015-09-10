/**
 * \file        AX12A.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        29/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "AX12A.h"

arm_controller::AX12A::AX12A( const int ID, const int defaultBaudrate, const int speedLimit, const int highStepLimit, const int lowStepLimit, const int angleOffset)
: Motor(ID),
  lowAngleLimit(lowStepLimit/STEP_PRECISION),
  highAngleLimit(highStepLimit/STEP_PRECISION),
  speedRpmLimit(speedRpmLimit),
  defaultBaudrate(defaultBaudrate),
  lowStepLimit(lowStepLimit),
  highStepLimit(highStepLimit),
  angleOffset(angleOffset)
{}

arm_controller::AX12A::~AX12A() {

}

void arm_controller::AX12A::initializeMotor() {
// Check if current angle is within limits
    auto motor_id = getCurrentID();
    auto currentAngle = getCurrentAngle();
    if(lowStepLimit != INFINITE_BOUNDARIES && highStepLimit != INFINITE_BOUNDARIES) {
        if (currentAngle != -1) {
            ROS_INFO("Motor:%d at current Angle: %f", motor_id, currentAngle);
            if (currentAngle < (lowStepLimit * STEP_PRECISION - angleOffset - 10) ||
                currentAngle > (highStepLimit * STEP_PRECISION - angleOffset + 10)) { // TODO HARDCODED VALUES
                ROS_INFO("Not within boundaries L:%f  H%f", (lowStepLimit * STEP_PRECISION),
                         (highStepLimit * STEP_PRECISION));
                opened = false;
            } else {
                ROS_INFO("Motor:%d is opened", motor_id);
                opened = true;
            }

            // Clockwise angle limit
            dxl_write_word(motor_id, CW_ANGLE_LIMIT_L, static_cast<int>(lowStepLimit));

            // Counter-Clockwise angle limit
            dxl_write_word(motor_id, CCW_ANGLE_LIMIT_L, static_cast<int>(highStepLimit));

        }
    } else {
        ROS_INFO("Motor:%d is opened", motor_id);
        opened = true;

        // Clockwise angle limit
        dxl_write_word(motor_id, CW_ANGLE_LIMIT_L, MIN_VALUE_FOR_INFINITE_JOINTS);

        // Counter-Clockwise angle limit
        dxl_write_word(motor_id, CCW_ANGLE_LIMIT_L, MAX_VALUE_FOR_INFINITE_JOINTS);

    }
    if( opened ) {

        ROS_INFO("Settings Motor:%d limits", motor_id);

        // Speed
        dxl_write_word( motor_id, MOVING_SPEED_L, speedRpmLimit);
    }
}

void arm_controller::AX12A::printCommStatus(const int status) const {
    switch(status)
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

void arm_controller::AX12A::printErrorCode() const {
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

void arm_controller::AX12A::setCurrentAngle(const float angle) {
    auto motor_id = getCurrentID();
    if( opened ){
        ROS_INFO("Angle : %f", angle);
        if(lowStepLimit != INFINITE_BOUNDARIES && highStepLimit != INFINITE_BOUNDARIES) {
            if ((angleOffset + angle) > (lowStepLimit * STEP_PRECISION) &&
                (angleOffset - angle) < (highStepLimit * STEP_PRECISION)) if ((angle - angleOffset) < 0)
                dxl_write_word(motor_id, GOAL_POSITION_L, static_cast<int>((angleOffset - angle) / STEP_PRECISION));
            else
                dxl_write_word(motor_id, GOAL_POSITION_L, static_cast<int>((angle + angleOffset) / STEP_PRECISION));
            else
                ROS_ERROR("Motor:%d cannot move to angle:%f , limits are %f to %f", motor_id, angle,
                          (lowStepLimit * STEP_PRECISION), (highStepLimit * STEP_PRECISION));
        } else {
            dxl_write_word(motor_id, GOAL_POSITION_L, static_cast<int>((angle + angleOffset) / STEP_PRECISION));
        }
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

void arm_controller::AX12A::moveSyncMotors(const std::vector<int> &motorIDs, const float angle) {
    ROS_INFO("Moving motors synchro");
    auto NUM_ACTUATOR = motorIDs.size();

    auto id = std::vector<int>();

    auto i = 0;
    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        id.push_back(motorIDs[i]);
    }

    ROS_INFO("Before packet");
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, AX12A::GOAL_POSITION_L);
    dxl_set_txpacket_parameter(1, 2);


    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        ROS_INFO("Adding motor %d", i );
        dxl_set_txpacket_parameter(2+3*i, id[i]);
        auto goalPos = static_cast<int>( angle / AX12A::STEP_PRECISION);
        dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(goalPos));
        dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(goalPos));
    }
    dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);

    ROS_INFO("Before sending packet");
    dxl_txrx_packet();
}