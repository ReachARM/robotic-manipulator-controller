/**
 * \file        Controller.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "Controller.h"

arm_controller::Controller::Controller()
:base_left(std::make_unique<Motor>(static_cast<__uint8_t >(BASE_LEFT_ID),-1,-1,1,80,77)),
 base_right(std::make_unique<Motor>(static_cast<__uint8_t >(BASE_RIGHT_ID),-1,-1,1,80,77)),
 shoulder_left(std::make_unique<Motor>(static_cast<__uint8_t>(SHOULDER_LEFT_ID),181,783,1,80,177)),
 shoulder_right(std::make_unique<Motor>(static_cast<__uint8_t>(SHOULDER_RIGHT_ID),181,783,1,80,177)),
 elbow_left(std::make_unique<Motor>(static_cast<__uint8_t>(ELBOW_LEFT_ID),181,783,1,80,177)),
 elbow_right(std::make_unique<Motor>(static_cast<__uint8_t>(ELBOW_RIGHT_ID),181,783,1,80,177)),
 wrist(std::make_unique<Motor>(static_cast<__uint8_t>(WRIST_ID),0,0)),
 base_joint(std::vector<MOTOR_ID>()),
 shoulder_joint(std::vector<MOTOR_ID>()),
 elbow_joint(std::vector<MOTOR_ID>()),
 currentState(NO_ERROR),
 BASE_START_POSITION(0),
 SHOULDER_START_POSITION(0),
 ELBOW_START_POSITION(0),
 WRIST_START_POSITION(0)
{
    if( dxl_initialize(USB2DYNAMIXEL_ID,1) != 0 ){
        base_left->initializeMotor();
        base_right->initializeMotor();
        shoulder_left->initializeMotor();
        shoulder_right->initializeMotor();
        elbow_left->initializeMotor();
        elbow_right->initializeMotor();
        wrist->initializeMotor();
    }

    // The base
    if( base_left->isOpened() && base_right->isOpened()) {
        base_joint.push_back(MOTOR_ID(BASE_LEFT_ID));
        base_joint.push_back(MOTOR_ID(BASE_RIGHT_ID));
        moveBase(BASE_START_POSITION);
    }
    else
        currentState = BASE_ERROR;

    // The shoulder
    if( shoulder_left->isOpened() && shoulder_right->isOpened()) {
        shoulder_joint.push_back(MOTOR_ID(SHOULDER_LEFT_ID));
        shoulder_joint.push_back(MOTOR_ID(SHOULDER_RIGHT_ID));
        moveShoulder(SHOULDER_START_POSITION);
    }
    else
        currentState = SHOULDER_ERROR;

    // The elbow
    if( elbow_left->isOpened() && elbow_right->isOpened()) {
        elbow_joint.push_back(MOTOR_ID(ELBOW_LEFT_ID));
        elbow_joint.push_back(MOTOR_ID(ELBOW_RIGHT_ID));
        moveElbow(ELBOW_START_POSITION);
    }
    else
        currentState = ELBOW_ERROR;

    if( wrist->isOpened())
        wrist->setCurrentAngle(WRIST_START_POSITION);
    else
        currentState = WRIST_ERROR;

}

arm_controller::Controller::~Controller() {
    try {
        dxl_terminate();
    } catch (...) {
        ROS_ERROR("An exception occured while trying to close the Dynamixel driver.");
    }
}


bool arm_controller::Controller::moveAbsoluteMotor(const Controller::MOTOR_ID& motor, const float targetAngle) {
    ROS_INFO("Moving absolute motor");
    if(motor.id<BASE_RIGHT_ID || motor.id>WRIST_ID ) {
        ROS_WARN("Motor ID is no within boundaries");
        return false;
    }
    if(getMotor(motor)->isOpened())
        getMotor(motor)->setCurrentAngle(targetAngle);
    else {
        ROS_WARN("Cannot move target motor : The motor is closed.");
        return false;
    }
    return true;
}

bool arm_controller::Controller::moveIncrementMotor(const Controller::MOTOR_ID& motor, const float increment){
    ROS_INFO("Moving increment motor");
    if(motor.id<BASE_RIGHT_ID || motor.id>WRIST_ID ) {
        ROS_WARN("Motor ID is no within boundaries");
        return false;
    }
    if(getMotor(motor)->isOpened())
        getMotor(motor)->setIncrementAngle(increment);
    else {
        ROS_WARN("Cannot move target motor : The motor is closed.");
        return false;
    }
    return true;
}


bool arm_controller::Controller::moveRelativeTool(const float tX, const float tY, const float tZ,
                                  const float roll, const float pitch, const float yaw) {
    if( currentState == NO_ERROR ){
        // move the robot
    } else {
        ROS_WARN("Cannot move tool, error code : %d", currentState);
    }
    return false;
}


arm_controller::Motor* arm_controller::Controller::getMotor(const Controller::MOTOR_ID& motor)const{
    Motor* motorPtr;
    switch(motor.id){
        case (BASE_LEFT_ID):{
            motorPtr = base_left.get();
            break;
        }
        case (BASE_RIGHT_ID):{
            motorPtr = base_right.get();
            break;
        }
        case (SHOULDER_LEFT_ID):{
            motorPtr = shoulder_left.get();
            break;
        }
        case (SHOULDER_RIGHT_ID):{
            motorPtr = shoulder_right.get();
            break;
        }
        case (ELBOW_LEFT_ID):{
            motorPtr = elbow_left.get();
            break;
        }
        case (ELBOW_RIGHT_ID):{
            motorPtr = elbow_right.get();
            break;
        }
        case (WRIST_ID):{
            motorPtr = wrist.get();
            break;
        }
    }
    return motorPtr;
}

bool arm_controller::Controller::moveSyncMotor(const std::vector<MOTOR_ID>& motors, const float angle) {

    ROS_INFO("Moving motors synchro");
    auto NUM_ACTUATOR = motors.size();

    auto id = std::vector<int>();

    auto i = 0;
    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        id.push_back(motors[i].id);
    }

    ROS_INFO("Before packet");
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, Motor::GOAL_POSITION_L);
    dxl_set_txpacket_parameter(1, 2);


    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        ROS_INFO("Adding motor %d", i );
        dxl_set_txpacket_parameter(2+3*i, id[i]);
        auto goalPos = static_cast<int>(getMotor(MOTOR_ID(id[i]))->getRelativeMotorAngle(angle) / Motor::STEP_PRECISION);
        dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(goalPos));
        dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(goalPos));
    }
    dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);

    ROS_INFO("Before sending packet");
    dxl_txrx_packet();
}
