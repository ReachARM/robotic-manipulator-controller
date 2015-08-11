/**
 * \file        Controller.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "Controller.h"

Controller::Controller()
:base(std::make_unique<Motor>(static_cast<__uint8_t >(BASE_ID),-1,-1,1,80,77)),
 shoulder(std::make_unique<Motor>(static_cast<__uint8_t>(SHOULDER_ID),181,783,1,80,177)),
 elbow(std::make_unique<Motor>(static_cast<__uint8_t>(ELBOW_ID),181,783,1,80,177)),
 wrist(std::make_unique<Motor>(static_cast<__uint8_t>(WRIST_ID),0,0)),
 currentState(NO_ERROR),
 BASE_START_POSITION(0),
 SHOULDER_START_POSITION(0),
 ELBOW_START_POSITION(0),
 WRIST_START_POSITION(0)
{
    if( dxl_initialize(USB2DYNAMIXEL_ID,1) != 0 ){
        base->initializeMotor();
        shoulder->initializeMotor();
        elbow->initializeMotor();
        wrist->initializeMotor();
    }

    if( base->isOpened())
        base->setCurrentAngle(BASE_START_POSITION);
    else
        currentState = BASE_ERROR;

    if( shoulder->isOpened())
        shoulder->setCurrentAngle(SHOULDER_START_POSITION);
    else
        currentState = SHOULDER_ERROR;

    if( elbow->isOpened())
        elbow->setCurrentAngle(ELBOW_START_POSITION);
    else
        currentState = ELBOW_ERROR;

    if( wrist->isOpened())
        wrist->setCurrentAngle(WRIST_START_POSITION);
    else
        currentState = WRIST_ERROR;

}

Controller::~Controller() {
    try {
        dxl_terminate();
    } catch (...) {
        ROS_ERROR("An exception occured while trying to close the Dynamixel driver.");
    }
}


bool Controller::moveAbsoluteMotor(const Controller::MOTOR_ID& motor, const float targetAngle) {
    ROS_INFO("Moving absolute motor");
    if(motor.id<BASE_ID || motor.id>WRIST_ID ) {
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

bool Controller::moveIncrementMotor(const Controller::MOTOR_ID& motor, const float increment){
    ROS_INFO("Moving increment motor");
    if(motor.id<BASE_ID || motor.id>WRIST_ID ) {
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

// TODO Comment this section
bool Controller::moveRelativeTool(const float tX, const float tY, const float tZ,
                                  const float roll, const float pitch, const float yaw) {
    if( currentState == NO_ERROR ){
        // move the robot
    } else {
        ROS_WARN("Cannot move tool, error code : %d", currentState);
    }
    return false;
}


Motor* Controller::getMotor(const Controller::MOTOR_ID& motor)const{
    Motor* motorPtr;
    switch(motor.id){
        case (BASE_ID):{
            motorPtr = base.get();
            break;
        }
        case (SHOULDER_ID):{
            motorPtr = shoulder.get();
            break;
        }
        case (ELBOW_ID):{
            motorPtr = elbow.get();
            break;
        }
        case (WRIST_ID):{
            motorPtr = wrist.get();
            break;
        }
    }
    return motorPtr;
}

bool Controller::moveSyncMotor(const std::vector<MOTOR_ID>& motors, const float angle) {

    ROS_INFO("Moving motors synchro");
    auto NUM_ACTUATOR = motors.size();

    auto id = std::vector<int>(), phase = std::vector<int>();

    auto i = 0;
    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        id.push_back(motors[i].id);
        //phase.push_back()phase[i] = 2* PI * (float)i / (float)NUM_ACTUATOR;
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

bool Controller::moveBase(const float angle) {
    auto motors = std::vector<MOTOR_ID>();
    motors.push_back(MOTOR_ID(BASE_ID));
    motors.push_back(MOTOR_ID(SHOULDER_ID));
    moveSyncMotor(motors,angle);
}