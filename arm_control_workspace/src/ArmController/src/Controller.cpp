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