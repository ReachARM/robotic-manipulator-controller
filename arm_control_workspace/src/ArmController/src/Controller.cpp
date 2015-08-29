/**
 * \file        Controller.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "Controller.h"

arm_controller::Controller::Controller(const OPERATION_MODE& mode)
:base_joint(std::vector<MOTOR_ID>()),
 shoulder_joint(std::vector<MOTOR_ID>()),
 elbow_joint(std::vector<MOTOR_ID>()),
 currentState(NO_ERROR),
 BASE_START_POSITION(0),
 SHOULDER_START_POSITION(0),
 ELBOW_START_POSITION(0),
 WRIST_START_POSITION(0),
 currentOperationMode(mode)
{
    switch(currentOperationMode) {
        case(AX12A_MODE): {

            base_left = std::unique_ptr<AX12A>(new AX12A((BASE_LEFT_ID),1,80,-1,-1,77));
            base_right = std::unique_ptr<AX12A>(new AX12A((BASE_RIGHT_ID),1,80,-1,-1,77));
            shoulder_left = std::unique_ptr<AX12A>(new AX12A((SHOULDER_LEFT_ID),1,80,783,181,177));
            shoulder_right = std::unique_ptr<AX12A>(new AX12A((SHOULDER_RIGHT_ID),1,80,783,181,177));
            elbow_left = std::unique_ptr<AX12A>(new AX12A((ELBOW_LEFT_ID),1,80,783,181,177));
            elbow_right = std::unique_ptr<AX12A>(new AX12A((ELBOW_RIGHT_ID),1,80,783,181,177));
            wrist = std::unique_ptr<AX12A>(new AX12A((WRIST_ID),0,0,0,0,0));

            if (dxl_initialize(USB2DYNAMIXEL_ID, 1) != 0) {
                base_left->initializeMotor();
                base_right->initializeMotor();
                shoulder_left->initializeMotor();
                shoulder_right->initializeMotor();
                elbow_left->initializeMotor();
                elbow_right->initializeMotor();
                wrist->initializeMotor();
            }

            // The base
            if (base_left->isOpened() && base_right->isOpened()) {
                base_joint.push_back(MOTOR_ID(BASE_LEFT_ID));
                base_joint.push_back(MOTOR_ID(BASE_RIGHT_ID));
                moveBase(BASE_START_POSITION);
            }
            else
                currentState = BASE_ERROR;

            // The shoulder
            if (shoulder_left->isOpened() && shoulder_right->isOpened()) {
                shoulder_joint.push_back(MOTOR_ID(SHOULDER_LEFT_ID));
                shoulder_joint.push_back(MOTOR_ID(SHOULDER_RIGHT_ID));
                moveShoulder(SHOULDER_START_POSITION);
            }
            else
                currentState = SHOULDER_ERROR;

            // The elbow
            if (elbow_left->isOpened() && elbow_right->isOpened()) {
                elbow_joint.push_back(MOTOR_ID(ELBOW_LEFT_ID));
                elbow_joint.push_back(MOTOR_ID(ELBOW_RIGHT_ID));
                moveElbow(ELBOW_START_POSITION);
            }
            else
                currentState = ELBOW_ERROR;

            if (wrist->isOpened())
                wrist->setCurrentAngle(WRIST_START_POSITION);
            else
                currentState = WRIST_ERROR;
            break;
        }
        case (SIMULATION_MODE):{
            break;
        }
    }

    // Insert Fallback to Sim mode here

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

const std::vector<arm_controller::Controller::MOTOR_ID>* arm_controller::Controller::getJoint(const Controller::JOINT_ID &joint) const {
    const std::vector<Controller::MOTOR_ID>* ptr = nullptr;
    switch(joint.id){
        case (BASE_JOINT_ID):{
            ptr = &base_joint;
            break;
        }
        case (SHOULDER_JOINT_ID):{
            ptr = &shoulder_joint;
            break;
        }
        case (ELBOW_JOINT_ID):{
            ptr = &elbow_joint;
            break;
        }
        case (WRIST_JOINT_ID):{
            ptr = &wrist_joint;
            break;
        }
    }
    return ptr;
}

bool arm_controller::Controller::moveSyncMotor(const std::vector<MOTOR_ID>& motors, const float angle) {
    switch(currentOperationMode){
        case(SIMULATION_MODE):{
            break;
        }
        case(AX12A_MODE):{
            auto IDs = std::vector<int>();
            for( auto&& id : motors ){
                IDs.push_back(id.id);
            }
            arm_controller::AX12A::moveSyncMotors(IDs,angle);
            break;
        }
        default:{
            break;
        }
    }
}
