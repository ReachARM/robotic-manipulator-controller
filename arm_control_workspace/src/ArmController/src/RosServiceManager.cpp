/**
 * \file        RosServiceManager.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "RosServiceManager.h"

arm_controler::RosServiceManager::RosServiceManager(Controller * _controller)
:controller(_controller),
isInit(false)
{ ROS_INFO("Creation of the RosServiceManager."); }

void arm_controler::RosServiceManager::initServices(ros::NodeHandle* nodeHandlePtr) {
    if(!isInit){
        if( nodeHandlePtr != nullptr ) {
            services.push_back(nodeHandlePtr->advertiseService("move_absolute_motor",&RosServiceManager::moveAbsoluteMotor, this));
            services.push_back(nodeHandlePtr->advertiseService("move_relative_tool",&RosServiceManager::moveRelativeTool, this));
            services.push_back(nodeHandlePtr->advertiseService("move_increment_motor",&RosServiceManager::moveIncrementMotor, this));
            services.push_back(nodeHandlePtr->advertiseService("move_base", &RosServiceManager::moveBase, this));
            services.push_back(nodeHandlePtr->advertiseService("get_arm_status",&RosServiceManager::getArmStatus, this));
            services.push_back(nodeHandlePtr->advertiseService("get_motor_angle",&RosServiceManager::getMotorAngle, this));
            isInit = true;
        } else {
            ROS_WARN("Node handle pointer is null, cannot initialize services.");
        }
    } else {
        ROS_WARN("Services are already initialized.");
    }
}

bool arm_controler::RosServiceManager::moveRelativeTool(ArmController::MoveRelativeTool::Request &req,
                      ArmController::MoveRelativeTool::Response &res){
    ROS_INFO("Request for relative tool movement");
    ROS_INFO("tx: %d    ty: %d    tz:%d",(int)req.tx,(int)req.ty,(int)req.tz);
    ROS_INFO("roll: %f    pitch: %f    yaw:%f",(float)req.roll,(float)req.pitch,(float)req.yaw);
    controller->moveRelativeTool(req.tx,req.ty,req.tz,req.roll,req.pitch,req.yaw);
    return true;
}

bool arm_controler::RosServiceManager::moveAbsoluteMotor(ArmController::MoveAbsoluteMotor::Request &req,
                       ArmController::MoveAbsoluteMotor::Response &res){
    ROS_INFO("Request for absolute motor movement");
    ROS_INFO("Motor: %d   Angle: %f",(int)req.motorID,(float)req.angle);
    controller->moveAbsoluteMotor(static_cast<Controller::MOTOR_ID>(req.motorID),static_cast<float>(req.angle));
    return true;
}

bool arm_controler::RosServiceManager::getArmStatus(ArmController::GetArmStatus::Request &req,
                  ArmController::GetArmStatus::Response &res){
    ROS_INFO("Request for arm status");
    __uint8_t status = controller->getCurrentArmStatus();
    ROS_INFO("Sending response with status: %d", status);
    res.status = status;
    return true;
}

bool arm_controler::RosServiceManager::getMotorAngle(ArmController::GetMotorAngle::Request &req,
                   ArmController::GetMotorAngle::Response &res){
    ROS_INFO("Request for motor angle");
    ROS_INFO("Motor: %d", req.motorID);
    res.angle = controller->getMotorCurrentAngle(Controller::MOTOR_ID(req.motorID));
    return true;
}

bool arm_controler::RosServiceManager::moveIncrementMotor(ArmController::MoveIncrementMotor::Request &req,
                        ArmController::MoveIncrementMotor::Response &res){
    ROS_INFO("Request for increment motor movement");
    ROS_INFO("Motor :%d    Angle: %f",(int)req.motorID,(float)req.increment);
    controller->moveIncrementMotor(static_cast<Controller::MOTOR_ID>(req.motorID),static_cast<float>(req.increment));
    return true;
}

bool arm_controler::RosServiceManager::moveBase(ArmController::MoveBase::Request &req,
              ArmController::MoveBase::Response &res){
    ROS_INFO("Request for moving base at angle : %f", (float)req.angle);
    if( (float)req.angle > -1 && (float) req.angle < 361 )
        controller->moveBase(static_cast<float>(req.angle));
    else
        ROS_INFO("Bad angle");
    return true;
}

