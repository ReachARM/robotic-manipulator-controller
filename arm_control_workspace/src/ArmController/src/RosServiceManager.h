/**
 * \file        RosServiceManager.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_ROSSERVICEMANAGER_H
#define ARMCONTROLLER_ROSSERVICEMANAGER_H

#include "ros/ros.h"

#include "Controller.h"

#include "ArmController/MoveRelativeTool.h"
#include "ArmController/MoveAbsoluteMotor.h"
#include "ArmController/MoveIncrementMotor.h"
#include "ArmController/GetArmStatus.h"
#include "ArmController/GetMotorAngle.h"

class RosServiceManager {

public:

    RosServiceManager(Controller* _controller);

    void initServices(ros::NodeHandle* nodeHandlePtr);

    bool moveRelativeTool(ArmController::MoveRelativeTool::Request &req,
                          ArmController::MoveRelativeTool::Response &res);

    bool moveAbsoluteMotor(ArmController::MoveAbsoluteMotor::Request &req,
                          ArmController::MoveAbsoluteMotor::Response &res);

    bool moveIncrementMotor(ArmController::MoveIncrementMotor::Request &req,
                            ArmController::MoveIncrementMotor::Response &res);

    bool getArmStatus(ArmController::GetArmStatus::Request &req,
                          ArmController::GetArmStatus::Response &res);

    bool getMotorAngle(ArmController::GetMotorAngle::Request &req,
                          ArmController::GetMotorAngle::Response &res);

private:

    bool isInit;

    Controller* controller;

    std::vector<ros::ServiceServer> services;

    // Locked methods
    RosServiceManager(){}
    RosServiceManager(const RosServiceManager&){}
    RosServiceManager& operator=(const RosServiceManager&){}
};


#endif //ARMCONTROLLER_ROSSERVICEMANAGER_H
