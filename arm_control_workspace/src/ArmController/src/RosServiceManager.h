//
// Created by mojo on 16/06/15.
//

#ifndef ARMCONTROLLER_ROSSERVICEMANAGER_H
#define ARMCONTROLLER_ROSSERVICEMANAGER_H

#include "ros/ros.h"

#include "Controller.h"

#include "ArmController/MoveRelativeTool.h"
#include "ArmController/MoveAbsoluteMotor.h"
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
