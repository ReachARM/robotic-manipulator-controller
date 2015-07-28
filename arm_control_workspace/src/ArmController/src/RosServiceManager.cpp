//
// Created by mojo on 16/06/15.
//

#include "RosServiceManager.h"

RosServiceManager::RosServiceManager(Controller * _controller)
:controller(_controller),
isInit(false)
{ ROS_INFO("Creation of the RosServiceManager."); }

void RosServiceManager::initServices(ros::NodeHandle* nodeHandlePtr) {
    if(!isInit){
        if( nodeHandlePtr != nullptr ) {
            services.push_back(nodeHandlePtr->advertiseService("move_absolute_motor",&RosServiceManager::moveAbsoluteMotor, this));
            services.push_back(nodeHandlePtr->advertiseService("move_relative_tool",&RosServiceManager::moveRelativeTool, this));
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

bool RosServiceManager::moveRelativeTool(ArmController::MoveRelativeTool::Request &req,
                      ArmController::MoveRelativeTool::Response &res){
    ROS_INFO("Request for relative tool movement");
    ROS_INFO("tx: %d    ty: %d    tz:%d",(int)req.tx,(int)req.ty,(int)req.tz);
    ROS_INFO("roll: %f    pitch: %f    yaw:%f",(float)req.roll,(float)req.pitch,(float)req.yaw);
    controller->moveRelativeTool(req.tx,req.ty,req.tz,req.roll,req.pitch,req.yaw);
    return true;
}

bool RosServiceManager::moveAbsoluteMotor(ArmController::MoveAbsoluteMotor::Request &req,
                       ArmController::MoveAbsoluteMotor::Response &res){
    ROS_INFO("Request for absolute motor movement");
    ROS_INFO("Motor: %d   Angle: %f",(int)req.motorID,(float)req.angle);
    controller->moveAbsoluteMotor(static_cast<Controller::MOTOR_ID>(req.motorID),static_cast<float>(req.angle));
    return true;
}

bool RosServiceManager::getArmStatus(ArmController::GetArmStatus::Request &req,
                  ArmController::GetArmStatus::Response &res){
    ROS_INFO("Request for arm status");
    __uint8_t status = controller->getCurrentArmStatus();
    ROS_INFO("Sending response with status: %d", status);
    res.status = status;
    return true;
}

bool RosServiceManager::getMotorAngle(ArmController::GetMotorAngle::Request &req,
                   ArmController::GetMotorAngle::Response &res){
    ROS_INFO("Request for motor angle");
    ROS_INFO("Motor: %d", req.motorID);
    res.angle = controller->getMotorCurrentAngle(Controller::MOTOR_ID(req.motorID));
    return true;
}

