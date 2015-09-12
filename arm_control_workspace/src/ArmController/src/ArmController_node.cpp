/**
 * \file        ArmController_node.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        27/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */


#include "ros/RosServiceManager.h"
#include "Controller.h"
#include "ros/ros.h"
#include "ros/RosRobotStatePublisher.h"

const auto OPERATION_MODE_FLAG = std::string("mode");
const auto SIMULATION_MODE = std::string("SIM");
const auto AX12A_MODE = std::string("AX12A");

int main(int argc, char** argv){

    ros::init(argc, argv, "ArmController_node");

    auto n = ros::NodeHandle();

    auto strOperationMode = std::string();
    auto operationMode = arm_controller::Controller::OPERATION_MODE();

    // If the name is invalid or the sequence throws and exception
    // the Controller will be defaulted to Simulation mode
    // IT MUST BE SPECIFIED to run in AX12A mode, otherwise it will
    // run in Simulation mode
    // use _mode:=AX12A on the command line after the rosrun command
    ROS_INFO("Getting operation mode...");
    try {
        if (n.getParam(OPERATION_MODE_FLAG, strOperationMode)) {
            if(strOperationMode.compare(SIMULATION_MODE)==0){
                ROS_INFO("Setting simulation mode...");
                operationMode = arm_controller::Controller::OPERATION_MODE::SIMULATION_MODE;
            } else if ( strOperationMode.compare(AX12A_MODE) == 0){
                ROS_INFO("Setting AX-12A mode...");
                operationMode = arm_controller::Controller::OPERATION_MODE::AX12A_MODE;
            } else {
                ROS_INFO("Invalid operation mode... Setting simulation mode...");
                operationMode = arm_controller::Controller::OPERATION_MODE::SIMULATION_MODE;
            }
        } else {
            ROS_INFO("No operation mode specified... Setting simulation mode...");
            operationMode = arm_controller::Controller::OPERATION_MODE::SIMULATION_MODE;
        }
    } catch (ros::InvalidNameException){
        ROS_INFO("Exception while getting operation mode... Setting simulation mode...");
        operationMode = arm_controller::Controller::OPERATION_MODE::SIMULATION_MODE;
    }

    ROS_INFO("Creating the controller");
    arm_controller::Controller controller(operationMode);

    ROS_INFO("Creating the RosServiceManager");
    arm_controller::RosServiceManager rosServiceManager(&controller);

    ROS_INFO("Initializing services");
    rosServiceManager.initServices(&n);

    ROS_INFO("Starting the robot state publisher");
    arm_controller::RosRobotStatePublisher robotStatePublisher("/home/mojo/robot.urdf",&controller); // TODO FIXME with relative path
    robotStatePublisher.initPublisher();

    ros::spin();
    return 0;
}