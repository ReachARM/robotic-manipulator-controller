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

int main(int argc, char** argv){

    ros::init(argc, argv, "ArmController_node");

    ros::NodeHandle n;

    ROS_INFO("Creating the controller");
    arm_controller::Controller controller;

    ROS_INFO("Creating the RosServiceManager");
    arm_controller::RosServiceManager rosServiceManager(&controller);

    ROS_INFO("Initializing services");
    rosServiceManager.initServices(&n);

    ROS_INFO("Starting the robot state publisher");
    arm_controller::RosRobotStatePublisher robotStatePublisher("...",&controller); // TODO FIXME with relative path

    ros::spin();
    return 0;
}