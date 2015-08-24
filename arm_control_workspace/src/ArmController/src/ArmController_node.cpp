/**
 * \file        ArmController_node.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        27/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */


#include "RosServiceManager.h"
#include "Controller.h"

#include "ros/ros.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "ArmController_node");

    ros::NodeHandle n;

    ROS_INFO("Creating the controller");
    arm_controler::Controller controller;

    ROS_INFO("Creating the RosServiceManager");
    arm_controler::RosServiceManager rosServiceManager(&controller);

    ROS_INFO("Initializing services");
    rosServiceManager.initServices(&n);

    ros::spin();
    return 0;
}