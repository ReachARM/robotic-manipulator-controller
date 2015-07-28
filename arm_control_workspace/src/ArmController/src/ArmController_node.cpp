//
// Created by mojo on 16/06/15.
//


#include "RosServiceManager.h"
#include "Controller.h"

#include "ros/ros.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "ArmController_node");

    ros::NodeHandle n;

    ROS_INFO("Creating the controller");
    Controller controller;

    ROS_INFO("Creating the RosServiceManager");
    RosServiceManager rosServiceManager(&controller);

    ROS_INFO("Initializing services");
    rosServiceManager.initServices(&n);

    ros::spin();
    return 0;
}