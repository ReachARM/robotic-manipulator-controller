//
// Created by mojo on 26/08/15.
//

#include "RosRobotStatePublisher.h"

arm_controller::RosRobotStatePublisher::RosRobotStatePublisher(const std::string& urdfFilepath, const arm_controller::Controller* controller)
:publish(false),
 urdfPath(urdfFilepath),
 robotKDLTree(KDL::Tree()),
 publisher(nullptr),
 isInit(false),
 controller(controller)
{}

arm_controller::RosRobotStatePublisher::~RosRobotStatePublisher() {
    try{
        stopPublisher();
    } catch(...){
        ROS_ERROR("Error while closing the publishing thread");
    }
    ROS_INFO("Publisher was successfully closed");
}

void arm_controller::RosRobotStatePublisher::publishJoints() {
    if(isInit) {
        ROS_INFO("Starting the state publisher");
        while (publish) {
            publisher.get()->publishFixedTransforms("");
            usleep(1000);
        }
    } else {
        ROS_WARN("Publisher is not initialized");
    }
}

void arm_controller::RosRobotStatePublisher::initPublisher() {
    if(!isInit) {
        if (!kdl_parser::treeFromFile(urdfPath, robotKDLTree)) {
            ROS_ERROR("Failed to construct kdl tree");
        } else {
            ROS_INFO("KDL tree successfully constructed");
            publisher = std::unique_ptr<robot_state_publisher::RobotStatePublisher>(
                    new robot_state_publisher::RobotStatePublisher(robotKDLTree));
            publish = true;
            publishingThread = std::thread(&arm_controller::RosRobotStatePublisher::publishJoints, this);
            publishingThread.detach();
            isInit = true;
            ROS_INFO("Publisher successfully initialized");
        }
    } else {
        ROS_WARN("Publisher is already initialized");
    }
}