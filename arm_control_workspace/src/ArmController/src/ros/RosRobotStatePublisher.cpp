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
    publish = false;
    publishingThread.join();

}

void arm_controller::RosRobotStatePublisher::publishJoints() {
    while(publish){
        publisher.get()->publishFixedTransforms("");
    }
}

void arm_controller::RosRobotStatePublisher::InitPublisher() {
    if(!isInit) {
        if (!kdl_parser::treeFromFile(urdfPath, robotKDLTree)) {
            ROS_ERROR("Failed to construct kdl tree");
        } else {
            ROS_INFO("KDL tree successfully constructed");
            publisher = std::unique_ptr<robot_state_publisher::RobotStatePublisher>(
                    new robot_state_publisher::RobotStatePublisher(robotKDLTree));
            publish = true;
            publishingThread = std::thread(&arm_controller::RosRobotStatePublisher::publishJoints, this);
            isInit = true;
        }
    } else {
        ROS_WARN("Publisher is already initialized");
    }
}