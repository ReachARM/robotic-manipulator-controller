/**
 * \file        RosRobotStatePublisher.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        29/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

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
        auto transforms = std::map<std::string, double>();
        while (publish) {

            // Based on the Robot state publisher code
            ROS_DEBUG("Publishing transforms for moving joints");
            std::vector<tf::StampedTransform> tf_transforms;

            tf::Quaternion quat;

            tf::StampedTransform base, shoulder, elbow;
            quat = tf::Quaternion();
            quat.setRPY(0,0, controller->getJointCurrentAngle(Controller::JOINT_ID(Controller::BASE_JOINT_ID)) * M_PI / 180.000F);
            base = tf::StampedTransform();
            base.stamp_ = ros::Time::now();
            base.frame_id_ = "base_link";
            base.child_frame_id_ = "joint_1";
            base.setOrigin(tf::Vector3(0,0,0));
            base.setRotation(quat);
            tf_transforms.push_back(base);

            quat = tf::Quaternion();
            quat.setRPY(0,controller->getJointCurrentAngle(Controller::JOINT_ID(Controller::SHOULDER_JOINT_ID)) * M_PI / 180.000F,0);
            shoulder = tf::StampedTransform();
            shoulder.stamp_ = ros::Time::now();
            shoulder.frame_id_ = "joint_1";
            shoulder.child_frame_id_ = "joint_2";
            shoulder.setOrigin(tf::Vector3(0,0,1));
            shoulder.setRotation(quat);
            tf_transforms.push_back(shoulder);

            quat = tf::Quaternion();
            quat.setRPY(0,controller->getJointCurrentAngle(Controller::JOINT_ID(Controller::ELBOW_JOINT_ID)) * M_PI / 180.000F,0);
            elbow = tf::StampedTransform();
            elbow.stamp_ = ros::Time::now();
            elbow.frame_id_ = "joint_2";
            elbow.child_frame_id_ = "joint_3";
            elbow.setOrigin(tf::Vector3(0,0,2));
            elbow.setRotation(quat);
            tf_transforms.push_back(elbow);

            publisher.get()->sendTransform(tf_transforms);

            transforms.clear();
            usleep(1000);
        }
    } else {
        ROS_WARN("Publisher is not initialized");
    }
}

void arm_controller::RosRobotStatePublisher::initPublisher() {
    if(!isInit) {
        ROS_INFO("KDL tree successfully constructed");
        isInit = true;
        publisher = std::unique_ptr<tf::TransformBroadcaster>(
                new tf::TransformBroadcaster());
        publish = true;
        
        publishingThread = std::thread(&arm_controller::RosRobotStatePublisher::publishJoints, this);
        publishingThread.detach();

        ROS_INFO("Publisher successfully initialized");
    } else {
        ROS_WARN("Publisher is already initialized");
    }
}