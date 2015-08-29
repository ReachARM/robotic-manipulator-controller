/**
 * \file        RosRobotStatePublisher.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        29/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
#define ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H

#include "kdl/tree.hpp"
#include "../Controller.h"

#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

#include <thread>
#include <string>

namespace arm_controller {

    class RosRobotStatePublisher {

    public:

        ~RosRobotStatePublisher();
        RosRobotStatePublisher(const std::string &urdfFilepath, const arm_controller::Controller* controller);

        void initPublisher();


    private :

        void publishJoints();
        inline void stopPublisher();
        // Locked methods
        RosRobotStatePublisher();
        RosRobotStatePublisher(const RosRobotStatePublisher &);
        RosRobotStatePublisher &operator=(const RosRobotStatePublisher &);

        bool isInit;
        bool publish;

        std::string urdfPath;

        KDL::Tree robotKDLTree;

        const arm_controller::Controller* controller;

        std::unique_ptr<robot_state_publisher::RobotStatePublisher> publisher;
        std::thread publishingThread;

    };
}

void arm_controller::RosRobotStatePublisher::stopPublisher() {
    publish = false;
    publishingThread.join();
}

#endif //ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
