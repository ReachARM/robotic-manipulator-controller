//
// Created by mojo on 26/08/15.
//

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

        void InitPublisher();

    private :

        void publishJoints();

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


#endif //ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
