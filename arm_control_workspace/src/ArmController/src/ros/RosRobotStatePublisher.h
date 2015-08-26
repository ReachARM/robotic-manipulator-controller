//
// Created by mojo on 26/08/15.
//

#ifndef ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
#define ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H

#include "kdl
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

#include <string>

namespace arm_controller {

    class RosRobotStatePublisher {

    public:

        RosRobotStatePublisher(const std::string &urdfFilepath);

        void publish();

    private :

        void publishJoints();

        // Locked methods
        RosRobotStatePublisher();
        RosRobotStatePublisher(const RosRobotStatePublisher &);
        RosRobotStatePublisher &operator=(const RosRobotStatePublisher &);

        bool publish;

    };
}


#endif //ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
