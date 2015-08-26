//
// Created by mojo on 26/08/15.
//

#ifndef ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
#define ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H

#include <kdl_parser/kdl_parser.hpp>

#include <string>

namespace arm_controller {

    class RosRobotStatePublisher {

    public:

        RosRobotStatePublisher(const std::string &urdfFilepath);

    private :

        // Locked methods
        RosRobotStatePublisher();

        RosRobotStatePublisher(const RosRobotStatePublisher &);

        RosRobotStatePublisher &operator=(const RosRobotStatePublisher &);


    };
}


#endif //ARMCONTROLLER_ROSROBOTSTATEPUBLISHER_H
