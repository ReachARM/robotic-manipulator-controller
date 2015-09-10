/**
 * \file        Controller.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_CONTROLLER_H
#define ARMCONTROLLER_CONTROLLER_H


#include "motor/Motor.h"
#include "motor/AX12A.h"
#include "motor/SimMotor.h"

#include "ros/ros.h"

#include "util/Pointers.h"

namespace arm_controller {

    class Controller {

    public:

        struct MOTOR_ID {
            int id;
            explicit MOTOR_ID(const int _id) : id(_id) { }
        };

        struct JOINT_ID {
            int id;
            explicit JOINT_ID(const int _id) : id(_id) {}
        };

        enum OPERATION_MODE{
            SIMULATION_MODE,
            AX12A_MODE
        };

        Controller(const OPERATION_MODE& mode);

        ~Controller(); // dxl_terminate();

        static const int USB2DYNAMIXEL_ID = 0;
        static const int BASE_LEFT_ID = USB2DYNAMIXEL_ID + 1;
        static const int BASE_RIGHT_ID = USB2DYNAMIXEL_ID + 2;
        static const int SHOULDER_LEFT_ID = USB2DYNAMIXEL_ID + 3;
        static const int SHOULDER_RIGHT_ID = USB2DYNAMIXEL_ID + 4;
        static const int ELBOW_LEFT_ID = USB2DYNAMIXEL_ID + 5;
        static const int ELBOW_RIGHT_ID = USB2DYNAMIXEL_ID + 6;
        static const int WRIST_ID = USB2DYNAMIXEL_ID + 7;

        enum MOTOR {
            BASE_LEFT,
            BASE_RIGHT,
            SHOULDER_LEFT,
            SHOULDER_RIGHT,
            ELBOW_LEFT,
            ELBOW_RIGHT,
            WRIST
        };

        enum JOINT {
            BASE_JOINT_ID = 0,
            SHOULDER_JOINT_ID = 1,
            ELBOW_JOINT_ID = 2,
            WRIST_JOINT_ID = 3
        };

        enum ERROR {
            NO_ERROR,
            BASE_ERROR,
            SHOULDER_ERROR,
            ELBOW_ERROR,
            WRIST_ERROR
        };



        inline __uint8_t getCurrentArmStatus() const;

        inline float getMotorCurrentAngle(const MOTOR_ID &motor) const;

        inline float getJointCurrentAngle( const JOINT_ID &joint) const;

        bool moveRelativeTool(const float tX = 0.0F, const float tY = 0.0F, const float tZ = 0.0F,
                              const float roll = 0.0F, const float pitch = 0.0F, const float yaw = 0.0F);

        bool moveAbsoluteMotor(const MOTOR_ID &motor, const float targetAngle);

        bool moveIncrementMotor(const MOTOR_ID &motor, const float increment);

        inline bool moveBase(const float angle);

        inline bool moveShoulder(const float angle);

        inline bool moveElbow(const float angle);

    private:



        OPERATION_MODE currentOperationMode;

        bool moveSyncMotor(const std::vector<MOTOR_ID> &motors, const float angle);

        Motor *getMotor(const MOTOR_ID &motor) const;

        const std::vector<MOTOR_ID>* getJoint(const JOINT_ID &joint) const;

        // Locked methods
        Controller &operator==(const Controller &);
        Controller(const Controller &);

        // The Motors
        std::unique_ptr<Motor> base_left, base_right,
                shoulder_left, shoulder_right,
                elbow_left, elbow_right,
                wrist;

        std::vector<MOTOR_ID> base_joint, shoulder_joint, elbow_joint, wrist_joint;

        // Current State
        ERROR currentState;

        const int BASE_START_POSITION;
        const int SHOULDER_START_POSITION;
        const int ELBOW_START_POSITION;
        const int WRIST_START_POSITION;

        // Reset the currentState to NO_ERROR
        // Used to clean error state;
        inline void resetError();

    };

}

float arm_controller::Controller::getMotorCurrentAngle(const Controller::MOTOR_ID& motor)const {
    return getMotor(motor)->getCurrentAngle();
}

float arm_controller::Controller::getJointCurrentAngle(const JOINT_ID &joint) const {
    auto angle_left_servo = getMotorCurrentAngle(getJoint(joint)->at(0));
    auto angle_right_servo = getMotorCurrentAngle(getJoint(joint)->at(1));
    return static_cast<float>((angle_left_servo+angle_right_servo)/2.0);
}

void arm_controller::Controller::resetError() {
    currentState = NO_ERROR;
}

__uint8_t arm_controller::Controller::getCurrentArmStatus() const {
    return currentState;
}

bool arm_controller::Controller::moveBase(const float angle) {
    moveSyncMotor(base_joint,angle);
}

bool arm_controller::Controller::moveShoulder(const float angle) {
    moveSyncMotor(shoulder_joint,angle);
}

bool arm_controller::Controller::moveElbow(const float angle) {
    moveSyncMotor(elbow_joint,angle);
}

#endif //ARMCONTROLLER_CONTROLLER_H
