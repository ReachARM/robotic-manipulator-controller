/**
 * \file        Motor.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        16/07/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_MOTOR_H
#define ARMCONTROLLER_MOTOR_H

#include "dynamixel.h"

#include "ros/ros.h"

#include <stdio.h>
#include <iostream>

namespace arm_controller {

    class Motor {

    public :

        Motor(const int id);

        virtual ~Motor();

        // Getters
        virtual float getCurrentAngle() const = 0;
        virtual int getCurrentSpeed() const = 0;
        virtual int getCurrentStep() const = 0;
        virtual float getRelativeMotorAngle(const float angle) const = 0;

        inline int getCurrentID() const;
        inline bool isOpened() const;

        // Setters
        virtual void halt() = 0;
        virtual void setStep(const int step) = 0;
        virtual void setCurrentAngle(const float angle) = 0;
        virtual void setIncrementAngle(const float increment) = 0;
        virtual void setSpeed(const int speed) = 0;
        virtual void initializeMotor() = 0;

    protected :

        bool opened;

    private :

        // Locked
        Motor &operator=(const Motor &);
        Motor(const Motor &);
        Motor();

        int motor_id;

    };

}

inline int arm_controller::Motor::getCurrentID() const {
    return motor_id;
}

inline bool arm_controller::Motor::isOpened() const {
    return opened;
}

#endif //ARMCONTROLLER_MOTOR_H
