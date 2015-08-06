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


#include "../Motor/Motor.h"
#include "ros/ros.h"

#include "../util/Pointers.h"

class Controller {

public:

    struct MOTOR_ID {
        int id;
        explicit MOTOR_ID(const int _id):id(_id){ }
    };

    Controller();
    ~Controller(); // dxl_terminate();

    static const __uint8_t USB2DYNAMIXEL_ID = 0;
    static const __uint8_t BASE_ID = USB2DYNAMIXEL_ID +1;
    static const __uint8_t SHOULDER_ID = USB2DYNAMIXEL_ID +2;
    static const __uint8_t ELBOW_ID = USB2DYNAMIXEL_ID +3;
    static const __uint8_t WRIST_ID = USB2DYNAMIXEL_ID +4;

    enum MOTOR {

        BASE,
        SHOULDER,
        ELBOW,
        WRIST

    };

    enum ERROR{
        NO_ERROR,
        BASE_ERROR,
        SHOULDER_ERROR,
        ELBOW_ERROR,
        WRIST_ERROR
    };

    inline __uint8_t getCurrentArmStatus()const;

    inline float getMotorCurrentAngle(const MOTOR_ID& motor)const;

    bool moveRelativeTool(const float tX = 0.0F, const float tY = 0.0F, const float tZ = 0.0F,
                          const float roll = 0.0F, const float pitch = 0.0F, const float yaw = 0.0F);

    bool moveAbsoluteMotor( const MOTOR_ID& motor, const float targetAngle );

    bool moveIncrementMotor( const MOTOR_ID& motor, const float increment );

private:

    Motor* getMotor(const MOTOR_ID& motor)const;

    Controller& operator==(const Controller&);
    Controller(const Controller&);

    // The Motors
    std::unique_ptr<Motor> base, shoulder, elbow, wrist;

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

float Controller::getMotorCurrentAngle(const Controller::MOTOR_ID& motor)const {
    return getMotor(motor)->getCurrentAngle();
}

void Controller::resetError() {
    currentState = NO_ERROR;
}

__uint8_t Controller::getCurrentArmStatus() const {
    return currentState;
}

#endif //ARMCONTROLLER_CONTROLLER_H
