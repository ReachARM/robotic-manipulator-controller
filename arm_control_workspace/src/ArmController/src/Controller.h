//
// Created by mojo on 03/06/15.
//

#ifndef ARMCONTROLLER_CONTROLLER_H
#define ARMCONTROLLER_CONTROLLER_H


#include "../Motor/Motor.h"
#include "ros/ros.h"

#include <tr1/memory>

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

    inline __uint8_t getCurrentArmStatus()const;

    inline float getMotorCurrentAngle(const MOTOR_ID& motor)const;

    bool moveRelativeTool(const float tX = 0.0F, const float tY = 0.0F, const float tZ = 0.0F,
                          const float roll = 0.0F, const float pitch = 0.0F, const float yaw = 0.0F);

    bool moveAbsoluteMotor( const MOTOR_ID& motor, const float targetAngle );

private:

    Motor* getMotor(const MOTOR_ID& motor)const;

    Controller& operator==(const Controller&);
    Controller(const Controller&);

    // The Motors
    std::tr1::shared_ptr<Motor> base, shoulder, elbow, wrist;

    // Current State
    __uint8_t currentState;
    const int NO_ERROR;

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
