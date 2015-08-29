/**
 * \file        AX12A.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        29/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_AX12A_H
#define ARMCONTROLLER_AX12A_H

#include "Motor.h"

namespace arm_controller {

    class AX12A : public arm_controller::Motor {

    public:

        AX12A(const int ID, const int defaultBaudrate, const int speedLimit, const int highAngleLimit, const int lowAngleLimit, const int angleOffset);
        virtual ~AX12A();

        virtual float getCurrentAngle() const override;
        virtual float getRelativeMotorAngle(const float angle) const override;
        virtual int getCurrentStep() const override;
        virtual int getCurrentSpeed() const override;

        virtual void halt() override;
        virtual void setStep(const int step) override;
        virtual void setCurrentAngle(const float angle) override;
        virtual void setIncrementAngle(const float increment) override;
        virtual void setSpeed(const int speed) override;
        virtual void initializeMotor() override;

        inline int getHighAngleLimit() const;
        inline int getLowAngleLimit() const;
        inline int getSpeedLimit() const;
        inline int getDefaultBaudrate() const;

        // Status
        void printCommStatus(const int status) const;
        void printErrorCode() const;

        static void moveSyncMotors(const std::vector<int>& motorIDs, const float angle);

    private:

        // Control table address
        // L : Low byte
        // H : high byte
        // A word adress is 16 bits (i.e. composed of a L and H byte)
        static const auto CW_ANGLE_LIMIT_L = 6;
        static const auto CW_ANGLE_LIMIT_H = 7;
        static const auto CCW_ANGLE_LIMIT_L = 8;
        static const auto CCW_ANGLE_LIMIT_H = 9;
        static const auto GOAL_POSITION_L = 30;
        static const auto GOAL_POSITION_H = 31;
        static const auto PRESENT_POSITION_L = 36;
        static const auto PRESENT_POSITION_H = 37;
        static const auto MOVING_SPEED_L = 32;
        static const auto MOVING_SPEED_H = 33;

        static constexpr auto STEP_PRECISION = 0.3500;


        static const auto MAX_RPM_SPEED = 114;
        static const auto BONDING_BOX_SIZE_FOR_INITIALIZATION = 10; // +/- 10 degrees at init is allowed
        static const auto MIN_VALUE_FOR_INFINITE_JOINTS = 0;
        static const auto MAX_VALUE_FOR_INFINITE_JOINTS = 1000;

        static const auto INFINITE_BOUNDARIES = -1;

        // Dynamixel controlling values
        int speedRpmLimit;
        int highAngleLimit;
        int lowAngleLimit;
        int defaultBaudrate;
        int lowStepLimit;
        int highStepLimit;
        int angleOffset;

        // Locked
        AX12A(const AX12A&);
        AX12A& operator = (const AX12A&);
        AX12A();

    };

}

// INLINES

inline void arm_controller::AX12A::halt() {
    auto motor_id = getCurrentID();
    if( isOpened() ) {
        auto current = dxl_read_word(motor_id, PRESENT_POSITION_L);
        dxl_write_word(getCurrentID(), GOAL_POSITION_L, current);
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void arm_controller::AX12A::setStep(const int step) {
    auto motor_id = getCurrentID();
    if( isOpened() ) {
        if (step > lowStepLimit && step < highStepLimit)
            dxl_write_word(motor_id, GOAL_POSITION_L, step);
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void arm_controller::AX12A::setSpeed( const int speed ){
    auto motor_id = getCurrentID();
    if( isOpened() ) {
        if (speed >= 0 && speed < speedRpmLimit)
            dxl_write_word(motor_id, MOVING_SPEED_L, speed);
        else
            ROS_ERROR("Motor:%d cannot set speed to %d , must be between 0 and %d", motor_id,speed,speedRpmLimit);
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void arm_controller::AX12A::setIncrementAngle(const float increment){
    auto motor_id = getCurrentID();
    if( isOpened() ){
        auto currentAngle = getCurrentAngle();
        ROS_INFO("Current angle %f", currentAngle);
        ROS_INFO("Increment %f", increment);
        setCurrentAngle(currentAngle+increment);
    }  else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline int arm_controller::AX12A::getCurrentSpeed() const {
    auto motor_id = getCurrentID();
    int current_speed = dxl_read_word(motor_id,MOVING_SPEED_L);
    if( dxl_get_result() == COMM_RXSUCCESS ) {
        return current_speed;
    } else {
        ROS_WARN("Error while reading speed on Motor:%d", motor_id);
    }
    return -1;
}

inline float arm_controller::AX12A::getCurrentAngle() const {
    auto motor_id = getCurrentID();
    auto angleRead = dxl_read_word(motor_id, PRESENT_POSITION_L);
    auto current_angle = static_cast<float>(dxl_read_word(motor_id, PRESENT_POSITION_L)) * STEP_PRECISION;
    if( dxl_get_result() == COMM_RXSUCCESS ) {
        return  angleOffset - current_angle;
    } else {
        ROS_WARN("Error code is : %d ", dxl_get_result());
        ROS_WARN("Error while reading angle on Motor:%d", motor_id);
    }
    return -1.0F;
}

inline int arm_controller::AX12A::getCurrentStep() const {
    auto motor_id = getCurrentID();
    return dxl_read_word(motor_id, PRESENT_POSITION_L);
}

inline float arm_controller::AX12A::getRelativeMotorAngle(const float angle) const {
    return 0.0f; // TODO
}

#endif //ARMCONTROLLER_AX12A_H
