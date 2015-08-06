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

class Motor {

public :

    // CTOR - DTOR
    // Max angle = 244 deg @ step 815
    // Min angle = 61 deg @ step 200
    Motor( const __uint8_t id, const float lowAngleLimit = 200, const float highAngleLimit = 815, const int defaultBaudrate = 1,
           const int speedRpmLimit = 80, const int angle_offset_ = 0);
    ~Motor();

    //-----------------------------------------------
    // DYNAMIXEL Section
    //

    // Getters
    inline float getCurrentAngle() const;
    inline int getCurrentSpeed() const;
    inline __uint8_t getCurrentID() const;


    // Setters
    inline void halt();
    inline void setStep(const int step);
    inline void setCurrentAngle(const float angle);
    inline void setIncrementAngle(const float increment);
    inline void setSpeed(const int speed);

    // Status
    void PrintCommStatus(const int status) const;
    void PrintErrorCode() const;

    //
    // END OF DYNAMIXEL SECTION
    //-----------------------------------------------

    inline bool isOpened() const;

    void initializeMotor();

private :

    Motor& operator=(const Motor&);
    Motor(const Motor&);
    Motor();

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

    // Dynamixel controlling values
    int speedRpmLimit;
    int highStepLimit;
    int lowStepLimit;
    int defaultBaudrate;
    __uint8_t motor_id;

    const int angle_offset;

    // status
    bool opened;

};

// INLINES

inline void Motor::halt() {
    if( opened ) {
        auto current = dxl_read_word(motor_id, PRESENT_POSITION_L);
        dxl_write_word(motor_id, GOAL_POSITION_L, current);
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void Motor::setStep(const int step) {
    if( opened ) {
        if (step > lowStepLimit && step < highStepLimit)
            dxl_write_word(motor_id, GOAL_POSITION_L, step);
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void Motor::setSpeed( const int speed ){
    if( opened ) {
        if (speed >= 0 && speed < speedRpmLimit)
            dxl_write_word(motor_id, MOVING_SPEED_L, speed);
        else
            ROS_ERROR("Motor:%d cannot set speed to %d , must be between 0 and %d", motor_id,speed,speedRpmLimit);
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void Motor::setCurrentAngle(const float angle) {
    if( opened ){
        ROS_INFO("Angle : %f", angle);
        if( (angle_offset+angle) > (lowStepLimit*STEP_PRECISION) && (angle_offset - angle) < (highStepLimit*STEP_PRECISION) )
            if((angle-angle_offset)<0)
                dxl_write_word(motor_id,GOAL_POSITION_L,static_cast<int>((angle_offset-angle)/STEP_PRECISION));
            else
                dxl_write_word(motor_id,GOAL_POSITION_L,static_cast<int>((angle+angle_offset)/STEP_PRECISION));
        else
            ROS_ERROR("Motor:%d cannot move to angle:%f , limits are %f to %f",motor_id,angle,(lowStepLimit*STEP_PRECISION),(highStepLimit*STEP_PRECISION));
    } else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline void Motor::setIncrementAngle(const float increment){
    if( opened ){
        auto currentAngle = getCurrentAngle();
        ROS_INFO("Current angle %f", currentAngle);
        ROS_INFO("Increment %f", increment);
        setCurrentAngle(currentAngle+increment);
    }  else {
        ROS_WARN("Motor:%d is not opened", motor_id);
    }
}

inline __uint8_t Motor::getCurrentID() const {
    return motor_id;
}

inline int Motor::getCurrentSpeed() const {
    int current_speed = dxl_read_word(motor_id,MOVING_SPEED_L);
    if( dxl_get_result() == COMM_RXSUCCESS ) {
        return current_speed;
    } else {
        ROS_WARN("Error while reading speed on Motor:%d", motor_id);
    }
    return -1;
}

inline float Motor::getCurrentAngle() const {
    auto  angleRead = dxl_read_word(motor_id, PRESENT_POSITION_L);
    auto current_angle = static_cast<float>(dxl_read_word(motor_id, PRESENT_POSITION_L)) * STEP_PRECISION;
    if( dxl_get_result() == COMM_RXSUCCESS ) {

        return  angle_offset - current_angle;
    } else {
        ROS_WARN("Error code is : ", dxl_get_result());
        ROS_WARN("Error while reading angle on Motor:%d", motor_id);
    }
    return -1.0F;
}

inline bool Motor::isOpened() const {
    return opened;
}

#endif //ARMCONTROLLER_MOTOR_H
