//
// Created by mojo on 01/06/15.
//

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
           const int speedRpmLimit = 80);
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
    static const int CW_ANGLE_LIMIT_L = 6;
    static const int CW_ANGLE_LIMIT_H = 7;
    static const int CCW_ANGLE_LIMIT_L = 8;
    static const int CCW_ANGLE_LIMIT_H = 9;
    static const int GOAL_POSITION_L = 30;
    static const int GOAL_POSITION_H = 31;
    static const int PRESENT_POSITION_L = 36;
    static const int PRESENT_POSITION_H = 37;
    static const int MOVING_SPEED_L = 32;
    static const int MOVING_SPEED_H = 33;

    static constexpr float STEP_PRECISION = 0.3500;

    static const int MAX_RPM_SPEED = 114;

    // Dynamixel controlling values
    int speedRpmLimit;
    int highStepLimit;
    int lowStepLimit;
    int defaultBaudrate;
    int motor_id;

    // status
    bool opened;

};

// INLINES

inline void Motor::halt() {
    if( opened ) {
        int current = dxl_read_word(motor_id, PRESENT_POSITION_L);
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
        if( angle > (lowStepLimit*STEP_PRECISION) && angle < (highStepLimit*STEP_PRECISION) )
            dxl_write_word(motor_id,GOAL_POSITION_L,static_cast<int>(angle/STEP_PRECISION));
        else
            ROS_ERROR("Motor:%d cannot move to angle:%f , limits are %f to %f",motor_id,angle,(lowStepLimit*STEP_PRECISION),(highStepLimit*STEP_PRECISION));
    } else {
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
    __uint16_t  angleRead = dxl_read_word(motor_id, PRESENT_POSITION_L);
    float current_angle = static_cast<float>(dxl_read_word(motor_id, PRESENT_POSITION_L)) * STEP_PRECISION;
    if( dxl_get_result() == COMM_RXSUCCESS ) {
        return current_angle;
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
