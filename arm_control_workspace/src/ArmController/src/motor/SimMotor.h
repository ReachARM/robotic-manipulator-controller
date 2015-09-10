/**
 * \file        SimMotor.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        29/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_SIMMOTOR_H
#define ARMCONTROLLER_SIMMOTOR_H

#include "Motor.h"

namespace arm_controller {

    class SimMotor : public arm_controller::Motor {

    public:

        SimMotor(const int ID, const int highAngleLimit, const int lowAngleLimit, const int angleOffset);
        virtual ~SimMotor();

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

    private:

        static const auto INFINITE_BOUNDARIES = -1;

        int highAngleLimit;
        int lowAngleLimit;
        int angleOffset;

        int currentSpeed;
        int currentAngle;
        int currentStep;

        const int STEP_PRECISION;

        // Locked
        SimMotor(const SimMotor&);
        SimMotor& operator = (const SimMotor&);
        SimMotor();

    };

}

inline float arm_controller::SimMotor::getCurrentAngle() const {
    return currentAngle;
}

inline int arm_controller::SimMotor::getCurrentStep() const {
    return currentStep;
}

inline int arm_controller::SimMotor::getCurrentSpeed() const {
    return currentSpeed;
}

inline float arm_controller::SimMotor::getRelativeMotorAngle(const float angle) const {
    return 0; // TODO implement this !
}

inline void arm_controller::SimMotor::halt() {

}

inline void arm_controller::SimMotor::setStep(const int step) {
    setCurrentAngle(step*STEP_PRECISION);
};

inline void arm_controller::SimMotor::setCurrentAngle(const float angle) {
    if( angle >= lowAngleLimit && angle <= highAngleLimit ){
        currentAngle = angle;
        currentStep = currentAngle/STEP_PRECISION;
    } else {
        ROS_WARN("Cannot set angle, not within boundaries");
    }
};

inline void arm_controller::SimMotor::setIncrementAngle(const float increment) {
    setCurrentAngle(currentAngle + increment);
};

inline void arm_controller::SimMotor::setSpeed(const int speed) {
    currentSpeed = speed;
}

inline void arm_controller::SimMotor::initializeMotor() {
    opened = true;
}

#endif //ARMCONTROLLER_SIMMOTOR_H
