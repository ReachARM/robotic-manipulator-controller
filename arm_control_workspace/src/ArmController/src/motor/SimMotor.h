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

        virtual inline float getCurrentAngle() override;
        virtual inline float getCurrentSpeed() override;
        virtual inline float getRelativeMotorAngle(const float angle) const override;
        virtual inline int getCurrentStep() override;

        virtual void halt() override;
        virtual void setStep(const int step) override;
        virtual void setCurrentAngle(const float angle) override;
        virtual void setIncrementAngle(const float increment) override;
        virtual void setSpeed(const int speed) override;
        virtual void initializeMotor() override;

    private:

        // Locked
        SimMotor(const SimMotor&);
        SimMotor& operator = (const SimMotor&);
        SimMotor();

    };

}

#endif //ARMCONTROLLER_SIMMOTOR_H
