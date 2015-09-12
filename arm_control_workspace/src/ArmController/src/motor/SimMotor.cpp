/**
 * \file        SimMotor.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        29/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "SimMotor.h"

arm_controller::SimMotor::SimMotor(const int ID, const int highAngleLimit, const int lowAngleLimit, const int angleOffset)
 :Motor(ID),
  highAngleLimit(highAngleLimit),
  lowAngleLimit(lowAngleLimit),
  angleOffset(angleOffset),
  STEP_PRECISION(0.3500f),
  currentAngle(0),
  currentSpeed(0),
  currentStep(0)
{
    this->opened = true;
}

arm_controller::SimMotor::~SimMotor() { }