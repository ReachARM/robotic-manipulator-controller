/**
 * \file        Motor.cpp
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        02/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#include "Motor.h"

arm_controller::Motor::Motor(const int id)
:motor_id(id),
 opened(false)
{}

arm_controller::Motor::~Motor() {
}