/**
 * \file        Pointers.h
 * \author      Karl Ritchie <ritchie.karl@gmail.com>
 * \date        02/08/2015 - dd/mm/yyyy
 * \copyrights  Copyright (c) 2015 Karl Ritchie. All rights reserved.
 *              Use of this source code is governed by the MIT license that can be
 *              found in the LICENSE file.
 */

#ifndef ARMCONTROLLER_POINTERS_H
#define ARMCONTROLLER_POINTERS_H

#include <memory>
#include <type_traits>

namespace std{

    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args &&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

}

#endif //ARMCONTROLLER_POINTERS_H
