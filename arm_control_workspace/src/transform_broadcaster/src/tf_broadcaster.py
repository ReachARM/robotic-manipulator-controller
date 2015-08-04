#!/usr/bin/env python
#
#   @file       tf_broadcaster.py
#   @author     Karl Ritchie
#   @date       03/08/2015  -  dd/mm/yyy
#   @license    The MIT License (MIT)
#
#               Copyright (c) 2015 Karl Ritchie
#
#               Permission is hereby granted, free of charge, to any person obtaining a copy
#               of this software and associated documentation files (the "Software"), to deal
#               in the Software without restriction, including without limitation the rights
#               to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#               copies of the Software, and to permit persons to whom the Software is
#               furnished to do so, subject to the following conditions:
#
#               The above copyright notice and this permission notice shall be included in
#               all copies or substantial portions of the Software.
#
#               THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#               IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#               FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
#               AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#               LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#               OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#               THE SOFTWARE.
#

import rospy
import math
import tf
import threading
import time

theta1 = 0.0
theta2 = 0.0
theta3 = 0.0

end = False

def publish_transform():
    while not end :

        br.sendTransform((0,0,0.0),
                         tf.transformations.quaternion_from_euler(0,0,theta1),
                         rospy.Time.now(),
                         "joint_1",
                         "base_link")

        br.sendTransform((0,0,0.05),
                         tf.transformations.quaternion_from_euler(0,theta2,0),
                         rospy.Time.now(),
                         "joint_2",
                         "joint_1")

        br.sendTransform((0,0,0.23),
                         tf.transformations.quaternion_from_euler(0,theta3,0),
                         rospy.Time.now(),
                         "joint_3",
                         "joint_2")

        br.sendTransform((0,0,0.165),
                         tf.transformations.quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "joint_4",
                         "joint_3")

        time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('transform_broadcaster')
    br = tf.TransformBroadcaster('tf')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    t = threading.Thread(target=publish_transform)
    t.start()

    ref = math.radians(90)

    while not rospy.is_shutdown():
        try:
            t = rospy.Time.now().to_sec() * math.pi

            inputs = (raw_input("Transform theta1 theta2 theta3 : ").split(' '))

            theta1 = math.radians(float(inputs[0]))
            theta2 = math.radians(float(inputs[1]))
            theta3 = math.radians(float(inputs[2]))
        except:
            continue

    end = True
