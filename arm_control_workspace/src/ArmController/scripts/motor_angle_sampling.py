#!/usr/bin/env python

import rospy
from ArmController.srv import *

import time

f = file("/home/mojo/motor_sampling.txt", mode='w')

def get_current_angle_client(id):
     rospy.wait_for_service('get_motor_angle')
     try:
         get_current_angle = rospy.ServiceProxy('get_motor_angle', GetMotorAngle)
         resp1 = get_current_angle(id)
         print resp1
         resp1 = str(resp1).replace("angle: ", "")
         f.write(str(resp1))
         f.write('\n')
     except rospy.ServiceException, e:
         print "Service call failed: %s"%e


end = False
while not end:
    get_current_angle_client(1)
    time.sleep(0.01)

f.close()

