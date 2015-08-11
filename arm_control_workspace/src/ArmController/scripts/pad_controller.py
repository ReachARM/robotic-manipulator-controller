#!/usr/bin/env python

import pygame
import pygame.display
import pygame.joystick
import rospy
from ArmController.srv import *
from pygame.locals import *

import time

def move_absolute_motor_client(id,angle):
     rospy.wait_for_service('move_increment_motor')
     try:
         move_absolute_motor = rospy.ServiceProxy('move_increment_motor', MoveIncrementMotor)
         resp1 = move_absolute_motor(id, angle)
     except rospy.ServiceException, e:
         print "Service call failed: %s"%e

pygame.display.init()

pygame.joystick.init() #initialize joystick module
pygame.joystick.get_init() #verify initialization (boolean)

joystick_count = pygame.joystick.get_count()#get number of joysticks
print('%d joystick(s) connected' %joystick_count)#print number

joystick_object = pygame.joystick.Joystick(0)
#create an instance of a joystick
#first joystick is [0] in the list
#haven't had much luck with multiple joysticks

joystick_object.get_name()
print joystick_object.get_name()
#grab joystick name - flightstick, gravis...
#can (and is in this case) be done before initialization

joystick_object.init()

joystick_object.get_init()
#verify initialization (maybe cool to do some
#error trapping with this so game doesn't crash

num_axes = joystick_object.get_numaxes()
num_buttons = joystick_object.get_numbuttons()



print 'Joystick has %d axes and %d buttons' %(num_axes,num_buttons)
end = False
while not end:
    pygame.event.pump()
    #necessary for os to pass joystick events

    if joystick_object.get_button(0) > 0 :
        move_absolute_motor_client(1,7)
    if joystick_object.get_button(1) > 0 :
        move_absolute_motor_client(1,-7)
    if joystick_object.get_button(4) > 0 :
        move_absolute_motor_client(2,7)
    if joystick_object.get_button(6) > 0 :
        move_absolute_motor_client(2,-7)
    if joystick_object.get_button(5) > 0 :
        move_absolute_motor_client(3,7)
    if joystick_object.get_button(7) > 0 :
        move_absolute_motor_client(3,-7)


    time.sleep(0.1)


joystick_object.quizt()
#destroy objects and clean up
pygame.joystick.quit()

