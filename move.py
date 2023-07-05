#!/usr/bin/env python

import rospy
import math
import time

from motors_waveshare import MotorControllerWaveshare


def tf_listener():

    rate = rospy.Rate(0.1)  # Rate of 1 Hz
    goal_x = 70.0
    goal_y = 70.0
    motor = MotorControllerWaveshare()
    motor.set_speed(0, 0.15)


if __name__ == '__main__':
    try:
        tf_listener()
    except rospy.ROSInterruptException:
        pass

