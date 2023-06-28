#!/usr/bin/env python

import rospy
import tf
from tf import transformations as tfs
import math
import time
from geometry_msgs.msg import Twist  #geometry_msgs/Twist
from controller import PIDController, calculate_pid_controller
from motor_control import Motor_Control
from motors_waveshare import MotorControllerWaveshare


def follow_point(goal_point):
    rospy.init_node('tf_listener_node', anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # Rate of 1 Hz
    goal_x = goal_point[0] *100
    goal_y = goal_point[1] *100
    
    # To control the motor wheels speed
    motor = MotorControllerWaveshare()
    # PID controller gains (adjust these based on your requirements)
    Kp = 1.0
    Ki = 0.0
    Kd = 0.0
    pid_controller = PIDController(Kp, Ki, Kd)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/csi://0', rospy.Time(0))
            x, y, z = trans
            # rospy.loginfo("Frame: /your_frame_name, Position: [x: %.2f, y: %.2f, z: %.2f]", x, y, z)

            roll, pitch, yaw = tfs.euler_from_quaternion(rot)
            roll_degrees = math.degrees(roll)
            pitch_degrees = math.degrees(pitch)
            yaw_degrees = math.degrees(yaw)
            
            # rospy.loginfo("Frame: /your_frame_name, Orientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]", roll, pitch, yaw)
            # rospy.loginfo("Frame: /your_frame_name, Orientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]", roll_degrees, pitch_degrees, yaw_degrees)

            #determine the left and right speed of wheels
            robot_x, robot_y, robot_orientation = x*100, y*100, math.radians(yaw_degrees)
            #rospy.loginfo("Frame: /your_frame_name, Position: [robot_x: %.2f, robot_y: %.2f, robot_orientation: %.2f]", robot_x, robot_y,yaw_degrees)
            print("follow_point(): goal_point", goal_point)
            print("follow_point(): robot_point", str((x, y)))
            left_speed, right_speed = calculate_pid_controller(robot_x, robot_y, robot_orientation, goal_x, goal_y, 1.0)
            #rospy.loginfo("posereader: , : [left_speed: %.2f, right_speed: %.2f]", left_speed, right_speed) 
            print("follow_point(): left_speed, right_speed ", left_speed, right_speed)
            #set speed of wheels
            motor.set_speed(left_speed, right_speed)

            # Calculate the distance to the goal
            if left_speed == 0.0 and right_speed ==0.0:
                return (robot_x/100, robot_y/100)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform for frame: /your_frame_name")
        
        rate.sleep()

