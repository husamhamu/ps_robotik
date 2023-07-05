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
import numpy as np
from std_msgs.msg import String
import os 
from yolo_detection import run_detect

def follow_point(goal_point, motor):
    pub = rospy.Publisher('string_message_topoic', String, queue_size=10)
    rospy.init_node('tf_listener_node', anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # Rate of 1 Hz
    goal_x = goal_point[0] *100
    goal_y = goal_point[1] *100
    
    # To control the motor wheels speed
    # motor = MotorControllerWaveshare()
    # PID controller gains (adjust these based on your requirements)
    Kp = 0.97
    Ki = 0.0
    Kd = 0.0
    pid_controller = PIDController(Kp, Ki, Kd)
    while not rospy.is_shutdown():
        try:
            # Look up camera pose with respect to arena
            (trans, rot) = listener.lookupTransform('/csi://0', '/map', rospy.Time(0))           
            translation_matrix = tf.transformations.translation_matrix(trans)
            rotation_matrix = tf.transformations.quaternion_matrix(rot)
            #build inverse
            R_inv = rotation_matrix.transpose()
            t_inv = -np.dot(R_inv, translation_matrix)
            x, y, z = t_inv
            T_inv = np.eye(4)
            T_inv[:3, :3] = R_inv
            T_inv[:3, 3] = t_inv
            #build 
            # Create a quaternion from the rotation matrix
            quaternion = tf.transformations.quaternion_from_matrix(R_inv)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

            #determine the left and right speed of wheels
            robot_x, robot_y, robot_orientation = x*100, y*100, yaw

            left_speed, right_speed, robot_orientation = calculate_pid_controller(robot_x, robot_y, robot_orientation, goal_x, goal_y, 1.0)
            
            #set speed of wheels
            motor.set_speed(left_speed, right_speed)

            # If robot is close enough to goal point
            if left_speed == 0.0 and right_speed ==0.0:
                time.sleep(1)
                return (robot_x/100, robot_y/100), T_inv, yaw


            # Look up camera pose with respect to arena
            (trans, rot) = listener.lookupTransform('/map', '/csi://0', rospy.Time(0))
            x, y, z = trans
            # rospy.loginfo("Frame: /your_frame_name, Position: [x: %.2f, y: %.2f, z: %.2f]", x, y, z)

            # Read angle values
            roll, pitch, yaw = tfs.euler_from_quaternion(rot)
            roll_degrees = math.degrees(roll)
            pitch_degrees = math.degrees(pitch)
            yaw_degrees = math.degrees(yaw)
            # rospy.loginfo("Frame: /your_frame_name, Orientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]", roll_degrees, pitch_degrees, yaw_degrees)

            #determine the left and right speed of wheels
            robot_x, robot_y, robot_orientation = x*100, y*100, yaw

            left_speed, right_speed, robot_orientation = calculate_pid_controller(robot_x, robot_y, robot_orientation, goal_x, goal_y, 1.0)

            #set speed of wheels
            motor.set_speed(left_speed, right_speed)

            # If robot is close enough to goal point
            if left_speed == 0.0 and right_speed ==0.0:
                time.sleep(1)
                (trans, rot) = listener.lookupTransform('/map', '/csi://0', rospy.Time(0))
                x, y, z = trans
                # Construct the transformation matrix
                translation_matrix = tf.transformations.translation_matrix(trans)
                rotation_matrix = tf.transformations.quaternion_matrix(rot)
                transformation_matrix = np.dot(translation_matrix, rotation_matrix)
                print("transformation_matrix ", transformation_matrix)
                robot_x, robot_y = x, y
                return (robot_x, robot_y), transformation_matrix, robot_orientation
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform for frame: /your_frame_name")
        
        rate.sleep()

