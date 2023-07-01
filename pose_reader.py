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
import subprocess
from std_msgs.msg import String
from threading import Thread

def run_detect():
    subprocess.call(['python3.7', '/home/weilin/workspace/catkin_ws/src/pose_reader/detect.py'])

def tf_listener():
    pub = rospy.Publisher('string_message_topoic', String, queue_size=10)
    rospy.init_node('tf_listener_node', anonymous=True)
    #rospy.spin()
    listener = tf.TransformListener()
    print('started')
    #detect_thread = Thread(target=run_detect)
    #detect_thread.start()
    print('stopped')

    rate = rospy.Rate(0.1)  # Rate of 1 Hz
    goal_x = 70.0
    goal_y = 70.0
    motor = MotorControllerWaveshare()
    while not rospy.is_shutdown():
        try:
            msg = String()
            filename = "/home/weilin/workspace/catkin_ws/src/pose_reader/temp/image{}.jpg".format(rospy.Time.now().to_sec())
            msg.data = filename
            pub.publish(msg)
            print('published')
            (trans, rot) = listener.lookupTransform('/map', '/csi://0', rospy.Time(0))
            x, y, z = trans
            # rospy.loginfo("Frame: /your_frame_name, Position: [x: %.2f, y: %.2f, z: %.2f]", x, y, z)

            # Construct the transformation matrix
            translation_matrix = tf.transformations.translation_matrix(trans)
            rotation_matrix = tf.transformations.quaternion_matrix(rot)
            transformation_matrix = np.dot(translation_matrix, rotation_matrix)
            #print("transformation_matrix ", transformation_matrix)

            roll, pitch, yaw = tfs.euler_from_quaternion(rot)
            roll_degrees = math.degrees(roll)
            pitch_degrees = math.degrees(pitch)
            yaw_degrees = math.degrees(yaw)
            
            # rospy.loginfo("Frame: /your_frame_name, Orientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]", roll, pitch, yaw)
            # rospy.loginfo("Frame: /your_frame_name, Orientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]", roll_degrees, pitch_degrees, yaw_degrees)

            #determine the left and right speed of wheels
            robot_x, robot_y, robot_orientation = x*100, y*100, math.radians(yaw_degrees)
            #rospy.loginfo("Frame: /your_frame_name, Position: [robot_x: %.2f, robot_y: %.2f, robot_orientation: %.2f]", robot_x, robot_y,yaw_degrees)
            left_speed, right_speed = calculate_pid_controller(robot_x, robot_y, robot_orientation, goal_x, goal_y, 1.0)
            #rospy.loginfo("posereader: , : [left_speed: %.2f, right_speed: %.2f]", left_speed, right_speed) 
            #set speed of wheels
            #motor.set_speed(left_speed, right_speed)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform for frame: /your_frame_name")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_listener()
    except rospy.ROSInterruptException:
        pass

