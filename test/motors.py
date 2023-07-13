#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import Twist  #geometry_msgs/Twist


class MotorController(object):  #(Node):
    """
    Abstract motor controller base node for supporting different JetBots.
    Can be extended to support any diff drive by overriding set_speed(),
    or any node that subscribes to the /cmd_vel Twist message.
    """
    def __init__(self):
        #rospy.init_node('motors') # namespace='jetbot
        #sub = rospy.Subscriber('cmd_vel', Twist, self.twist_listener)
        
        #rospy.set_param("left_trim", 0.0)
        #rospy.set_param("right_trim", 0.0)
        #rospy.set_param("max_pwm", 255)
        #rospy.set_param("max_rpm", 200)  # https://www.adafruit.com/product/3777
        #rospy.set_param("wheel_separation", 0.1016)   # 4 inches
        #rospy.set_param("wheel_diameter", 0.060325)  # 2 3/8 inches
        
        self.left_trim = 0.0
        self.right_trim = 0.0
        self.max_pwm = 100
        self.max_rpm = 200
        self.wheel_separation = 0.1016
        self.wheel_diameter = 0.060
         
        self.last_x = -999
        self.last_rot = -999
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        Override this function for other motor controller setups.
        Should take into account left_trim, right_trim, and max_pwm.
        """
        raise NotImplementedError('MotorController subclasses should implement set_speed()')

    def stop(self):
        self.set_speed(0,0)

    def twist_listener(self, msg):
        x = msg.linear.x
        rot = msg.angular.z
        
        if x == self.last_x and rot == self.last_rot:
            return
            
        self.last_x = x
        self.last_rot = rot
        
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/231a7219b36b8a6cdd100b59f66a3df2955df787/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L331
        left = x - rot * self.wheel_separation / 2.0
        right = x + rot * self.wheel_separation / 2.0
        
        # convert velocities to [-1,1]
        max_speed = (self.max_rpm / 60.0) * 2.0 * math.pi * (self.wheel_diameter * 0.5)

        left = max(min(left, max_speed), -max_speed) / max_speed
        right = max(min(right, max_speed), -max_speed) / max_speed
        
        #self.get_logger().info(f"x={x:.03f} rotation={rot:.03f} -> left={left:.03f} right={right:.03f}  (max_speed={max_speed:.03f} m/s)")
        self.set_speed(left, right)

    
if __name__ == '__main__':
    raise NotImplementedError("motors.py shouldn't be instantiated directly - instead use motors_nvidia.py, motors_waveshare.py, ect")
    
    
	

