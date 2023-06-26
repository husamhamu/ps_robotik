#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String



class Motor_Control():
  def __init__(self):
    # setup motor controller
    self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)

    motor_left_ID = 1
    motor_right_ID = 2

    self.motor_left = self.motor_driver.getMotor(motor_left_ID)
    self.motor_right = self.motor_driver.getMotor(motor_right_ID)
 
  # sets motor speed between [-1.0, 1.0]
  def set_speed(self, motor_ID, value):
    max_pwm = 115.0
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

    if motor_ID == 1:
      motor = self.motor_left
    elif motor_ID == 2:
      motor = self.motor_right
    else:
      rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
      return
    
    motor.setSpeed(speed)

    if value < 0:
      motor.run(Adafruit_MotorHAT.FORWARD)
    else:
      motor.run(Adafruit_MotorHAT.BACKWARD)


  # stops all motors
  def all_stop(self):
    self.motor_left.setSpeed(0)
    self.motor_right.setSpeed(0)

    self.motor_left.run(Adafruit_MotorHAT.RELEASE)
    self.motor_right.run(Adafruit_MotorHAT.RELEASE)
