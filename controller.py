#!/usr/bin/env python
import math



class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0

    def calculate_control_signal(self, error, dt):
        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # Calculate control signal
        control_signal = P + I + D

        # Update previous error
        self.prev_error = error

        return control_signal

def speed_sign(number):
   if (number>0):
      return 1
   elif (number<0):
      return -1
   else:
      return 0

def limit_speeds(speed):
    # Limit the wheel speeds to the valid range [-1, 1]
    if speed>0:
      speed = max(min(speed, 0.4), 0.05)
    elif speed <0:
      speed = max(min(speed, -0.05), -0.4)

    return speed

#transforms an angle that starts form the positive y axis
# def transform_angle(angle):
#     transformed_angle = (angle + 90) % 360
#     if transformed_angle > 180:
#         transformed_angle -= 360
#     transformed_angle = math.radians(transformed_angle)
#     return transformed_angle

#transforms an angle that starts form the negative y axis
def transform_angle(angle):
    normalized_angle = (angle + 180) % 360  # Normalize to range 0-360
    transformed_angle = (normalized_angle + 90) % 360  # Shift by -90 degrees
    if transformed_angle < -180:
        transformed_angle += 360
    elif transformed_angle > 180:
        transformed_angle -= 360
    transformed_angle = math.radians(transformed_angle)
    return transformed_angle

def left_or_right(robot_orientation, goal_orienatiojn):
    normalized_angle1 = robot_orientation % 360
    normalized_angle2 = goal_orienatiojn % 360

    raw_difference = normalized_angle2 - normalized_angle1

    if raw_difference < -180:
        raw_difference += 360
    elif raw_difference > 180:
        raw_difference -= 360

    if raw_difference>0:
      return "left"
    else:
      return "right"

def calculate_smallest_angle_difference(robot_orientation, goal_orienatiojn):
    normalized_angle1 = robot_orientation % 360
    normalized_angle2 = goal_orienatiojn % 360

    raw_difference = normalized_angle2 - normalized_angle1

    if raw_difference < -180:
        raw_difference += 360
    elif raw_difference > 180:
        raw_difference -= 360
    return raw_difference

def calculate_pid_controller(robot_x, robot_y, robot_orientation, goal_x, goal_y, max_speed=0.2):
    # Calculate the angle between the robot's orientation and the vector towards the goal
    #transofrm robot_orientation
    max_speed = 0.2
    robot_orientation = transform_angle(math.degrees(robot_orientation))

    angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)

    # Calculate the orientation error
    # orientation_error = angle_to_goal - robot_orientation
    orientation_error = math.radians(abs(calculate_smallest_angle_difference(math.degrees(robot_orientation), math.degrees(angle_to_goal))))
    #scale down the orientation error 
    orientation_error = (orientation_error - (-3.14)) / (3.14 - (-3.14)) * (0.2 - (-0.2)) + (-0.2)

    # Calculate the distance to the goal
    distance_to_goal = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
    if abs(distance_to_goal) < 4.0:
      return 0.0, 0.0, robot_orientation

    # PID controller gains (adjust these based on your requirements)
    Kp = 1.0
    Ki = 0.0
    Kd = 0.0

    pid_controller = PIDController(Kp, Ki, Kd)

    # Calculate the control signal based on the orientation error
    control_signal = pid_controller.calculate_control_signal(orientation_error, dt=0.2)

    # Calculate the desired robot speed based on the distance to the goal
    desired_speed = min(distance_to_goal, max_speed)
    #print("control_signal", control_signal)
    # Calculate the left and right wheel speeds
    if robot_orientation != angle_to_goal:
      motor_id = left_or_right(math.degrees(robot_orientation), math.degrees(angle_to_goal))
      #print("motor_id", motor_id)
      if motor_id == "left":
        left_speed = desired_speed - control_signal
        right_speed = desired_speed
      elif motor_id == "right":
        left_speed = desired_speed
        right_speed = desired_speed - control_signal
    else:
      left_speed = desired_speed - control_signal
      right_speed = desired_speed + control_signal

    # limit speed
    # limited_left_speed = (left_speed - (-0.8)) / (0.8 - (-0.8)) * (0.4 - (-0.4)) + (-0.4)
    # limited_right_speed = (right_speed - (-0.8)) / (0.8 - (-0.8)) * (0.4 - (-0.4)) + (-0.4)
    limited_left_speed = limit_speeds(left_speed)
    limited_right_speed = limit_speeds(right_speed)

    return limited_left_speed, limited_right_speed, robot_orientation

