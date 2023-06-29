#!/usr/bin/env python

import rospy
from path_planning import RRT
from go_to_point import follow_point
import tf
import time
from motors_waveshare import MotorControllerWaveshare
import math
import numpy as np
import matplotlib.pyplot as plt

def clockwise_points(obstacle_position, robot_position, radius, num_points):
    theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    x = obstacle_position[0] + radius * np.cos(theta)
    y = obstacle_position[1] + radius * np.sin(theta)

    # Calculate distances between the robot and the points
    distances = np.sqrt((x - robot_position[0])**2 + (y - robot_position[1])**2)

    # Find the index of the closest point
    closest_point_idx = np.argmin(distances)

    # Reorder the points starting from the closest point in a clockwise manner
    x = np.roll(x[::-1], closest_point_idx)
    y = np.roll(y[::-1], closest_point_idx)

    points = list(zip(x, y))
    return points

def anti_clockwise_points(obstacle_position, robot_position, radius, num_points):
    theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    x = obstacle_position[0] + radius * np.cos(theta)
    y = obstacle_position[1] + radius * np.sin(theta)

    # Calculate distances between the robot and the points
    distances = np.sqrt((x - robot_position[0])**2 + (y - robot_position[1])**2)

    # Find the index of the closest point
    closest_point_idx = np.argmin(distances)

    # Reorder the points starting from the closest point in an anti-clockwise manner
    x = np.roll(x, -closest_point_idx)
    y = np.roll(y, -closest_point_idx)

    points = list(zip(x, y))
    return points

def plot_points(robot_position, obstacle_position, points):
    plt.figure(figsize=(6, 6))

    plt.plot([point[0] for point in points], [point[1] for point in points], 'g-')
    plt.plot([point[0] for point in points], [point[1] for point in points], 'go')
    plt.plot(obstacle_position[0], obstacle_position[1], 'ro', label='Obstacle')
    plt.plot(robot_position[0], robot_position[1], 'bo', label='Robot')
    # plt.plot(points[0][0], points[0][1], 'yo', label='second point')

    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Points on a Circle')
    plt.legend()
    plt.grid(True)
    plt.show(block=False)

def add_circle_points(obstacle_position, smoothed_path):
    radius = 0.15
    num_points = 5

    # Calculate distances between obstacle_position and each point in smoothed_path
    distances = [calculate_distance(obstacle_position, point) for point in smoothed_path]

    # Filter distances to exclude points that are within the radius
    distances = [distance for distance in distances if distance > 0.15]

    # Check if there is a point with at least 0.15 distance from obstacle_position
    if min(distances, default=None) is not None:
        # If such a point exists, find the index of the closest point in smoothed_path
        closest_point_index = distances.index(min(distances))

        # Remove the points after the closest_point_index
        for _ in range(closest_point_index+1, len(smoothed_path)):
            smoothed_path.pop(-1)

        # Get the robot_position from the smoothed_path
        robot_position = smoothed_path[closest_point_index]

        # Generate additional points in a clockwise direction around obstacle_position
        points = clockwise_points(obstacle_position, robot_position, radius, num_points)

        # Append the additional points to the smoothed_path
        smoothed_path = smoothed_path + points

        # Plot the robot_position, obstacle_position, and smoothed_path
        plot_points(robot_position, obstacle_position, smoothed_path)

        return smoothed_path
    else:
        # If no point with at least 0.15 distance exists, find the farthest point
        distances = [calculate_distance(obstacle_position, point) for point in smoothed_path]
        closest_point_index = distances.index(max(distances))

        # Get the robot_position from the smoothed_path
        robot_position = smoothed_path[closest_point_index]

        # Generate additional points in a clockwise direction around obstacle_position
        points = clockwise_points(obstacle_position, robot_position, radius, num_points)

        # Plot the robot_position, obstacle_position, and points
        plot_points(robot_position, obstacle_position, points)

        return points
    
def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def traverse_points(start_point, end_point, point_set, motor):
    # current_point = start_point
    point_set.pop(0) # to remove the start point from the point_set 
    while point_set:
        print("traverse_points: start_point", start_point)
        print("traverse_points: point_set", point_set)
        if len(point_set)>= 2:
          distances = [calculate_distance(start_point, point) for point in point_set[0:3]] #determine the closeset point from the first 2 points in the set
          closest_point_index = distances.index(min(distances))
          closest_point = point_set[closest_point_index]
        
        #   current_point = closest_point
        
          # Move the robot towards the closest point (assuming it takes some time to move)
          print("traverse_points: closest_point ", closest_point)
          start_point = follow_point(closest_point, motor=motor)

          # Remove the visited point from the set
          for _ in range(closest_point_index + 1):
            point_set.pop(0) #make sure to pop the previous points as well if there is any 
          
        else:
        #   current_point = end_point
          start_point = follow_point(end_point, motor=motor) #it is over we return start_point as a starting point for next goal point
          point_set.pop(0)
    return start_point

def traverse_goal_points(start_point, goal_point_set):
    motor = MotorControllerWaveshare()
    x = 0
    rospy.init_node('tf_listener_node', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    while x==0:  
        try:
            (trans, rot) = listener.lookupTransform('/map', '/csi://0', rospy.Time(0))
            x, y, z = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("task: failed to look up poistion")
            time.sleep(0.2)

    start_point = (x, y)
    start_point
    while goal_point_set:
        distances = [calculate_distance(start_point, point) for point in goal_point_set] #determine the closeset point from the set of points 
        closest_goal_point_index = distances.index(min(distances))
        closest_goal_point = goal_point_set[closest_goal_point_index]
      
        # Move the robot towards the closest point (assuming it takes some time to move)
        # current_goal_point = closest_goal_point
        # rospy.loginfo("task1: closest_point " + str(closest_goal_point))
        
        # Create an instance of the RRT class and run the algorithm
        arena_size = (1.4, 1.4)
        print('traverse_goal_points: start_point ', start_point)
        print('traverse_goal_points: closest_goal_point', closest_goal_point)
        rrt = RRT(start_point, closest_goal_point, 3, arena_size=arena_size)
        if rrt.extend_tree():
            # If a path is found, retrieve the path and plot it
            path1 = rrt.find_path()
            #smooth path to less points
            smoothed_path  = rrt.smooth_path(path1)
            rrt.plot_smoothed_path(smoothed_path)
            print('traverse_goal_points: smoothed path: ', smoothed_path)
        else:
            rospy.loginfo("task1: unable to find path!")

        obstacle_position = closest_goal_point
        smoothed_path = add_circle_points(obstacle_position, smoothed_path) # Adding points in a clockwise direction around obstacle_position
        start_point = traverse_points(start_point, closest_goal_point, smoothed_path, motor)

        # Remove the visited point from the set
        goal_point_set.pop(closest_goal_point_index) 


if __name__ == '__main__':
    # Example usage
    start_point = (0.2, 0.2)
    goal_point_set = [(0.2, 1.2), (1.2, 1.2), (1.2, 0.2)]
    traverse_goal_points(start_point, goal_point_set)