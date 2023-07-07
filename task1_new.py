#!/usr/bin/env python

import rospy
import math
from path_planning import RRT
from go_to_point import follow_point,adjust_orientation
import tf
import time
from motors_waveshare import MotorControllerWaveshare
import numpy as np
from std_msgs.msg import String
from yolo_detection import run_detect
from update_obstacles_approach import update_obstacles
import matplotlib.pyplot as plt
from plot_results import plot_path, plot_obstacle_with_labels

def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def is_collision_free(path, obstacles):
    # Check if the path from a node to a point is collision-free
    for point in path:
        for obstacle in obstacles:
            if np.linalg.norm(np.array(point) - np.array(obstacle)) < 0.18: # Make sure the point is at least 0.15m away from obstacls
                return False
    return True

def traverse_points(start_point, end_point, smoothed_path, path, motor):
    smoothed_path.pop(0) # to remove the start point from the point_set 
    while smoothed_path:
        print("traverse_points: start_point", start_point)
        print("traverse_points: point_set", smoothed_path)
        if len(smoothed_path)>= 2:
            distances = [calculate_distance(start_point, point) for point in smoothed_path[0:3]] #determine the closeset point from the first 2 points in the set
            closest_point_index = distances.index(min(distances))
            closest_point = smoothed_path[closest_point_index]

            # Move the robot towards the closest point (assuming it takes some time to move)
            print("traverse_points: closest_point ", closest_point)
            start_point, transformation_matrix, robot_orientation = follow_point(closest_point, motor=motor)

            # Remove the visited point from the set
            for _ in range(closest_point_index + 1):
                smoothed_path.pop(0) #make sure to pop the previous points as well if there is any     

        else:
            #current_point = end_point
            start_point, transformation_matrix, robot_orientation = follow_point(end_point, motor=motor) #it is over we return start_point as a starting point for next goal point

            smoothed_path.pop(0)
            return start_point

    # Add the points to the planned and driven points
    planned_path.append(closest_point)
    driven_path.append(start_point)



def robot_position(listener):
    x = 0
    rate = rospy.Rate(10)
    while x==0:  
        try:
            (trans, rot) = listener.lookupTransform('/map', '/csi://0', rospy.Time(0))
            x, y, z = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("task: failed to look up poistion")
            time.sleep(0.2)
    start_point = (x, y)

    # Construct the transformation matrix
    translation_matrix = tf.transformations.translation_matrix(trans)
    rotation_matrix = tf.transformations.quaternion_matrix(rot)
    transformation_matrix = np.dot(translation_matrix, rotation_matrix)

    return start_point, transformation_matrix

def find_path(start_point, current_goal_point, obstacles):
    # Create an instance of the RRT class and run the algorithm
    rrt = RRT(start_point, current_goal_point, obstacles, distance_from_obstacle=0.18,max_iter=2000)
    if rrt.extend_tree():
        # If a path is found, retrieve the path and plot it
        path1 = rrt.find_path()
        #print('find_path(): path1 ', path1)
        #rrt.plot_path(path1)
        smoothed_path  = rrt.smooth_path(path1)
        print('find_path(): smoothed_path', smoothed_path)
        rrt.plot_smoothed_path(smoothed_path)
        
        return path1, smoothed_path
    else:
        print("Unable to find a path.")
        return None, None

def go_to_start_goal(start_point, goal_point, obstacle_positions, motor):
    path, smoothed_path = find_path(start_point, goal_point, obstacle_positions)
    while smoothed_path:
        print("go_to_start_goal: start_point", start_point)
        print("go_to_start_goal: point_set", smoothed_path)
        if len(smoothed_path)>= 2:
            distances = [calculate_distance(start_point, point) for point in smoothed_path[:2]] #determine the closeset point from the first 2 points in the set
            closest_point_index = distances.index(min(distances))
            closest_point = smoothed_path[closest_point_index]

            # Move the robot towards the closest point (assuming it takes some time to move)
            print("go_to_start_goal: closest_point ", closest_point)
            start_point, transformation_matrix, robot_orientation = follow_point(closest_point, motor=motor)

            # Remove the visited point from the set
            for _ in range(closest_point_index + 1):
                smoothed_path.pop(0) #make sure to pop the previous points as well if there is any 
        else:
            start_point, transformation_matrix, robot_orientation = follow_point(goal_point, motor=motor)
            return
        # Add the points to the planned and driven points
        planned_path.append(closest_point)
        driven_path.append(start_point)

def traverse_goal_points(start_point, goal_point_set):
    motor = MotorControllerWaveshare()
    rospy.init_node('tf_listener_node', anonymous=True)
    listener = tf.TransformListener()
    pub = rospy.Publisher('string_message_topoic', String, queue_size=10)

    # Get robot position 
    start_point, transformation_matrix = robot_position(listener)
    # Get Obstacle positions
    previous_obstacles, previous_labels = [],[]
    
    while goal_point_set:
        distances = [calculate_distance(start_point, point) for point in goal_point_set] #determine the closeset point from the set of points 
        closest_goal_point_index = distances.index(min(distances))
        closest_goal_point = goal_point_set[closest_goal_point_index]

        # adjust orienation
        robot_orientation= adjust_orientation(closest_goal_point, motor)
        new_obstacles, new_labels = run_detect(pub, transformation_matrix)
        # Check if new obstacles are there and update them if so 
        updated_obstacles, updated_labels = update_obstacles(previous_obstacles, new_obstacles, previous_labels, new_labels, start_point, robot_orientation, fov=160)
        previous_obstacles = updated_obstacles # Dont forget to update 
        previous_labels = updated_labels
      
        # Move the robot towards the closest point 
        
        # Find the path to the closest goal point
        print('traverse_goal_points: start_point ', start_point)
        print('traverse_goal_points: closest_goal_point', closest_goal_point)
        path, smoothed_path = find_path(start_point, closest_goal_point, updated_obstacles)
        # Traverse the points to the goal point and obtain updated start point and obstacle positions
        start_point = traverse_points(start_point, closest_goal_point, smoothed_path, path, motor)

        # Remove the visited point from the set
        goal_point_set.pop(closest_goal_point_index)

    robot_orientation= adjust_orientation((0.1, 0.1), motor)
    go_to_start_goal(start_point, (0.1, 0.1), updated_obstacles, motor)
    new_obstacles, new_labels = run_detect(pub, transformation_matrix)
    # Check if new obstacles are there and update them if so 
    updated_obstacles, updated_labels = update_obstacles(previous_obstacles, new_obstacles, previous_labels, new_labels, start_point, robot_orientation, fov=160)
    
    #save planned and driven path and obstacles 
    plot_path(planned_path, title='Planned path')
    plot_path(driven_path, title='Driven path')
    plot_obstacle_with_labels(updated_obstacles, updated_labels, title="Task1")


if __name__ == '__main__':
    # Example usage
    global planned_path, driven_path
    planned_path, driven_path = [], []

    start_point = (0.2, 0.2)
    goal_point_set = [(0.2, 1.2), (1.2, 1.2), (1.2, 0.2)]
    traverse_goal_points(start_point, goal_point_set)
    plt.close("all")