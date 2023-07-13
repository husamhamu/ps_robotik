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
from path_planning import RRT
from std_msgs.msg import String
from yolo_detection import run_detect
from update_obstacles_approach import update_obstacles
from plot_results import plot_path, plot_obstacle_with_labels
import sys

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


def add_circle_points(obstacle_position, smoothed_path, closest_goal_point_label):
    radius = 0.18
    num_points = 5

    #smoothed_path.pop(-1)
    # Calculate distances between obstacle_position and each point in smoothed_path
    distances = []
    distances = [calculate_distance(obstacle_position, point) for point in smoothed_path]

    # Filter distances to exclude points that are within the radius
    distances = [distance for distance in distances if distance > 0.2]

    # Check if there is a point with at least 0.15 distance from obstacle_position
    if len(distances) >0:
        if min(distances):
            # If such a point exists, find the index of the closest point in smoothed_path
            closest_point_index = distances.index(min(distances))

            # Remove the points after the closest_point_index
            for _ in range(closest_point_index+1, len(smoothed_path)):
                smoothed_path.pop(-1)

            # Get the robot_position from the smoothed_path
            robot_position = smoothed_path[closest_point_index]

            # Generate additional points in a clockwise direction around obstacle_position
            if closest_goal_point_label=="red":
                points = clockwise_points(obstacle_position, robot_position, radius, num_points)
            else:
                points = anti_clockwise_points(obstacle_position, robot_position, radius, num_points)

            # Append the additional points to the smoothed_path
            smoothed_path = smoothed_path + points
            smoothed_path.append(points[0])

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
            if closest_goal_point_label=="red":
                points = clockwise_points(obstacle_position, robot_position, radius, num_points)
            else:
                points = anti_clockwise_points(obstacle_position, robot_position, radius, num_points)
            points.append(points[0])

            # Plot the robot_position, obstacle_position, and points
            plot_points(robot_position, obstacle_position, points)
    
        return points
    
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

def traverse_points(start_point, end_point, smoothed_path, path, motor, pub, previous_obstacles, previous_labels):
    #smoothed_path.pop(0) # to remove the start point from the point_set 
    end_point = smoothed_path[-1]
    updated = False
    while smoothed_path:
        print("traverse_points: start_point", start_point)
        print("traverse_points: point_set", smoothed_path)
        if len(smoothed_path)>= 2:
            distances = [calculate_distance(start_point, point) for point in smoothed_path[:2]] #determine the closeset point from the first 2 points in the set
            closest_point_index = distances.index(min(distances))
            closest_point = smoothed_path[closest_point_index]

            # Move the robot towards the closest point (assuming it takes some time to move)
            print("traverse_points: closest_point ", closest_point)
            start_point, transformation_matrix, robot_orientation = follow_point(closest_point, motor=motor)

            # Remove the visited point from the set
            for _ in range(closest_point_index + 1):
                smoothed_path.pop(0) #make sure to pop the previous points as well if there is any 


            #new_obstacles, new_labels = run_detect(pub, transformation_matrix, task= "task2")
            new_obstacles, new_labels = [], []
            # Check if new obstacles are there and update them if so 
            if len(new_obstacles) != 0:
                updated = True
                updated_obstacles, updated_labels = update_obstacles(previous_obstacles, new_obstacles, previous_labels, new_labels, start_point, robot_orientation, fov=160)
                previous_obstacles = updated_obstacles # Dont forget to update 
                previous_labels = updated_labels

                # Check if path is still valid with the new update
                collision_free = is_collision_free(path, updated_obstacles)
                if not collision_free:
                    path, smoothed_path = find_path(start_point, end_point, updated_obstacles) #if path not valid update it
                    smoothed_path.pop(0) # to remove the start point from the point set
                

        else:
            #current_point = end_point
            start_point, transformation_matrix, robot_orientation = follow_point(end_point, motor=motor) #it is over we return start_point as a starting point for next goal point
            #new_obstacles, new_labels = run_detect(pub, transformation_matrix, task= "task2")
            new_obstacles, new_labels = [], []
                        # Check if new obstacles are there and update them if so 
            if len(new_obstacles) != 0:
                updated = True
                updated_obstacles, updated_labels = update_obstacles(previous_obstacles, new_obstacles, previous_labels, new_labels, start_point, robot_orientation, fov=160)
                previous_obstacles = updated_obstacles # Dont forget to update 
                previous_labels = updated_labels
            smoothed_path.pop(0)

        # Add the points to the planned and driven points
        planned_path.append(closest_point)
        driven_path.append(start_point)
    if updated:
        return start_point, updated_obstacles, updated_labels
    else:
        return start_point, previous_obstacles, previous_labels


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
    rrt = RRT(start_point, current_goal_point, obstacles, distance_from_obstacle=0.15, max_iter=1000)
    if rrt.extend_tree():
        # If a path is found, retrieve the path and plot it
        path1 = rrt.find_path()
        #print('find_path(): path1 ', path1)
        #rrt.plot_path(path1)
        smoothed_path  = rrt.smooth_path(path1)
        print('find_path(): smoothed_path', smoothed_path)
        #rrt.plot_smoothed_path(smoothed_path)
        
        return path1, smoothed_path
    else:
        print("Unable to find a path.")
        return None, None

def go_to_start_goal(start_point, goal_point, obstacle_positions, motor):
    path, smoothed_path = find_path(start_point, goal_point, obstacle_positions)
    while len(smoothed_path)>0:
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

def traverse_goal_points(start_point, goal_point_set, labels):
    motor = MotorControllerWaveshare()
    rospy.init_node('tf_listener_node', anonymous=True)
    listener = tf.TransformListener()
    pub = rospy.Publisher('string_message_topoic', String, queue_size=10)

    # Get robot position 
    start_point, transformation_matrix = robot_position(listener)
    # Get Obstacle positions
    #obstacle_positions, obstacle_labels = run_detect(pub, transformation_matrix, task= "task2")
    obstacle_positions, obstacle_labels = [], []

    while goal_point_set:
        distances = [calculate_distance(start_point, point) for point in goal_point_set] #determine the closeset point from the set of points 
        closest_goal_point_index = distances.index(min(distances))
        closest_goal_point = goal_point_set[closest_goal_point_index]
        closest_goal_point_label = labels[closest_goal_point_index]
        # Move the robot towards the closest point 
        
        # Find the path to the closest goal point
        print('traverse_goal_points: start_point ', start_point)
        print('traverse_goal_points: closest_goal_point', closest_goal_point)
        path, smoothed_path = find_path(start_point, closest_goal_point, obstacle_positions)

        # Traverse the points to the goal point and obtain updated start point and obstacle positions
        smoothed_path.pop(-1) #remove the goal point
        smoothed_path = add_circle_points(closest_goal_point, smoothed_path, closest_goal_point_label) # Adding points in a clockwise direction around obstacle_position
        start_point, obstacle_positions, obstacle_labels = traverse_points(start_point, closest_goal_point, smoothed_path, path, motor, pub, obstacle_positions, obstacle_labels)
        
        #append the goal point as an obstacle once you are done
        obstacle_positions.append(closest_goal_point) 
        obstacle_labels.append('goal_point')
        # Remove the visited point from the set
        goal_point_set.pop(closest_goal_point_index) 
        labels.pop(closest_goal_point_index)
    go_to_start_goal(start_point, (0.1, 0.1), obstacle_positions, motor)

    #save planned and driven path
    plot_path(planned_path, title='Planned path')
    plot_path(driven_path, title='Driven path')
    plot_obstacle_with_labels(obstacle_positions, obstacle_labels, title="Task1")


if __name__ == '__main__':
    # Example usage
    global planned_path, driven_path
    planned_path, driven_path = [], []

    start_point = (2.2, 0.2)
    goal_point_set = [(0.4, 0.5), (0.4, 1.0), (0.8, 0.5), (0.8, 1.0), (1.2, 0.5), (1.2, 1.0)]

    # Retrieve the command-line arguments
    args = rospy.myargv(argv=sys.argv)

    # Check if the correct number of arguments is provided
    if len(args) != 19:
        rospy.logerr("Incorrect number of arguments. Please provide 6 points with their labels.")
        sys.exit(1)

    # Extract the values from the command-line arguments
    points = []
    labels = []
    for i in range(1, 19, 3):
        point_x = float(args[i])
        point_y = float(args[i+1])
        label = args[i+2]
        points.append((point_x, point_y))
        labels.append(label)
    print(points)
    print(labels)
    traverse_goal_points(start_point, points, labels)
    print('planned_path', planned_path)
    print('driven_path', driven_path)
    plt.close("all")
