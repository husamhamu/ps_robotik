#!/usr/bin/env python

import math
import copy

import matplotlib.pyplot as plt

def plot_obstacles(previous_obstacles, new_obstacles, updated_obstacles, updated_labels):
    # Extract x and y coordinates from the obstacles lists
    prev_x, prev_y = zip(*previous_obstacles)
    new_x, new_y = zip(*new_obstacles)
    updated_x, updated_y = zip(*updated_obstacles)

    # Create a new figure
    plt.figure()

    # Plot the obstacle positions from each list
    plt.scatter(prev_x, prev_y, color='red', label='Previous Obstacles', s=100)
    plt.scatter(new_x, new_y, color='blue', label='New Obstacles', s=80)
    plt.scatter(updated_x, updated_y, color='green', label='Updated Obstacles')

    for obstacle, label in zip(updated_obstacles, updated_labels):
        plt.text(obstacle[0], obstacle[1], label, ha='center', va='bottom')  # Display the label near the obstacle

    # Add labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    # Show the plot
    plt.show(block=False)



def update_obstacles(previous_obstacles, new_obstacles, previous_labels, new_labels, camera_position, camera_orientation, fov=160):
    # Calculate FOV boundaries
    camera_orientation = math.degrees(camera_orientation)
    fov_half_angle = math.radians(fov / 2)
    fov_left_boundary = math.radians(camera_orientation) - fov_half_angle
    fov_right_boundary = math.radians(camera_orientation) + fov_half_angle

    # Step 1: Initialize the updated obstacle positions list
    updated_obstacles = []
    updated_labels = []

    # Step 2-5: Identify previous obstacles in the FOV
    obstacles_in_fov = []
    labels_in_fov = []
    for obstacle, label in zip(previous_obstacles, previous_labels):
        angle = math.atan2(obstacle[1] - camera_position[1], obstacle[0] - camera_position[0])
        angle = math.degrees(angle)
        angle_diff = (angle - camera_orientation + 180) % 360 - 180
        if -fov / 2 <= angle_diff <= fov / 2:
            obstacles_in_fov.append(obstacle)
            labels_in_fov.append(label)
    print('obstacles_in_fov', obstacles_in_fov)

    copy_obstacles_in_fov = copy.deepcopy(obstacles_in_fov)
    copy_new_obstacles = copy.deepcopy(new_obstacles)

    obstacles_in_fov_used = []
    new_obstacles_used = []
    # Step 6: Check if the number of obstacles in FOV is less than the number of new obstacles
    # Step 7-12: Update obstacle positions for close obstacles
    for new_obstacle, new_label in zip(new_obstacles, new_labels):
        min_distance = float('inf')
        closest_obstacle = None
        closest_label = None
        for obstacle in obstacles_in_fov:
            distance = math.sqrt((new_obstacle[0] - obstacle[0]) ** 2 + (new_obstacle[1] - obstacle[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_obstacle = obstacle
                closest_label = label

        # Step 8: Calculate the updated position as the average for close obstacles
        if closest_obstacle is not None:
            updated_position = ((new_obstacle[0] + closest_obstacle[0]) / 2, (new_obstacle[1] + closest_obstacle[1]) / 2) # Step 8: Calculate the updated position as the average
            obstacles_in_fov.remove(closest_obstacle) # Step 9: Remove the closest obstacle to avoid duplicate updating
            # labels_in_fov.remove(closest_label)
            updated_obstacles.append(updated_position) # Step 10: Append the updated obstacle position to the updated list
            updated_labels.append(new_label)
            #add to used 
            obstacles_in_fov_used.append(closest_obstacle)
            new_obstacles_used.append(new_obstacle)

    # Step 13: Append the remaining new obstacles and previosu obstacles as updated obstacles that did not find a match 
    updated_obstacles.extend(obstacle for obstacle in new_obstacles if obstacle not in new_obstacles_used)
    updated_labels.extend(label for label, obstacle in zip(new_labels, new_obstacles) if obstacle not in new_obstacles_used) #do you want to add previosu points that are in FOV but not used?
    updated_obstacles.extend(obstacle for obstacle in previous_obstacles if obstacle not in obstacles_in_fov_used)
    updated_labels.extend(label for label, obstacle in zip(previous_labels, previous_obstacles) if obstacle not in obstacles_in_fov_used)

    plot_obstacles(previous_obstacles, copy_new_obstacles, updated_obstacles, updated_labels)
    return updated_obstacles, updated_labels