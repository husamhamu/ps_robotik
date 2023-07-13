#!/usr/bin/env python

import math
import cv2
import numpy as np

import matplotlib.pyplot as plt
import numpy as np

import os 




def precpective_transform(x_cam, y_cam):
    # x_cam, y_cam = 682.49984, 464.50008  #input here

    # we set four blocks in the coordinate, and record how they look like in camera
    pts1 = np.float32([[188, 584], [990, 579], [979, 368],[503, 379]])

    # the coordinate of four points in real world coordinate system
    pts2 = np.float32([[-20, 20], [10, 20], [40, 90],[-30, 80]])

    # perspection matrix
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    # this is the point coordinate inported from model(here use several examples)
    point = np.array([[[x_cam, y_cam]]])


    # transform
    transformed_points = cv2.perspectiveTransform(point.reshape(-1, 1, 2).astype(np.float32), matrix)
    #transformed_point = cv2.perspectiveTransform(point, matrix)
    x_real = transformed_points[0][0][0]
    z_real = transformed_points[0][0][1]
    y_real = 10

    x_real, y_real, z_real = x_real/100, y_real/100, z_real/100
    return x_real, y_real, z_real


def estimate_position(taransformation_matrix, box_center_x, box_center_y, average_height_pixels=0.0):

    x, y, z = precpective_transform(box_center_x, box_center_y)

    # Transform point to arena coordinates
    homgenous_point = [x, y, z, 1]
    homgenous_transformed = np.dot(taransformation_matrix, homgenous_point)
    x, y, z, w = homgenous_transformed
    arena_x = x / w
    arena_y = y / w
    arena_z = z / w
    print('arena_x', arena_x)
    print('arena_y', arena_y)
    print()
    return arena_x, arena_y, arena_z

def plot_points(points):
    plt.figure(figsize=(6, 6))
    plt.plot([point[0] for point in points], [point[1] for point in points], 'go')
    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Obstacle points')
    plt.legend()
    plt.grid(True)
    plt.show(block=False)

def plot_obstacle(obstacle_labels, obstacle_positions):
    fig, ax = plt.subplots()

    # Plot the obstacles
    for label, position in zip(obstacle_labels, obstacle_positions):
        x, y = position
        ax.plot(x, y, 'bo')  # Plot as blue circles
        ax.text(x, y, label, ha='center', va='bottom')  # Add the label as text
    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.show(block=False)

def cube_maping(label_path, taransformation_matrix, task="task1"):
    labels = ["ball", "redcube", "bluecube", "yellowcube", "purplecube", "orangecube", "greencube"]
    obstacle_labels = []
    obstacle_positions = []
    # read bounding box
    with open(label_path, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        data = line.split()
        label_id = int(data[0])
        box_center_x, box_center_y, width, height = map(float, data[1:5])
        confidence_level = float(data[5])
        
        if task == "task2" and labels[label_id] !="ball":
            continue
        
        # put a threshold on confidence level
        if confidence_level > 0.55:
            print('label: ', labels[label_id])
            
            # determine object center in the image
            image_height, image_width = 720, 1280
            box_center_x = box_center_x* image_width
            box_center_y = box_center_y* image_height
            
            # estimate the position of the object
            x, y, z = estimate_position(taransformation_matrix, box_center_x, box_center_y)

            # if position if valid
            if not (x==0.0 and y==0.0 and z==0.0) and not (x>1.5 or y>1.5):
                obstacle_positions.append((x, y))
                obstacle_labels.append(labels[label_id])

    # plot_points(obstacle_positions)
    # plot_obstacle(obstacle_labels, obstacle_positions)
    return obstacle_positions, obstacle_labels
