#!/usr/bin/env python

import math
import cv2
import numpy as np

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import os 

def calculate_edge_length(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    # Apply the Sobel operator in the x-direction
    sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)

    # Take the absolute value of the derivative
    sobel_x = np.abs(sobel_x)

    # Normalize the result to a range of 0-255
    sobel_x = cv2.normalize(sobel_x, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    # Apply binary thresholding to obtain a binary image
    _, binary = cv2.threshold(sobel_x, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Perform morphological operations (optional)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(binary, kernel, iterations=1)

    # Find contours in the binary image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calculate the total length of the contours
    if len(contours) == 0:
      return 0
    else:
      # Sort the contours based on their bounding box height
      sorted_contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[3], reverse=True)

      # Get the two contours with the largest height
      top_two_contours = sorted_contours[:2]

      # Calculate the average height of the contours
      average_height = sum(cv2.boundingRect(c)[3] for c in top_two_contours) / len(top_two_contours)

    plt.imshow(edges)
    plt.axis('off')  # Disable axis
    plt.show(block=False)

    return average_height

def calculate_distance(object_height_pixels, object_height=3, focal_length=887.411967):
    # Calculate the distance using the formula: distance = (object_height * focal_length) / image_height
    distance = (object_height * focal_length) / object_height_pixels

    return distance

def get_angle_to_image_center(pixel_x):
    image_width= 1280
    image_high= 720

    #Calculate the pixel coordinates of the image center
    image_center_x = image_width / 2
    angle_degrees = 79.5

    angle_radians = math.radians(angle_degrees)
    # Calculate the pixel offset from the object to the center of the image
    delta_x = pixel_x - image_center_x

    # Calculate the angle of entropy (using the arctangent function)
    angle_to_image_center = angle_radians * delta_x/640

    return angle_to_image_center


def calculate_horizontal_projection_distance(camera_height, hypotenuse_length):
    # Calculate the horizontal projection length
    horizontal_projection_distance = math.sqrt(hypotenuse_length**2 - camera_height**2)

    return horizontal_projection_distance

def determine_position(angle, horizontal_projection, camera_height):
    # Calculate the cosine value
    cosine_value = math.cos(angle)
    # Calculating sine values
    sine_value = math.sin(angle)

    x=horizontal_projection * cosine_value
    y=camera_height + 1.5
    z=horizontal_projection * sine_value
    return x, y, z


def estimate_position(cropped_image, camera_position, camera_orientation, box_center_x):

    #estimate averatge height of edges
    average_height_pixels = calculate_edge_length(cropped_image)
    print('average_height_pixels', average_height_pixels)

    # Calculate the position of the object with respect to the arena coordinates
    hypotenuse_length = calculate_distance(average_height_pixels)  # Get the distance to the object=
    print("distance", hypotenuse_length)

    # Calculate the horizontal projection length
    camera_height = 8.5
    horizontal_projection = calculate_horizontal_projection_distance(camera_height, hypotenuse_length)

    # Calculate the angle of entrapment
    angle = get_angle_to_image_center(box_center_x)
    x, y, z = determine_position(angle, horizontal_projection, camera_height)

    TODO: transform the points to arena coordiante
    return x, y, z

def plot_points(points):
    plt.figure(figsize=(6, 6))
    plt.plot([point[0] for point in points], [point[1] for point in points], 'go')
    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Points on a Circle')
    plt.legend()
    plt.grid(True)
    plt.show(block=False)

def cube_maping(label_path, taransformation_matrix):
    obstacle_positions = []
    # read bounding box
    with open(label_path, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        data = line.split()
        label_id = int(data[0])
        box_center_x, box_center_y, width, height = map(float, data[1:5])
        confidence_level = float(data[5])
        
        if confidence_level > 0.6:
            image_name = os.path.basename(label_path)
            image_name = image_name.replace('.txt', '.jpg')
            image_path =  os.path.join('/home/weilin/workspace/catkin_ws/src/pose_reader/temp', image_name)
            image = cv2.imread(image_path)
            
            image_height, image_width = image.shape[:2]
            
            # Calculate top-left and bottom-right coordinates of the bounding box
            x = int((box_center_x - width / 2) * image_width)
            y = int((box_center_y - height / 2) * image_height)
            right = int((box_center_x + width / 2) * image_width)
            bottom = int((box_center_y + height / 2) * image_height)
            
            # Crop the image based on the calculated coordinates
            cropped_image = image[y:bottom, x:right]
            plt.imshow(cropped_image)
            plt.axis('off')  # Disable axis
            plt.show(block=False)

            #estimate position of each object
            x, y, z = estimate_position(cropped_image, taransformation_matrix, box_center_x)
            obstacle_positions.append((x, y))
            print('arena_center_x', x)
            print('arena_center_y', y)

    plot_points(obstacle_positions)

# Camera position and orientation
# camera_position = (0.2, 0.2)  # Example camera position in meters (x, y)
# camera_orientation = -math.radians(45)  # Example camera orientation

# arena_size = (1.4, 1.4)  # Size of the arena in meters (width, height)

# # Load the input image
# image = cv2.imread('/content/20230609-094436.jpg')

# # Define a list of bounding boxes [x, y, width, height]

# bounding_boxes = [[576, 401, 617 - 576, 434 - 401], [450, 572, 584 - 450, 708 - 584]]

# cube_maping(image, bounding_boxes, camera_position, camera_orientation, arena_size)

# cv2.destroyAllWindows()