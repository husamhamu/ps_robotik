#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os 
import time
from obstacle_position_estimation import cube_maping

def get_versuch_path(folder_path):
    # Get a list of all directories in the "Versuchen" folder
    directories = [d for d in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, d))]

    # Filter out directories that start with "Versuch"
    versuch_directories = [d for d in directories if d.startswith("Versuch")]

    if len(versuch_directories) == 0:
        # print("No 'Versuch' folder found.")
        return None
    elif len(versuch_directories) ==1:
        latest_versuch_folder = os.path.join(folder_path, 'Versuch')
        return latest_versuch_folder
    else:
        # Extract the numerical values from directory names
        versuch_numbers = []
        for directory in versuch_directories:
            number = directory.lstrip("Versuch")
            try:
                versuch_numbers.append(int(number))
            except ValueError:
                pass

        # Sort the numerical values in descending order
        sorted_numbers = sorted(versuch_numbers, reverse=True)

        # Find the path to the latest "Versuch" folder
        latest_versuch_number = sorted_numbers[0]
        latest_versuch_folder = os.path.join(folder_path, "Versuch{}".format(latest_versuch_number))
        return latest_versuch_folder

def run_detect(pub, transformation_matrix):
    msg = String()
    filename = "/home/weilin/workspace/catkin_ws/src/pose_reader/temp/image{}.jpg".format(rospy.Time.now().to_sec()) #Replace with "temp" folder path
    msg.data = filename
    # Trigger yolo detection in detect.py
    pub.publish(msg)
    # time.sleep(2) do we need to sleep?
    
    # Run object position estimation once we have the label
    folder_path = "/home/weilin/workspace/catkin_ws/src/pose_reader/Versuchen"  # Replace with the actual path to the "Versuchen" folder
    latest_versuch_folder = get_versuch_path(folder_path)
    image_name = os.path.basename(filename)
    label_name = image_name.replace('.jpg', '.txt')
    label_path = os.path.join(latest_versuch_folder, label_name)

    #wait for the label path 
    start_time = time.time()
    timeout = 5  # Timeout in seconds
    interval = 0.2  # Check interval in seconds
    obstacle_positions = []
    obstacle_labels = []
    while True:
        if os.path.isfile(label_path):
            print("image2.txt exists in the directory.")
            obstacle_positions, obstacle_labels = cube_maping(label_path, transformation_matrix)
            return obstacle_positions, obstacle_labels

        elapsed_time = time.time() - start_time
        if elapsed_time >= timeout:
            print("Timed out. Moving on.")
            return obstacle_positions, obstacle_labels

        time.sleep(interval)