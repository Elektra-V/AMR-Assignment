#!/usr/bin/env python3

import rclpy
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
from rclpy.serialization import deserialize_message
import math
from scipy import ndimage
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
import time

def plot_scan_data(scan_data, timestamps, title):
    plt.scatter(scan_data[0], scan_data[1], c=timestamps, cmap='viridis')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title(title)
    plt.colorbar(label='Timestamp')
    plt.axis('equal')  # Ensure aspect ratio is equal
    # plt.show()

bag_file_path = '/home/ibhu/amr_assignmnet/robile_bag_file/robile_bag_file.db3'
scan_topic_name = '/scan'

desired_duration = 0.3
end_time = time.time() + desired_duration
elapsed_time = 0.0  # Initialize the elapsed time

# Configure storage and converter options.
storage_options = StorageOptions(uri=bag_file_path, storage_id="sqlite3")
converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")

# Initialize SequentialReader.
reader = SequentialReader()
reader.open(storage_options, converter_options)

scan_data = [[], []]
timestamps = []

# Find and store the first set of scan data.
while reader.has_next() and elapsed_time < desired_duration:
    topic, data, timestamp = reader.read_next()
    if topic.endswith("/scan"):
        msg = deserialize_message(data, LaserScan)
        angular_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i, range_value in enumerate(msg.ranges):
            if not math.isnan(range_value):
                x = range_value * math.cos(angle_min + i * angular_increment)
                y = range_value * math.sin(angle_min + i * angular_increment)

                scan_data[0].append(x)
                scan_data[1].append(y)
                timestamps.append(timestamp)

    # Update the elapsed time
    elapsed_time = time.time() - end_time




plt.figure(1)
plot_scan_data(scan_data, timestamps, 'Laser Scan Data (Cartesian)')


window_size = 3 # Adjust the window size as needed
filtered_x = ndimage.uniform_filter(scan_data[0], window_size)
filtered_y = ndimage.uniform_filter(scan_data[1], window_size)



# Plot the second figure
plt.figure(2)
plot_scan_data([filtered_x, filtered_y], timestamps, 'Laser Scan Data with Mean Filter (Cartesian)')

# Display both figures
plt.show()