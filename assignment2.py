#!/usr/bin/env python3


import rclpy
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
from rclpy.serialization import deserialize_message
import math

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

import tf2_ros
import tf_transformations
from geometry_msgs.msg import Quaternion

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# YOUR CODE HERE
# raise NotImplementedError()




# YOUR CODE HERE

bag_file_path = '/home/ibhu/amr_assignmnet/AMR-Assignment/robile_bag_file/robile_bag_file.db3'
scan_topic_name = '/scan'


# Configure storage and converter options.
storage_options = StorageOptions(uri=bag_file_path,storage_id="sqlite3")
converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
# Initialize SequentialReader.

reader = SequentialReader()
reader.open(storage_options, converter_options)

scan_data = []
timestamps = 1697469546
# start_time = 1697469541
# end_time = 1697469546


# Find and store the first set of scan data.

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic.endswith("/scan"):  # Adjust the topic name as needed
        msg = deserialize_message(data, LaserScan)  # Use the appropriate message type


     # Get the angular resolution and angle_min from the LaserScan message
        angular_increment = msg.angle_increment
        angle_min = msg.angle_min

        print("I have acheived the angles")

        for i, range_value in enumerate(msg.ranges):

            if not math.isnan(range_value):  # Skip invalid measurements
                # Convert to Cartesian coordinates
                x = range_value * math.cos(angle_min + i * angular_increment)
                y = range_value * math.sin(angle_min + i * angular_increment)

                scan_data.append((x, y))
                # timestamps.append(timestamp)
                print(timestamps)


# Plot the scan data in Cartesian coordinates
plt.figure()
y_values,x_values = zip(*scan_data)
plt.scatter(x_values, y_values)
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Laser Scan Data (Cartesian)')
plt.colorbar(label='Timestamp')
plt.axis('equal')  # Ensure aspect ratio is equal
plt.show()
plt.savefig("timestamp.png")

# raise NotImplementedError()


