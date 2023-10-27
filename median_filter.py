#!/usr/bin/env python3


import rclpy
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
from rclpy.serialization import deserialize_message
import math
from scipy.signal import medfilt


from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

import tf2_ros
import tf_transformations
from geometry_msgs.msg import Quaternion
import time


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# YOUR CODE HERE
# raise NotImplementedError()




# YOUR CODE HERE

bag_file_path = '/home/ibhu/amr_assignmnet/robile_bag_file/robile_bag_file.db3'
scan_topic_name = '/scan'

desired_duration = 1.0
end_time = time.time() + desired_duration
elapsed_time = 0.0  # Initialize the elapsed time

# Configure storage and converter options.
storage_options = StorageOptions(uri=bag_file_path,storage_id="sqlite3")
converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
# Initialize SequentialReader.

reader = SequentialReader()
reader.open(storage_options, converter_options)

scan_data = []
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

                scan_data.append((x, y))
                timestamps.append(timestamp)

    # Update the elapsed time
    elapsed_time = time.time() - end_time
                # print(timestamps)


# Plot the scan data in Cartesian coordinates
plt.figure(1)
y_values,x_values = zip(*scan_data)
plt.scatter(x_values, y_values, c=timestamps, cmap='viridis')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Laser Scan Data (Cartesian)')
plt.colorbar(label='Timestamp')
plt.axis('equal')  # Ensure aspect ratio is equal
plt.show()
# plt.savefig("timestamp.png")



window_size = 11  # Adjust the window size as needed
filtered_x = medfilt(x_values, kernel_size=window_size)
filtered_y = medfilt(y_values, kernel_size=window_size)


plt.figure(2)
plt.scatter(filtered_x, filtered_y, c=timestamps, cmap='viridis')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Laser Scan Data with Median Filter (Cartesian)')
plt.colorbar(label='Timestamp')
plt.axis('equal')  # Ensure aspect ratio is equal
plt.show()
# plt.savefig("filtered_timestamp.png")

# raise NotImplementedError()


