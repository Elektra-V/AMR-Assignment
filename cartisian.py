#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import math

def plot_scan_data(input_bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    scan_data = []
    timestamps = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic.endswith("/scan"):  # Adjust the topic name as needed
            msg = deserialize_message(data, LaserScan)  # Use the appropriate message type

            # Get the angular resolution and angle_min from the LaserScan message
            angular_increment = msg.angle_increment
            angle_min = msg.angle_min

            for i, range_value in enumerate(msg.ranges):
                if not math.isinf(range_value):  # Skip invalid measurements
                    # Convert to Cartesian coordinates
                    x = range_value * math.cos(angle_min + i * angular_increment)
                    y = range_value * math.sin(angle_min + i * angular_increment)

                    scan_data.append((x, y))
                    timestamps.append(timestamp)
    # Plot the scan data in Cartesian coordinates
    plt.figure()
    y_values,x_values = zip(*scan_data)
    plt.scatter(x_values, y_values)#, c=timestamps, cmap='viridis', marker='.')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Laser Scan Data (Cartesian)')
    plt.colorbar(label='Timestamp')
    plt.axis('equal')  # Ensure aspect ratio is equal
    plt.show()

   
if __name__ == "__main__":
    input_bag = "/home/ibhu/amr_assignmnet/amr_assignmnet_0.db3"  # Replace with the path to your bag file
    plot_scan_data(input_bag)
