#!/usr/bin/env python3




import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan  # Import the appropriate message type

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

            # Assuming the scan data is in a list called "ranges"
            scan_data.extend(msg.ranges)
            timestamps.extend([timestamp] * len(msg.ranges))


    # Plot the scan data
    plt.figure()
    plt.plot(timestamps, scan_data, label='Scan Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Scan Value')
    plt.title('Scan Data Plot')
    plt.legend()
    plt.show()



if __name__ == "__main__":
    input_bag = "/home/ibhu/amr_assignmnet/AMR_WS2023/assign2/robile_bag_file/robile_bag_file.db3"  # Replace with the path to your bag file
    plot_scan_data(input_bag)
