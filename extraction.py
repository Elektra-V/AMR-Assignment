from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan

import numpy as np
import matplotlib.pyplot as plt

class ScanAndIMUExtraction:
    __SCAN_TOPIC_NAME = '/scan'
    __IMU_TOPIC_NAME = '/imu'

    def __init__(self, bag_file_path: str) -> None:
        self.__bag_file_path: str = bag_file_path
        self.__reader = self.__construct_sequential_reader()

    def read_scan_and_imu_data(self) -> tuple[list[LaserScan], list[Imu]]:
        scan_data: list[LaserScan] = []
        imu_data: list[Imu] = []

        while self.__reader.has_next():
            (topic, data, _) = self.__reader.read_next()
            if topic == ScanAndIMUExtraction.__SCAN_TOPIC_NAME:
                scan_data.append(deserialize_message(data, LaserScan()))
            if topic == ScanAndIMUExtraction.__IMU_TOPIC_NAME:
                imu_data.append(deserialize_message(data, Imu()))
        
        return scan_data, imu_data
    
    def turn_scan_data_into_arrays(self, scan_data: list[LaserScan]) -> tuple[np.ndarray, np.ndarray]:
        lidar_x = np.concatenate([
            np.array(scan.ranges) * np.cos(np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment))
            for scan in scan_data
        ])
        lidar_y = np.concatenate([
            np.array(scan.ranges) * np.sin(np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment))
            for scan in scan_data
        ])
        return lidar_x, lidar_y

    def turn_imu_data_into_array(self, imu_data: list[Imu]) -> np.ndarray:
        return np.array([imu.linear_acceleration.y for imu in imu_data])
    
    def visualize_scan_data(self, scan_data: list[LaserScan], figure_name: str) -> None:
        plt.figure(figsize=(10, 5))

        for scan in scan_data:
            timestamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

            angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)            
            X = np.array(scan.ranges) * np.cos(angles)
            Y = np.array(scan.ranges) * np.sin(angles)
            
            plt.plot([timestamp] * len(X), X, 'b.', label='X axis' if scan is scan_data[0] else "")
            plt.plot([timestamp] * len(Y), Y, 'r.', label='Y axis' if scan is scan_data[0] else "")
        
        plt.title('LiDAR Data (X and Y axis) with respect to time')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (meters)')
        plt.legend()
        plt.savefig(figure_name)

    def visualize_imu_data(self, imu_data: list[Imu], figure_name: str) -> None:
        time = np.arange(0, len(imu_data))  # replace with actual time data if available
        acceleration_y = np.array([data.linear_acceleration.y for data in imu_data])
        
        plt.figure()
        plt.plot(time, acceleration_y, label='Acceleration Y')
        plt.title('Acceleration in Y-axis over Time')
        plt.xlabel('Time')
        plt.ylabel('Acceleration')
        plt.grid(True)
        plt.legend()
        plt.savefig(figure_name)

    def __construct_sequential_reader(self) -> SequentialReader:
        storage_options = StorageOptions(uri=self.__bag_file_path)
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        return reader
