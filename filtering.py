from scipy.signal import medfilt
from scipy.ndimage import uniform_filter1d
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple
from sensor_msgs.msg import Imu, LaserScan

class Filtering:
    def __init__(self, scan_data: List[LaserScan], imu_data: List[Imu]):
        self.__scan_data: List[LaserScan] = scan_data
        self.__imu_data: List[Imu] = imu_data

    def median_filter(self, window_size: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Make sure that the window size is an odd number to ensure that there is a median value
        window_size = window_size if window_size % 2 else window_size + 1

        # Initialize empty lists to store filtered data
        filtered_lidar_x = []
        filtered_lidar_y = []

        for scan in self.__scan_data:
            angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
            X = np.array(scan.ranges) * np.cos(angles)
            Y = np.array(scan.ranges) * np.sin(angles)

            # Apply median filter individually to each scan
            filtered_lidar_x.append(medfilt(X, window_size))
            filtered_lidar_y.append(medfilt(Y, window_size))

        # Concatenate filtered data from all scans into arrays
        filtered_lidar_x = np.concatenate(filtered_lidar_x)
        filtered_lidar_y = np.concatenate(filtered_lidar_y)

        # Extracting the IMU acceleration data in Y-axis
        imu_acc_y = np.array([imu.linear_acceleration.y for imu in self.__imu_data])

        # Applying the median filter to IMU data
        filtered_imu_acc_y = medfilt(imu_acc_y, window_size)
        
        return filtered_lidar_x, filtered_lidar_y, filtered_imu_acc_y
    
    def mean_filter(self, window_size: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Initialize empty lists to store filtered data
        filtered_lidar_x = []
        filtered_lidar_y = []

        for scan in self.__scan_data:
            angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
            X = np.array(scan.ranges) * np.cos(angles)
            Y = np.array(scan.ranges) * np.sin(angles)

            # Apply mean filter individually to each scan
            filtered_lidar_x.append(uniform_filter1d(X, size=window_size, mode='reflect'))
            filtered_lidar_y.append(uniform_filter1d(Y, size=window_size, mode='reflect'))

        # Concatenate filtered data from all scans into arrays
        filtered_lidar_x = np.concatenate(filtered_lidar_x)
        filtered_lidar_y = np.concatenate(filtered_lidar_y)

        # Extracting the IMU acceleration data in Y-axis
        imu_acc_y = np.array([imu.linear_acceleration.y for imu in self.__imu_data])

        # Applying the mean filter to IMU data
        filtered_imu_acc_y = uniform_filter1d(imu_acc_y, size=window_size, mode='reflect')
        
        return filtered_lidar_x, filtered_lidar_y, filtered_imu_acc_y

    def plot_filtered_lidar_data(self, 
                                        original_lidar_x: np.ndarray, 
                                        original_lidar_y: np.ndarray,
                                        filtered_lidar_x: np.ndarray, 
                                        filtered_lidar_y: np.ndarray, 
                                        figure_name: str,
                                        filter_name: str) -> None:
        # Create a time array for x-axis (assuming equal time intervals)
        time = np.arange(len(original_lidar_x))
        fig, axs = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot original and filtered X-axis data
        axs[0, 0].plot(time, original_lidar_x, 'b.', label='Original X axis')
        axs[0, 0].set_title('Original X axis')
        axs[0, 0].set_ylabel('Distance (meters)')
        axs[0, 0].grid(True)
        axs[0, 0].legend()
        
        axs[0, 1].plot(time, filtered_lidar_x, 'r.', label=f'Filtered X axis (using {filter_name} filter)')
        axs[0, 1].set_title('Filtered X axis')
        axs[0, 1].grid(True)
        axs[0, 1].legend()

        # Plot original and filtered Y-axis data
        axs[1, 0].plot(time, original_lidar_y, 'b.', label='Original Y axis')
        axs[1, 0].set_title('Original Y axis')
        axs[1, 0].set_xlabel('Time (assuming equal time intervals)')
        axs[1, 0].set_ylabel('Distance (meters)')
        axs[1, 0].grid(True)
        axs[1, 0].legend()
        
        axs[1, 1].plot(time, filtered_lidar_y, 'r.', label=f'Filtered Y axis (using {filter_name} filter)')
        axs[1, 1].set_title('Filtered Y axis')
        axs[1, 1].set_xlabel('Time (assuming equal time intervals)')
        axs[1, 1].grid(True)
        axs[1, 1].legend()

        plt.tight_layout()
        plt.savefig(figure_name)

    def plot_filtered_imu_data(self, 
                                      original_imu_acceleration_y: np.ndarray, 
                                      filtered_imu_acceleration_y: np.ndarray, 
                                      figure_name: str,
                                      filter_name: str) -> None:
        time = np.arange(0, len(original_imu_acceleration_y))
        fig, axs = plt.subplots(1, 2, figsize=(15, 5))

        # Plotting the original IMU data
        axs[0].plot(time, original_imu_acceleration_y, label='Original IMU Y Acceleration')
        axs[0].set_title('Original IMU Acceleration in Y-axis over Time')
        axs[0].set_xlabel('Time')
        axs[0].set_ylabel('Acceleration')
        axs[0].grid(True)
        axs[0].legend()

        # Plotting the filtered IMU data
        axs[1].plot(time, filtered_imu_acceleration_y, label=f'Filtered IMU Y Acceleration (using {filter_name} filter)', color='r')
        axs[1].set_title('Filtered IMU Acceleration in Y-axis over Time')
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('Acceleration')
        axs[1].grid(True)
        axs[1].legend()

        plt.tight_layout()
        plt.savefig(figure_name)
