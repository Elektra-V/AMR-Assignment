import math

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from src.common.types import CoordinatesTuple, TwistMessage

MAX_LINEAR_VELOCITY = 2.0
MAX_ANGULAR_VELOCITY = 1.0
ATTRACTIVE_GAIN = 2.0
REPULSIVE_GAIN = 0.05
REPULSIVE_DISTANCE = 0.1


class PotentialFieldService:
    def run_potential_field(
        self,
        current_position: CoordinatesTuple,
        goal: CoordinatesTuple,
        odometry: Odometry,
        laser_scan: LaserScan,
    ) -> TwistMessage:
        attractive_force = self.__calculate_attractive_force(
            current_position=current_position, goal=goal, odometry=odometry
        )
        repulsive_force = self.__calculate_repulsive_force(
            odometry=odometry, laser_scan=laser_scan
        )

        return TwistMessage(
            linear_velocity_x=min(
                MAX_LINEAR_VELOCITY,
                attractive_force.linear_velocity_x + repulsive_force.linear_velocity_x,
            ),
            angular_velocity_z=min(
                MAX_ANGULAR_VELOCITY,
                attractive_force.angular_velocity_z
                + repulsive_force.angular_velocity_z,
            ),
        )

    def __calculate_attractive_force(
        self,
        current_position: CoordinatesTuple,
        goal: CoordinatesTuple,
        odometry: Odometry,
    ) -> TwistMessage:
        yaw = self.__quaternion_to_euler(odometry.pose.pose.orientation)
        delta_x = goal[0] - current_position[0]
        delta_y = goal[1] - current_position[1]

        angle_to_goal = math.atan2(delta_y, delta_x)
        relative_angle_to_goal = angle_to_goal - yaw

        distance_to_goal = math.sqrt(delta_x**2 + delta_y**2)
        linear_velocity = ATTRACTIVE_GAIN * distance_to_goal

        angular_velocity = ATTRACTIVE_GAIN * relative_angle_to_goal

        return TwistMessage(
            linear_velocity_x=min(MAX_LINEAR_VELOCITY, linear_velocity),
            angular_velocity_z=min(MAX_ANGULAR_VELOCITY, angular_velocity),
        )

    def __calculate_repulsive_force(
        self, odometry: Odometry, laser_scan: LaserScan
    ) -> TwistMessage:
        yaw = self.__quaternion_to_euler(odometry.pose.pose.orientation)
        repulsive_linear_velocity_x = 0.0
        repulsive_angular_velocity_z = 0.0

        for i, range_value in enumerate(laser_scan.ranges):
            if 0 < range_value < REPULSIVE_DISTANCE:
                obstacle_angle = laser_scan.angle_min + i * laser_scan.angle_increment
                relative_obstacle_angle = obstacle_angle - yaw

                force_magnitude = REPULSIVE_GAIN / range_value
                repulsive_linear_velocity_x += force_magnitude * math.cos(
                    relative_obstacle_angle
                )
                repulsive_angular_velocity_z += force_magnitude * math.sin(
                    relative_obstacle_angle
                )

        return TwistMessage(
            linear_velocity_x=-abs(repulsive_linear_velocity_x),
            angular_velocity_z=-abs(repulsive_angular_velocity_z),
        )

    def __quaternion_to_euler(self, quaternion: Quaternion) -> float:
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (w * z + x + y)
        cosy_cosp = 1 - 2 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)
