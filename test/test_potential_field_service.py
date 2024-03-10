import unittest

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from src.common.types import Coordinates
from src.services.potential_field_service import (
    MAX_ANGULAR_VELOCITY,
    MAX_LINEAR_VELOCITY,
    PotentialFieldService,
)


class TestPotentialFieldService(unittest.TestCase):
    def setUp(self):
        self.service = PotentialFieldService()

    def create_mock_odometry(
        self, x: int | float, y: int | float, orientation_z: int | float
    ) -> Odometry:
        odometry = Odometry()
        odometry.pose.pose.position.x = float(x)
        odometry.pose.pose.position.y = float(y)
        odometry.pose.pose.orientation = Quaternion(z=float(orientation_z))
        return odometry

    def create_mock_laser_scan(self, ranges: list) -> LaserScan:
        laser_scan = LaserScan()
        laser_scan.ranges = [float(range_value) for range_value in ranges]
        laser_scan.angle_min = -1.57
        laser_scan.angle_increment = 0.1
        return laser_scan

    def test_calculate_attractive_force(self):
        current_position = Coordinates(x=0, y=0)
        goal = Coordinates(x=10, y=0)
        odometry = self.create_mock_odometry(0, 0, 0)

        attractive_force = (
            self.service._PotentialFieldService__calculate_attractive_force(
                current_position, goal, odometry
            )
        )

        self.assertAlmostEqual(attractive_force.linear_velocity_x, 2.0)
        self.assertAlmostEqual(attractive_force.angular_velocity_z, 0)

    def test_calculate_repulsive_force(self):
        odometry = self.create_mock_odometry(0, 0, 0)
        laser_scan = self.create_mock_laser_scan([0.05] * 10 + [1] * 10)

        repulsive_force = (
            self.service._PotentialFieldService__calculate_repulsive_force(
                odometry, laser_scan
            )
        )

        self.assertLess(repulsive_force.linear_velocity_x, 0)
        self.assertNotEqual(repulsive_force.angular_velocity_z, 0)

    def test_run_potential_field_integration(self):
        current_position = Coordinates(x=0, y=0)
        goal = Coordinates(x=10, y=0)
        odometry = self.create_mock_odometry(0, 0, 0)
        laser_scan = self.create_mock_laser_scan([1] * 20)

        twist_message = self.service.run_potential_field(
            current_position, goal, odometry, laser_scan
        )

        self.assertLessEqual(twist_message.linear_velocity_x, MAX_LINEAR_VELOCITY)
        self.assertLessEqual(
            abs(twist_message.angular_velocity_z), MAX_ANGULAR_VELOCITY
        )


if __name__ == "__main__":
    unittest.main()
