import math
from typing import List, Tuple

import numpy as np
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.common.types import InputMessageFull, Particle
from src.utils.event_bus import EventBus


class LocalizationService:
    __latest_odometry: Odometry | None = None
    __latest_scan: LaserScan | None = None

    def __init__(
        self,
        event_bus: EventBus,
        initial_position: Tuple[float, float],
    ):
        self.__event_bus = event_bus
        self.__particles = self.__initialize_particles(initial_position)

        self.__estimated_position = initial_position

    def update_odometry_and_scan(self, msg: InputMessageFull):
        if isinstance(msg, Odometry):
            self.__latest_odometry = msg
        if isinstance(msg, LaserScan):
            self.__latest_scan = msg
            self.__perform_mcl()

    def __initialize_particles(
        self,
        position: Tuple[float, float],
        uncertainty_x: float = 0.5,
        uncertainty_y: float = 0.5,
        uncertainty_yaw=0.5,
    ) -> List[Particle]:
        num_particles = 100
        particles = []

        for _ in range(num_particles):
            x_sample = np.random.normal(loc=position[0], scale=uncertainty_x)
            y_sample = np.random.normal(loc=position[1], scale=uncertainty_y)
            yaw_sample = np.random.normal(loc=0.0, scale=uncertainty_yaw)

            orientation_sample = self.__euler_to_quaternion(0, 0, yaw_sample)

            particle = Particle(
                position=Point(x=x_sample, y=y_sample, z=0.0),
                orientation=orientation_sample,
                weight=0.0,
            )

            particles.append(particle)

        return particles

    def __perform_mcl(self):
        if not self.__latest_odometry or not self.__latest_scan:
            return

        self.__motion_update()
        self.__sensor_update()
        self.__resample()
        self.__estimate_position()
        self.__publish_estimated_position()

    def __motion_update(self):
        if self.__latest_odometry is None:
            return

        delta_x = (
            self.__latest_odometry.pose.pose.position.x - self.__estimated_position[0]
        )
        delta_y = (
            self.__latest_odometry.pose.pose.position.y - self.__estimated_position[1]
        )
        delta_yaw = self.__euler_to_quaternion(
            self.__latest_odometry.pose.pose.orientation.w
        ).w

        for particle in self.__particles:
            particle_yaw = (
                self.__euler_from_quaternion(particle.orientation)[2] + delta_yaw
            )
            particle.position.x += delta_x + np.random.normal(0, 0.1)
            particle.position.y += delta_y + np.random.normal(0, 0.1)
            particle.orientation = self.__euler_to_quaternion(0, 0, particle_yaw)

    def __sensor_update(self):
        if self.__latest_scan is None:
            return

        for particle in self.__particles:
            weight = 1.0
            particle.weight = weight

    def __resample(self):
        weights = [particle.weight for particle in self.__particles]
        new_particles = np.random.choice(
            self.__particles, size=len(self.__particles), p=weights / np.sum(weights)
        )
        self.__particles = list(new_particles)

    def __estimate_position(self):
        x = np.mean([particle.position.x for particle in self.__particles])
        y = np.mean([particle.position.y for particle in self.__particles])
        yaw = np.mean(
            [
                self.__euler_from_quaternion(particle.orientation)[2]
                for particle in self.__particles
            ]
        )
        self.__estimated_position = (x, y, yaw)

    def __publish_estimated_position(self):
        self.__event_bus.publish(
            Constants.EVENT_TOPIC_LOCALIZED_LOCATION, self.__estimated_position
        )

    def __euler_to_quaternion(
        self, roll: float, pitch: float = 0.0, yaw: float = 0.0
    ) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def __euler_from_quaternion(
        self, quaternion: Quaternion
    ) -> Tuple[float, float, float]:
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z
