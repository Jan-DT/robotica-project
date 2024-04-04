import abc
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from robot.robot import BotData


class BaseSonar(abc.ABC):
    @abc.abstractmethod
    def get_distance(self) -> float:
        pass


class TestSonar(BaseSonar):
    def __init__(self, data: 'BotData'):
        self.data = data

    @staticmethod
    def random_offset(spread: float) -> float:
        """
        Returns a random offset from a normal distribution with spread spread.
        Used to simulate sensor noise.
        :param spread: standard deviation of the normal distribution
        """
        return np.random.normal(0, spread)

    def get_distance(self):
        """Returns distance in cm. Simulates a sonar sensor, returns distance in cm."""
        angle = self.data.angle

        A = 160
        B = 200

        # simulate some obstacles
        if (120 * np.pi / 180) < angle < (130 * np.pi / 180):  # too close to the center
            return 5.0 + self.random_offset(2)

        if (60 * np.pi / 180) < angle < (85 * np.pi / 180):  # too large
            return 65.0 + self.random_offset(2)

        if (230 * np.pi / 180) < angle < (233 * np.pi / 180):  # very small
            return 50.0 + self.random_offset(2)

        if (320 * np.pi / 180) < angle < (325 * np.pi / 180):  # very close to the wall
            return 76.0 + self.random_offset(2)

        # simulate walls
        return np.sqrt(
            (A / 2 * np.cos(angle)) ** 2 + (B / 2 * np.sin(angle)) ** 2
        ) + self.random_offset(2) - np.sin(angle) * 10


## Commented out by default, as Python likes to complain about missing imports
# by setting them to None, we can bypass the import errors
rp = GetDistance = None

# import rospy as rp
# from geometry_msgs.msg import Pose, Vector3
# from mirte_msgs.msg import *
# from mirte_msgs.srv import *
# from sensor_msgs.msg import *
# from std_srvs.srv import *

class Sonar(BaseSonar):
    def __init__(self, service_name: str):
        self.distance_func = rp.ServiceProxy(service_name, GetDistance)

    def get_distance(self) -> float:
        return float(self.distance_func().data)
