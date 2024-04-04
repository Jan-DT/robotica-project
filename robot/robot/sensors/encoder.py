import abc


class BaseEncoder(abc.ABC):
    @abc.abstractmethod
    def get_ticks(self) -> int:
        pass

    @abc.abstractmethod
    def reset_ticks(self):
        pass

    @abc.abstractmethod
    def get_ticks_per_rev(self) -> int:
        pass


class TestEncoder(BaseEncoder):
    def __init__(self, motor):
        self.ticks = 0
        self.motor = motor

    def get_ticks(self) -> int:
        self.ticks += self.motor.get_speed() / 50
        return self.ticks

    def reset_ticks(self):
        self.ticks = 0

    def get_ticks_per_rev(self) -> int:
        return 20


class PlaceholderEncoder(BaseEncoder):
    def get_ticks(self) -> int:
        return 0

    def reset_ticks(self):
        pass

    def get_ticks_per_rev(self) -> int:
        return 10


## Commented out by default, as Python likes to complain about missing imports
# by setting them to None, we can bypass the import errors
rp = GetEncoderTicks = None

# import rospy as rp
# from geometry_msgs.msg import Pose, Vector3
# from mirte_msgs.msg import *
# from mirte_msgs.srv import *
# from sensor_msgs.msg import *
# from std_srvs.srv import *

class Encoder(BaseEncoder):
    def __init__(self, encoder_service, ticks_per_rev=20):
        self.encoder_func = rp.ServiceProxy(encoder_service, GetEncoderTicks)
        self.ticks = 0
        self.ticks_per_rev = ticks_per_rev

    def get_ticks(self) -> int:
        return self.encoder_func().data - self.ticks

    def get_ticks_per_rev(self) -> int:
        return self.ticks_per_rev

    def reset_ticks(self):
        self.ticks = self.get_ticks()
