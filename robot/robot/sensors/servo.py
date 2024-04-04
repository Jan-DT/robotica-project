import abc
import time


class BaseServo(abc.ABC):
    @abc.abstractmethod
    def get_angle(self) -> int:
        pass

    @abc.abstractmethod
    def set_angle(self, angle: int):
        pass


class TestServo(BaseServo):
    def __init__(self, data):
        self.data = data
        self.angle = 0

    def get_angle(self) -> int:
        return self.angle

    def set_angle(self, angle: int):
        time.sleep(1)
        self.angle = angle


## Commented out by default, as Python likes to complain about missing imports
# by setting them to None, we can bypass the import errors
rp = SetServoAngle = None

# import rospy as rp
# from geometry_msgs.msg import Pose, Vector3
# from mirte_msgs.msg import *
# from mirte_msgs.srv import *
# from sensor_msgs.msg import *
# from std_srvs.srv import *

class Servo(BaseServo):
    def __init__(self, servo_service):
        self.encoder_func = rp.ServiceProxy(servo_service, SetServoAngle)
        self.angle = 0

    def get_angle(self) -> int:
        return self.angle

    def set_angle(self, angle: int):
        self.angle = angle
        self.encoder_func(angle)
