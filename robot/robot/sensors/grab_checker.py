import abc
import time


class BaseGrabChecker(abc.ABC):
    @abc.abstractmethod
    def get_conductive(self) -> bool:
        pass


class TestGrabChecker(BaseGrabChecker):
    def get_conductive(self) -> bool:
        return True if time.time() % 2 == 0 else False


## Commented out by default, as Python likes to complain about missing imports
# by setting them to None, we can bypass the import errors
GetPinValue = None

# import rospy as rp
# from geometry_msgs.msg import Pose, Vector3
# from mirte_msgs.msg import *
# from mirte_msgs.srv import *
# from sensor_msgs.msg import *
# from std_srvs.srv import *

class GrabChecker(BaseGrabChecker):
    def __init__(self, pin, pin_type):
        self.pin = pin
        self.pin_type = pin_type

    def get_conductive(self) -> bool:
        return GetPinValue(self.pin, self.pin_type)
