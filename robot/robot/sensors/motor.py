import abc


class BaseMotor(abc.ABC):
    @abc.abstractmethod
    def set_speed(self, speed: float):
        pass

    @abc.abstractmethod
    def get_speed(self) -> float:
        pass

    @abc.abstractmethod
    def stop(self):
        pass


class TestMotor(BaseMotor):
    def __init__(self, correction: float = 1.0):
        self.speed: int = 0
        self.correction: float = correction

    def set_speed(self, target_speed: float):
        speed = max(-100.0, min(100.0, target_speed))
        speed = int(speed * self.correction)
        self.speed = speed

    def get_speed(self):
        return self.speed

    def stop(self):
        self.speed = 0


## Commented out by default, as Python likes to complain about missing imports
# by setting them to None, we can bypass the import errors
rp = SetMotorSpeed = None

# import rospy as rp
# from geometry_msgs.msg import Pose, Vector3
# from mirte_msgs.msg import *
# from mirte_msgs.srv import *
# from sensor_msgs.msg import *
# from std_srvs.srv import *


class Motor(BaseMotor):
    def __init__(self, motor_service, correction: float = 1.0):
        self.speed_func = rp.ServiceProxy(motor_service, SetMotorSpeed)
        self.correction: float = correction

        self.speed: int = 0

    def set_speed(self, target_speed: float):
        speed = max(-100.0, min(100.0, target_speed))
        speed = int(speed * self.correction)
        self.speed = speed
        self.speed_func(speed)

    def get_speed(self):
        return self.speed

    def stop(self):
        self.speed = 0
        self.speed_func(0)
