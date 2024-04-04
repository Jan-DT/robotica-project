#!/usr/bin/env python3
# license removed for brevity

import math
import time
from audioop import avg

import numpy as np
import rospy as rp
from geometry_msgs.msg import Pose, Vector3
from mirte_msgs.msg import *
from mirte_msgs.srv import *
from sensor_msgs.msg import *
from std_srvs.srv import *

# SetRightSpeed = rp.ServiceProxy('/mirte/set_right_speed', SetMotorSpeed)
# SetLeftSpeed = rp.ServiceProxy('/mirte/set_left_speed', SetMotorSpeed)
# GetDistanceLeft = rp.ServiceProxy('/mirte/get_distance_left', GetDistance)
GetPinValue = rp.ServiceProxy('/mirte/get_pin_value', GetPinValue)
# GetEncoderLeft = rp.ServiceProxy('/mirte/get_encoder_left', GetEncoder)
SetLeftServo = rp.ServiceProxy('/mirte/set_left_servo_angle', SetServoAngle)

# distances = []

# def callback(data) -> None:
#     distance = GetDistanceLeft()

#     distances.append(float(distance.data))
#     if len(distances) > 10:
#         distances.pop(0)
#     avg_distance = sum(distances) / len(distances)

#     error = float(avg_distance) - D
#     speed = min(max(math.floor(K_P * error), -100), 100)

# pin_26 = GetPinValue('26','analog')  #LET OP: dit is voor de PICO, waar alleen pinnen 26,27,28 analoge ingangen
# zijn. Gebruik andere pinnen bij de Arduino.

#     SetLeftSpeed(LEFT_SIGN * speed)
#     SetRightSpeed(math.floor(RIGHT_SIGN * speed / 1.4))
#     print(f"afstand: {distance.data}, error: {error}, speed: {speed}, pin26: {pin_26}")


def main():
    rp.init_node('jt_test_grabber', anonymous=True)
    print('Gestart!')

    def set_grabber(state=True):
        if state:
            SetLeftServo(200)
        else:
            SetLeftServo(90)
        return state

    def test_grabber():
        return GetPinValue('22', 'digital')

    state = set_grabber(True)

    while True:
        print(f"Grabber: {state}, pin22: {test_grabber()}")
        _input = input("Enter to toggle")
        if _input == 'q':
            break
        state = set_grabber(not state)
        time.sleep(2.0)


if __name__ == '__main__':
    main()








