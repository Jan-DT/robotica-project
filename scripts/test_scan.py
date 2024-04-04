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

SetRightSpeed = rp.ServiceProxy('/mirte/set_right_speed', SetMotorSpeed)
SetLeftSpeed = rp.ServiceProxy('/mirte/set_left_speed', SetMotorSpeed)
GetDistanceLeft = rp.ServiceProxy('/mirte/get_distance_left', GetDistance)
GetPinValue = rp.ServiceProxy('/mirte/get_pin_value', GetPinValue)
# SetLeftServo = rp.ServiceProxy('/mirte/set_left_servo_angle', SetServoAngle)

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

GRID_SIZE = 400
UNCERTAINTY = 5
FOV = 15  # sonar field of view
MOTOR_A_CORR = 1.0
MOTOR_B_CORR = 1.0
DRIVE_SPEED = 100
ROTATE_SPEED = -100


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def rotate(angle: float):
    left_speed = int(clamp(MOTOR_A_CORR * ROTATE_SPEED, -100, 100))
    right_speed = int(clamp(-MOTOR_B_CORR * ROTATE_SPEED, -100, 100))
    print(f"rotating: {angle}, left: {left_speed}, right: {right_speed}")
    SetLeftSpeed(left_speed)
    SetRightSpeed(right_speed)
    time.sleep(0.7)
    SetLeftSpeed(0)
    SetRightSpeed(0)


def grid_to_world(x: int, y: int):
    return x - GRID_SIZE / 2, y - GRID_SIZE / 2


def world_to_grid(x: float, y: float):
    return int(x + GRID_SIZE / 2), int(y + GRID_SIZE / 2)


def draw_circle(grid: np.ndarray, x: int, y: int, radius: int):
    for _i in range(x - radius, x + radius):
        for _j in range(y - radius, y + radius):
            if _i < 0 or _i >= GRID_SIZE or _j < 0 or _j >= GRID_SIZE:
                continue
            if (_i - x) ** 2 + (_j - y) ** 2 < radius ** 2:
                grid[_i, _j] = 1
    return grid


def draw_ellipse(grid: np.ndarray, x: int, y: int, width: int, height: int, angle: int):
    for _i in range(x - width, x + width):
        for _j in range(y - height, y + height):
            dx = _i - x
            dy = _j - y
            if (dx * np.cos(angle) + dy * np.sin(angle)) ** 2 / width ** 2 + \
                    (dx * np.sin(angle) - dy * np.cos(angle)) ** 2 / height ** 2 < 1:
                grid[_i, _j] = 1
    return grid


def draw_measurement(grid: np.ndarray, x: int, y: int, angle: float, distance: float):
    x, y = grid_to_world(x, y)
    angle = np.radians(angle)
    dx = distance * np.cos(angle)
    dy = distance * np.sin(angle)
    x2, y2 = world_to_grid(x + dx, y + dy)
    # return draw_ellipse(grid, x2, y2, UNCERTAINTY, FOV, angle)
    return draw_circle(grid, x2, y2, UNCERTAINTY)


def scan(grid: np.ndarray, count: int, resolution: int):
    for i in range(count):
        print(f"scan {i + 1} of {count}")
        for j in range(resolution):
            angle = -j * 180 / resolution
            rotate(angle)
            distance = GetDistanceLeft()
            grid = draw_measurement(grid, GRID_SIZE // 2, GRID_SIZE // 2, angle, distance.data)

            print(f"angle: {angle}, distance: {distance.data}")
            time.sleep(1.0)

    print("done scanning")
    return grid


def save_grid_to_file(grid: np.ndarray, filename: str):
    with open(filename, 'wb') as f:
        np.save(f, grid)


def run():
    print("running")
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    grid = scan(grid, 1, 12)
    print(grid)
    save_grid_to_file(grid, 'grid.npy')


def main():
    rp.init_node('jt_testnode', anonymous=True)
    print('Gestart!')
    run()
    SetLeftSpeed(0)
    SetRightSpeed(0)
    rp.signal_shutdown('Done')


if __name__ == '__main__':
    main()








