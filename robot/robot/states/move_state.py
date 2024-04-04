import time
from typing import TYPE_CHECKING

import numpy as np

from robot.movement import MoveType
from robot.states import State

if TYPE_CHECKING:
    from robot.robot import BotData

AVOID_OBSTACLE_CM = 15.0  # cm
SPEED_AT_100 = 15.0  # cm/s
ROT_AT_100 = 80 * np.pi / 180  # rad/s


class MoveState(State):
    def __init__(self, data, components):
        super().__init__(data, components)
        self.current_move = self.data.current_move

        self.encoder_l = self.components.left_encoder
        self.encoder_r = self.components.right_encoder

        self.speed = 100

        self.move = self.move_without_encoder
        self.rotate = self.rotate_without_encoder

    def allow_transition(self, from_state) -> bool:
        return True

    def execute(self) -> (str, 'BotData'):
        if self.current_move.move_type == MoveType.FORWARD:
            self.move(self.current_move.params['distance'])
        elif self.current_move.move_type == MoveType.ROTATION:
            if self.current_move.params['relative']:
                self.rotate(self.current_move.params['amount'])
            else:
                self.rotate(self.current_move.params['amount'] - self.data.angle)
        return 'RunState', self.data

    def move_with_encoder(self, distance):
        motor_l = self.components.left_motor
        motor_r = self.components.right_motor
        obstacle_sensor = self.components.high_sonar

        self.encoder_l.reset_ticks()
        self.encoder_r.reset_ticks()

        ticks = self.cm_to_tick(distance)

        motor_l.set_speed(self.speed)
        motor_r.set_speed(self.speed)

        while self.encoder_l.get_ticks() < ticks and self.encoder_r.get_ticks() < ticks:
            n_times_blocked = 0

            while obstacle_sensor.get_distance() < AVOID_OBSTACLE_CM:
                motor_l.stop()
                motor_r.stop()
                n_times_blocked += 1
                time.sleep(0.1)

                if n_times_blocked > 50:
                    self.move(-20)

                    time.sleep(3)

                    motor_l.stop()
                    motor_r.stop()
                    n_times_blocked -= 20

            if self.encoder_l.get_ticks() >= ticks and self.encoder_r.get_ticks() >= ticks:
                break

            if self.encoder_l.get_ticks() >= ticks:
                motor_l.stop()
            else:
                motor_l.set_speed(self.speed)
            if self.encoder_r.get_ticks() >= ticks:
                motor_r.stop()
            else:
                motor_r.set_speed(self.speed)

            self.data.position += (np.array([np.cos(self.data.angle), np.sin(self.data.angle)])
                                   * self.tick_to_cm(self.encoder_l.get_ticks()))

            time.sleep(0.01)

        motor_l.stop()
        motor_r.stop()

    def move_without_encoder(self, distance):
        # try to estimate the time it will take to move the distance, as we don't have encoders :(
        ESTIMATED_TIME = distance / SPEED_AT_100

        motor_l = self.components.left_motor
        motor_r = self.components.right_motor
        obstacle_sensor = self.components.high_sonar

        motor_l.set_speed(100)
        motor_r.set_speed(100)

        start_time = time.time()
        while time.time() - start_time < ESTIMATED_TIME:
            if obstacle_sensor.get_distance() < AVOID_OBSTACLE_CM:
                motor_l.stop()
                motor_r.stop()
                time.sleep(0.1)
                continue

            self.data.position += (np.array([np.cos(self.data.angle), np.sin(self.data.angle)]) * distance)
            time.sleep(0.01)

        motor_l.stop()
        motor_r.stop()

    def rotate_without_encoder(self, angle):
        # try to estimate the time it will take to rotate the angle
        ESTIMATED_TIME = angle / ROT_AT_100

        motor_l = self.components.left_motor
        motor_r = self.components.right_motor

        motor_l.set_speed(100)
        motor_r.set_speed(-100)

        time.sleep(ESTIMATED_TIME)

        motor_l.stop()
        motor_r.stop()

        self.data.angle += angle

    def tick_to_cm(self, ticks):
        ticks_per_rev = self.components.left_encoder.get_ticks_per_rev()
        return ticks / ticks_per_rev * np.pi * self.components.wheel_diameter

    def cm_to_tick(self, cm):
        ticks_per_rev = self.components.left_encoder.get_ticks_per_rev()
        return cm / (np.pi * self.components.wheel_diameter) * ticks_per_rev
