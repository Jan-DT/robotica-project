from typing import Optional, TYPE_CHECKING

import numpy as np

from robot.sensors.encoder import PlaceholderEncoder

if TYPE_CHECKING:
    from robot.movement import Move
from robot.states import GrabState, MoveState, RunState, ScanState, State, ManualState, AutoState
from robot.sensors import (BaseEncoder, BaseMotor, BaseSonar, TestEncoder, TestMotor, TestSonar, TestServo,
                           TestGrabChecker, BaseServo, BaseGrabChecker)
from robot.sensors import Encoder, Motor, Sonar, Servo, GrabChecker


class BotComponents:
    def __init__(self,
                 left_motor: BaseMotor,
                 right_motor: BaseMotor,
                 left_encoder: BaseEncoder,
                 right_encoder: BaseEncoder,
                 low_sonar: BaseSonar,
                 high_sonar: BaseSonar,
                 servo: BaseServo,
                 grab_checker: BaseGrabChecker,
                 wheel_diameter: float = 3.5):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.low_sonar = low_sonar
        self.high_sonar = high_sonar
        self.servo = servo
        self.grab_checker = grab_checker

        self.wheel_diameter = wheel_diameter  # cm

    @staticmethod
    def default_components() -> 'BotComponents':
        """
        Returns the default components for the robot
        :return: BotComponents
        """
        return BotComponents(Motor('/mirte/set_left_speed', correction=1.0),
                             Motor('/mirte/set_right_speed', correction=1.0 / 1.4),
                             # Encoder('/mirte/get_encoder_left'),
                             # Encoder('/mirte/get_encoder_right'),
                             PlaceholderEncoder(),
                             PlaceholderEncoder(),
                             Sonar('/mirte/get_distance_left'),
                             Sonar('/mirte/get_distance_right'),
                             Servo('/mirte/set_left_servo_angle'),
                             GrabChecker(22, 'digital'))

    @staticmethod
    def test_components(data: 'BotData') -> 'BotComponents':
        """
        Returns the test components for the robot
        :return: BotComponents
        """
        motor_l = TestMotor()
        motor_r = TestMotor()
        return BotComponents(motor_l, motor_r, TestEncoder(motor_l), TestEncoder(motor_r), TestSonar(data), TestSonar(data), TestServo, TestGrabChecker)


class BotData:
    def __init__(self, grid_size: int = 600, start_pos: np.ndarray = np.array((0.0, 0.0))):
        self.position: np.ndarray = start_pos  # actual position in cm
        self.angle: float = 0.0  # actual heading in radians
        self.grid = np.zeros((grid_size, grid_size))  # occupancy grid in cm
        self.light_map = np.zeros((grid_size // 10, grid_size // 10))  # unused for now
        self.grabber: bool = True  # up by default, as the sonar looks from underneath the grabber

        self.target_position: np.ndarray = np.array((0.0, 0.0))  # target position in cm, used for move planning
        self.target_angle: float = 0.0  # target heading in radians, used for move planning
        self.moves = []  # list of moves to be executed
        self.current_move: Optional['Move'] = None  # current move being executed


class StateMachine:
    def __init__(self, data: BotData, components: BotComponents, initial_state: State):
        self.data = data
        self.components = components
        self.current_state: Optional[State] = initial_state

    def execute(self):
        print("STATEMACHINE: Executing state")
        next_state, self.data = self.current_state.execute()
        if next_state is 'ManualState':
            self.current_state = ManualState
        elif next_state is 'GrabState':
            self.current_state = GrabState
        elif next_state is 'MoveState':
            self.current_state = MoveState
        elif next_state is 'ScanState':
            self.current_state = ScanState
        elif next_state is 'RunState':
            self.current_state = RunState
        elif next_state is 'AutoState':
            self.current_state = AutoState
        else:
            self.current_state = None


class Bot:
    def __init__(self, data: BotData, components: BotComponents, initial_state: 'State' = ManualState):
        self.data = data
        self.components = components
        self.state_machine = StateMachine(self.data, self.components, initial_state)

        self.components.servo.set_angle(200)
        self.data.grabber = True

    def execute(self):
        self.state_machine.execute()

    def run(self):
        print("ROBOT: Running")
        while True:
            self.execute()
            if self.state_machine.current_state is None:
                break

        print("ROBOT: Done running")
