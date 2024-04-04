import enum
from typing import Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from robot.robot import BotData


class MoveType(enum.Enum):
    FORWARD = 0
    ROTATION = 2
    GRAB = 2
    SCAN = 3


class Move:
    def __init__(self, move_type: 'MoveType', **params):
        """
        Create a move
        :param move_type: Type of move
        :param params: Parameters for the move
        """
        self.move_type = move_type
        self.params = params

    def __str__(self):
        return "{0} {1}".format(self.move_type, self.params)

    def __repr__(self):
        return self.__str__()

    @staticmethod
    def moves_to(start_pos: np.array, start_angle: float, end_pos: np.array, end_angle: Optional[float] = None) -> \
            (list, np.array, float):
        """
        Calculate the moves needed to go from start to end position and angle
        :param start_pos: Starting position
        :param start_angle: Starting angle
        :param end_pos: Ending position
        :param end_angle: Ending angle
        :return: List of moves, end position, end angle
        """
        move_direction = Move.direction_to(start_pos, start_angle, end_pos)  # direction to move in
        moves = [Rotation(move_direction, relative=True),
                 Movement(np.linalg.norm(end_pos - start_pos))]  # move to end position
        if end_angle is not None:
            moves.append(Rotation(end_angle, relative=False))  # rotate to end angle
        else:
            end_angle = start_angle + move_direction  # otherwise, set end angle to the move direction
        return moves, end_pos, end_angle

    @staticmethod
    def direction_to(start_pos: np.array, start_angle: float, end_pos: np.array):
        """
        Calculate the direction to move in to reach the end position
        :param start_pos: Starting position
        :param start_angle: Starting angle
        :param end_pos: Ending position
        :return: Direction in radians
        """
        return np.arctan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0]) - start_angle


class Movement(Move):
    def __init__(self, distance: float):
        """Distance is in cm. Positive is forward, negative is backward."""
        super().__init__(MoveType.FORWARD, distance=distance)

    def __str__(self):
        return "{0} {1:.2f}cm".format(self.move_type, self.params['distance'])


class Rotation(Move):
    def __init__(self, amount: float, relative: bool = False):
        """Amount is in radians. Positive is clockwise, negative is counter-clockwise."""
        super().__init__(MoveType.ROTATION, amount=amount, relative=relative)

    def __str__(self):
        return "{0} {1:.2f}deg {2}".format(self.move_type, self.params['amount'] / np.pi * 180,
                                           "rel" if self.params['relative'] else "abs")


class Grab(Move):
    def __init__(self, state: Optional[bool] = None):
        super().__init__(MoveType.GRAB, state=state)

    def __str__(self):
        return "{0} {1}".format(self.move_type, "grab" if self.params['state'] else "release")


class Scan(Move):
    def __init__(self, count: int, resolution: int):
        super().__init__(MoveType.SCAN, count=count, resolution=resolution)

    def __str__(self):
        return "{0} {1} {2}".format(self.move_type, self.params['count'], self.params['resolution'])
