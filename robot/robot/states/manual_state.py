from typing import List, Type, TYPE_CHECKING

import numpy as np
import traceback

from robot.movement import Grab, Move, Movement, Rotation, Scan
from robot.states import State

if TYPE_CHECKING:
    from robot.robot import BotData
    
    
# In this state the user can manually declare the moves the robot should make
# Very much in need of a refactor


class ManualState(State):
    def __init__(self, data, components):
        super().__init__(data, components)

    def allow_transition(self, from_state) -> bool:
        return True

    def execute(self) -> (str, 'BotData'):
        print(f"Current position: {self.data.position} | "
              f"Current angle: {'{:.2f}'.format(self.data.angle * 180 / np.pi)} deg")
        print(f"Target position: {self.data.target_position} | "
              f"Target angle: {'{:.2f}'.format(self.data.target_angle * 180 / np.pi)} deg")
        print(f"Moves: {self.data.moves}")
        _input = input("\n[m]ove (cm) | [r]otate (angle)(rad) (rel/abs) | [p]osition (x) (y) (angle) | [g]rab (on/off)"
                       "\n[s]can (res) | [cal]ibrate (x=0) (y=0) (angle=0) | clear | auto | run | [h]ome | [q]uit"
                       "\n> ")

        try:
            return self._parse_input(_input)
        except Exception as e:  # capturing all exceptions like a gangsta
            print(f"Error occured parsing input: {e}")
            traceback.print_tb(e.__traceback__)
            return ManualState, self.data

    def _parse_input(self, _input) -> (str, 'BotData'):
        _input: List[str] = _input.lower().split(' ')
        _args = len(_input)

        if _input[0] in ['m', 'mv', 'move']:  # move by a specific distance
            self._move(_input)

        elif _input[0] in ['r', 'rot', 'rotate']:  # rotate by a specific angle
            self._rotate(_args, _input)

        elif _input[0] in ['p', 'pos', 'position']:  # move to a specific position (and angle)
            self._move_position(_args, _input)

        elif _input[0] in ['g', 'grab']:  # grab or release object
            self._grab(_args, _input)

        elif _input[0] in ['s', 'scan']:  # scan for objects, resolution is the number of angles to scan
            self._scan(_input)

        elif _input[0] in ['cal', 'calibrate']:  # calibrate position and angle, default is (0, 0) with angle 0
            self._calibrate_pos(_args, _input)

        elif _input[0] in ['clr', 'clear']:  # clear moves
            self._clear_moves()

        elif _input[0] in ['auto']:  # switch to auto state, which will automatically decide what to do
            return 'AutoState', self.data

        elif _input[0] in ['run']:  # run the moves
            return 'RunState', self.data

        elif _input[0] in ['h', 'home']:  # go to home position, which is (0, 0) with angle 0
            self._move_home()

        elif _input[0] in ['q', 'quit']:  # gracefully quit the program
            return None, self.data

        return 'ManualState', self.data

    def _move(self, _input):
        distance = float(_input[1])
        self.data.moves.append(Movement(distance))
        self.data.target_position += np.array((distance * np.cos(self.data.target_angle),
                                               distance * np.sin(self.data.target_angle)))

    def _rotate(self, _args, _input):
        if _args > 1 and _input[1].endswith('rad'):
            input_angle = float(_input[1][:-3])  # remove 'rad' from input
        else:
            input_angle = float(_input[1]) * np.pi / 180  # convert degrees to radians
        if _args > 2 and _input[2] == 'abs':
            angle = input_angle
        else:
            angle = self.data.target_angle + input_angle
        self.data.moves.append(Rotation(angle))
        self.data.target_angle = angle

    def _move_position(self, _args, _input):
        end_pos = np.array((float(_input[1]), float(_input[2])))
        end_angle = float(_input[3]) / 180 * np.pi if _args > 3 else None
        moves, end_pos, end_angle = Move.moves_to(self.data.target_position, self.data.target_angle,
                                                  end_pos, end_angle)
        self.data.moves.extend(moves)
        self.data.target_position, self.data.target_angle = end_pos, end_angle

    def _grab(self, _args, _input):
        if _args > 1 and _input[1] in ['on', 'yes', 'grab']:
            self.data.moves.append(Grab(True))
        elif _args > 1 and _input[1] in ['off', 'no', 'release']:
            self.data.moves.append(Grab(False))
        else:
            self.data.moves.append(Grab())

    def _scan(self, _input):
        self.data.moves.append(Scan(1, int(_input[1])))

    def _calibrate_pos(self, _args, _input):
        pos, angle = np.array((0.0, 0.0)), 0.0
        if _args > 1:
            pos = np.array((float(_input[1]), float(_input[2])))
            if _args > 3:
                angle = float(_input[3])
        self.data.position, self.data.angle = pos, angle

    def _clear_moves(self):
        self.data.moves = []
        self.data.target_position = self.data.position.copy()  # copy because python is fucked
        self.data.target_angle = self.data.angle  # don't copy here because why be consistent?

    def _move_home(self):
        moves = Move.moves_to(self.data.target_position, self.data.target_angle, np.array((0.0, 0.0)), 0.0)[0]
        self.data.moves.extend(moves)
        self.data.target_position, self.data.target_angle = np.array((0.0, 0.0)), 0.0


if __name__ == '__main__':
    from robot.robot import BotData, BotComponents

    # test the manual state
    data = BotData()
    components = BotComponents.default_components()
    state = ManualState
    while state is not None:
        _state = state(data, components)
        state, data = _state.execute()
