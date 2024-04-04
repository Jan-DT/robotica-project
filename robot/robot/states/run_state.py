from robot.movement import MoveType
from robot.states import State


class RunState(State):
    def __init__(self, data, components):
        super().__init__(data, components)

    def allow_transition(self, from_state) -> bool:
        return True

    def execute(self):
        self.data.current_move = self.data.moves.pop()
        if self.data.current_move.move_type == MoveType.FORWARD:
            return 'MoveState', self.data
        elif self.data.current_move.move_type == MoveType.ROTATION:
            return 'MoveState', self.data
        elif self.data.current_move.move_type == MoveType.GRAB:
            return 'GrabState', self.data
        elif self.data.current_move.move_type == MoveType.SCAN:
            return 'ScanState', self.data

        return 'ManualState', self.data
