import time
from typing import TYPE_CHECKING

from robot.states import State

if TYPE_CHECKING:
    from robot.robot import BotData


class GrabState(State):
    def __init__(self, data, components):
        super().__init__(data, components)
        
        # if the state is not specified, default to the opposite of the current grabber state
        self.state = self.data.current_move.params['state']
        self.state = self.state if self.state is not None else not self.data.grabber

    def allow_transition(self, from_state) -> bool:
        return True

    def execute(self) -> (str, 'BotData'):
        self.grab(self.state)

        return 'RunState', self.data

    def grab(self, state: bool):
        if state:
            self.components.servo.set_angle(200)
        else:
            self.components.servo.set_angle(90)
        self.data.grabber = state
