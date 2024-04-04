from typing import TYPE_CHECKING, Type

from robot.states import State

if TYPE_CHECKING:
    from robot.robot import BotData
    
    
# UNIMPLEMENTED


class AutoState(State):
    def __init__(self, data, components):
        super().__init__(data, components)

    def allow_transition(self, from_state) -> bool:
        return True

    def execute(self) -> (str, 'BotData'):
        return None, self.data
