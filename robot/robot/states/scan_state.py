from typing import TYPE_CHECKING

from robot.states import State

if TYPE_CHECKING:
    from robot.robot import BotData
    
    
# NOT IMPLEMENTED
# a testing script for this state is available in the scripts/ directory


class ScanState(State):
    def __init__(self, data, components):
        super().__init__(data, components)

    def allow_transition(self, from_state) -> bool:
        return True

    def execute(self) -> (str, 'BotData'):
        self.scan()
        return 'RunState', self.data

    def scan(self):
        print(f"Scanning at {self.data.position}")
