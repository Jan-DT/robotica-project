import abc
from typing import TYPE_CHECKING, Type

if TYPE_CHECKING:
    from robot.robot import BotData, BotComponents


class State(abc.ABC):
    def __init__(self, data: 'BotData', components: 'BotComponents'):
        self.data = data
        self.components = components

    @abc.abstractmethod
    def allow_transition(self, from_state) -> bool:
        pass

    @abc.abstractmethod
    def execute(self) -> (str, 'BotData'):
        pass
