import copy
from typing import Any

import attr


class Model(object):
    pass


@attr.s(auto_attribs=True, slots=True)
class OperatingCondition(object):
    speed: Any
    torque: Any
    V: float
    imax: float

    def __add__(self, other):
        if self.V != other.V:
            raise ValueError("Cannot sum conditions")
        new = copy.copy(self)
        new.speed = self.speed + other.speed
        new.torque = self.torque + other.torque
        new.imax = max(self.imax, other.imax)
        return new

    def __sub__(self, other):
        if self.V != other.V:
            raise ValueError("Cannot substract conditions")
        new = copy.copy(self)
        new.speed = self.speed - other.speed
        new.torque = self.torque - other.torque
        new.imax = min(self.imax, other.imax)
        return new

    def copy(self):
        return copy.copy(self)
