import re
import typing

import attr
import numpy as np
import scipy.stats

from .models import OperatingCondition
from .models.motors import motor_names
from .util import create_actuator_from_x

op_set_1 = [
    OperatingCondition(speed=1.8, torque=0.8, V=9, imax=2.0),
    OperatingCondition(speed=0.3, torque=1.2, V=12, imax=2.0),
]

op_set_2 = [
    OperatingCondition(speed=1.35, torque=0.60, V=9, imax=2.0),
    OperatingCondition(speed=0.3, torque=1.0, V=12, imax=2.0),
]


class Objectives(object):
    # Problems are defined for maximization
    weights = tuple()
    ref = tuple()

    def __call__(self, *args, **kwargs):
        return tuple()


class Constraints(object):
    weights = tuple()

    def __call__(self, *args, **kwargs):
        return tuple()


class CT(Objectives):
    weights = (-1, 1)

    def __call__(self, actuator, output, t_err, kinematic, resistance):
        return [sum(actuator.cost(True)), min(t_err)]


class CS(Objectives):
    weights = (-1, 1)

    def __call__(self, actuator, output, t_err, kinematic, resistance):
        return [
            sum(actuator.cost(True)),
            scipy.stats.hmean(resistance[:, :, 2:4], axis=None),
        ]


class CTS(Objectives):
    weights = (-1, 1, 1)

    def __call__(self, actuator, output, t_err, kinematic, resistance):
        return [
            sum(actuator.cost(True)),
            min(t_err),
            scipy.stats.hmean(resistance[:, :, 2:4], axis=None),
        ]


class CTSE(Objectives):
    weights = (-1, 1, 1, 1)

    def __call__(self, actuator, output, t_err, kinematic, resistance):
        eff = [op.speed * op.torque / (op.imax * op.V) for op in output]
        return [
            sum(actuator.cost(True)),
            min(t_err),
            scipy.stats.hmean(resistance[:, :, 2:4], axis=None),
            min(eff),
        ]


class CTSEI(Objectives):
    weights = (-1, 1, 1, 1, -1)

    def __call__(self, actuator, output, t_err, kinematic, resistance):
        eff = [op.speed * op.torque / (op.imax * op.V) for op in output]
        return [
            sum(actuator.cost(True)),
            min(t_err),
            scipy.stats.hmean(resistance[:, :, 2:4], axis=None),
            min(eff),
            actuator.i_gp,
        ]


class C1(Constraints):
    weights = (-1, -1, -1, -1, -1, -1, -1)

    def __init__(self, min_t):
        self.min_t = min_t

    def __call__(self, actuator, output, t_err, kinematic, resistance):
        gkconsts = kinematic.min(axis=0)
        gkconsts[1:] /= [1.1, 5, 5]
        gkconsts[1:] += [-1, 1, 1]
        min_h = resistance[:, :, :2].min() - 1
        min_f = resistance[:, :, 2:4].min() - 1
        min_t_err = min(t_err) + self.min_t
        return (*gkconsts, min_h, min_f, min_t_err)


class C2(C1):
    weights = C1.weights + (1,)

    def __call__(self, actuator, *args):
        csts = super().__call__(actuator, *args)
        return csts + (actuator.internal_collisions(),)


class C3(C2):
    weights = C2.weights + (1, 1)

    def __call__(self, actuator, *args):
        csts = super().__call__(actuator, *args)
        _, _, space = actuator.mesh
        bb_bounds = space.bounding_box.extents[1:] / [50.0, 35.0] - 1
        return (*csts, *bb_bounds)


class C4(C2):
    weights = C2.weights + (1,)

    def __call__(self, actuator, *args):
        csts = super().__call__(actuator, *args)
        meshes, _, _ = actuator.mesh
        output = np.linalg.norm(meshes[-1].primitive.transform[:2, 3] - [40.0, 0.0])
        output = max(0, output - 0.5)
        return csts + (output / 10.0,)


class C5(C2):
    """Combine all constraints, only for analysis"""

    weights = C2.weights + (1, 1, 1)

    def __call__(self, actuator, *args):
        csts = super().__call__(actuator, *args)
        meshes, _, space = actuator.mesh
        bb_bounds = space.bounding_box.extents[1:] / [50.0, 35.0] - 1
        output = np.linalg.norm(meshes[-1].primitive.transform[:2, 3] - [40.0, 0.0])
        output = max(0, output - 0.5)
        return csts + (
            *bb_bounds,
            output / 10.0,
        )


@attr.s(auto_attribs=True)
class Problem(object):
    name: str
    op: typing.List[OperatingCondition]
    objectives: Objectives
    constraints: Constraints
    n_stages: int

    @property
    def weights(self):
        return self.objectives.weights

    @property
    def ref(self):
        return (11.0,) * len(self.objectives.weights)

    @property
    def c_weights(self):
        return self.constraints.weights

    def bounds(self):
        lb = [0, 0.3]
        ub = [len(motor_names) - 1e-6, 2.0]
        for _ in range(self.n_stages):
            lb.extend([9, 30, 0.3, 5.0, -20, -np.pi])
            ub.extend([41 - 1e-6, 81 - 1e-6, 1.0, 15.0, 20, np.pi])
        return np.array(lb), np.array(ub)

    def prepare(self, x):
        actuator = create_actuator_from_x(x, self.n_stages, True)
        control = actuator.matched_speed_control(self.op)
        output, op_per_comp = actuator.get_speed_torque(control)
        kinematic, resistance = actuator.gear_constraints(op_per_comp)
        t_err = [op.torque - op_t.torque for op_t, op in zip(self.op, output)]
        return actuator, output, t_err, kinematic, resistance

    def __call__(self, x):
        actuator, output, t_err, kinematic, resistance = self.prepare(x)
        obj = self.objectives(actuator, output, t_err, kinematic, resistance)
        csts = self.constraints(actuator, output, t_err, kinematic, resistance)
        return (obj, csts)


OBJECTIVES = {"CS": CS, "CT": CT, "CTS": CTS, "CTSE": CTSE, "CTSEI": CTSEI}


CONSTRAINTS = {"C1": C1, "C2": C2, "C3": C3, "C4": C4, "C5": C5}


def get_problem(name, op_set=op_set_2):
    m = re.match(r"^(c(t|s)s?e?i?)([1-9])(s[1-9])?$", name)
    if m is None:
        raise NotImplementedError("Unable to parse {}".format(name))

    o_name = m.group(1).upper()
    o = OBJECTIVES[o_name]()

    if "T" not in o_name:
        min_t = 0 + 0.001
    else:
        min_t = min([op.torque for op in op_set]) - 0.001
    c_name = "C" + m.group(3)
    c = CONSTRAINTS[c_name](min_t)

    n_stages = int(m.group(4)[1]) if m.group(4) else 3

    prob = Problem(name=name, objectives=o, constraints=c, n_stages=n_stages, op=op_set)
    return prob
