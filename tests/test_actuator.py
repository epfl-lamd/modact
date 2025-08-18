from math import pi

import numpy as np
import pytest

from modact.actuator import Actuator
from modact.models import GearPair, OperatingCondition, Stepper
from modact.models.gears import SpurGear
from modact.models.motors import motor_data


@pytest.fixture(scope="function")
def linear_2_stages():
    g1 = SpurGear(10, 0.5, 0, 8)
    g2 = SpurGear(50, 0.5, 0, 8)
    g3 = SpurGear(13, 0.5, 0, 12)
    g4 = SpurGear(78, 0.5, 0, 12)
    a = Actuator(components=[GearPair(g1, g2, 0, 0), GearPair(g3, g4, 10, 0)])
    return a


@pytest.fixture(scope="function")
def motored_2_stages():
    s = Stepper("A", motor_data["A"])
    g1 = SpurGear(10, 0.8, 0, 8)
    g2 = SpurGear(50, 0.8, 0, 8)
    g3 = SpurGear(13, 0.5, 0, 12)
    g4 = SpurGear(78, 0.5, 0, 12)
    comp = [s, GearPair(g1, g2, 10, 0), GearPair(g3, g4, 10, -2.3)]
    a = Actuator(components=comp)
    return a


@pytest.fixture(scope="function")
def good_motored_2_stages():
    s = Stepper("A", motor_data["A"])
    g1 = SpurGear(17, 0.5, 0.15, 8)
    g2 = SpurGear(60, 0.5, -0.15, 8)
    g3 = SpurGear(17, 0.5, 0.15, 12)
    g4 = SpurGear(78, 0.5, -0.15, 12)
    comp = [s, GearPair(g1, g2, 10, 0), GearPair(g3, g4, 10, -2.3)]
    a = Actuator(components=comp)
    return a


@pytest.fixture(scope="function")
def broken_motored_2_stages():
    s = Stepper("A", motor_data["A"])
    g1 = SpurGear(10, 0.8, 0, 8)
    g2 = SpurGear(50, 0.8, 0, 8)
    g3 = SpurGear(13, 0.5, 0, 12)
    g4 = SpurGear(80, 0.5, 0, 12)
    comp = [s, GearPair(g1, g2, 10, 0), GearPair(g3, g4, -3, -2.3)]
    return Actuator(components=comp)


@pytest.fixture(scope="function")
def impossible_motored_2_stages():
    s = Stepper("B", motor_data["B"])
    g1 = SpurGear(10, 0.8, 0, 8)
    g2 = SpurGear(35, 0.8, 0, 8)
    g3 = SpurGear(25, 0.5, 0, 12)
    g4 = SpurGear(80, 0.5, 0, 12)
    comp = [s, GearPair(g1, g2, 10, 0), GearPair(g3, g4, -30, -2.3)]
    a = Actuator(components=comp)
    return a


def test_mesh_generation(linear_2_stages, impossible_motored_2_stages):
    space = linear_2_stages.mesh[2]
    assert np.allclose(
        space.bounding_box.extents, (2.5 + 15 + 91 / 4 + 78 / 4, 78 / 2, 20.0), atol=0.2
    )
    space = impossible_motored_2_stages.mesh[2]
    assert abs(space.bounding_box.extents[2] - (30 + 6.4 + 6.0)) <= 0.001


def test_matched_speed_conditions(linear_2_stages):
    assert linear_2_stages.i == 30
    conditions = [
        OperatingCondition(4.0 / 3.0, 0.24, 12, 0.3),
        OperatingCondition(8.0 / 3.0, 0.24, 12, 0.3),
    ]
    control = linear_2_stages.matched_speed_control(conditions)
    assert len(control) == 2
    assert control[0].speed == 40.0
    assert control[0].torque == 0.0
    assert control[1].speed == 80.0
    assert control[1].torque == 0.0
    assert control[1].V == 12.0


def test_speed_torque_with_stepper(motored_2_stages):
    conditions = [OperatingCondition(4.0 * pi / 30.0, 0.24, 12, 0.3)]
    control = motored_2_stages.matched_speed_control(conditions)
    out, per_component = motored_2_stages.get_speed_torque(control)
    assert len(out) == 1
    assert len(per_component) == 3

    assert out[0].speed == 4.0 * pi / 30.0
    assert out[0].torque > 0.24

    out, per_component = motored_2_stages.get_speed_torque(control, target=conditions)
    diff = out[0] - conditions[0]
    assert abs(diff.speed) <= 1e-8
    assert abs(diff.torque) <= 1e-8


def test_constraints(good_motored_2_stages):
    conditions = [OperatingCondition(4.0 * pi / 30.0, 0.24, 12, 0.3)]
    control = good_motored_2_stages.matched_speed_control(conditions)
    _, per_component = good_motored_2_stages.get_speed_torque(control)
    kinematic, resistance = good_motored_2_stages.gear_constraints(per_component)

    assert kinematic.shape == (2, 4)
    assert np.all(kinematic[:, 0] == 0)
    assert np.all(kinematic[:, 1] >= 1.4)
    assert np.all(kinematic[:, 2:4] > -5)

    assert resistance.shape == (2, 1, 4)
    assert np.all(resistance > 1.0)


def test_get_actuator_cost(motored_2_stages):
    cost = motored_2_stages.cost()
    assert len(cost) == 3
    assert sum(cost) > 0
    cost_wh = motored_2_stages.cost(with_hull=True)
    assert len(cost_wh) == 4
    assert cost[0] == cost_wh[0]


def test_internal_collision_detection(
    motored_2_stages, broken_motored_2_stages, impossible_motored_2_stages
):
    assert motored_2_stages.internal_collisions() == 0
    assert broken_motored_2_stages.internal_collisions() > 0
    assert impossible_motored_2_stages.internal_collisions() > 0


def test_gear_mesh_plus_stretch():
    g1 = SpurGear(25, 0.5, 0, 10)
    g2 = SpurGear(80, 0.5, 0, 10)
    gp = GearPair(g1, g2, 10, 0)
    actuator = Actuator(components=[gp])
    meshes, _, space = actuator.mesh
    assert np.allclose(space.bounds[:, 2], [gp.stretch_margin, 15.0])

    assert np.allclose(meshes[0].bounds[:, 2], [gp.stretch_margin, 15.0])
    assert np.allclose(meshes[1].bounds[:, 2], [10.0, 15.0])


def test_gear_mesh_minus_stretch():
    g1 = SpurGear(25, 0.5, 0, 10)
    g2 = SpurGear(80, 0.5, 0, 10)
    gp = GearPair(g1, g2, -10, 0)
    actuator = Actuator(components=[gp])
    meshes, _, space = actuator.mesh
    assert np.allclose(space.bounds[:, 2], [-15.0, -gp.stretch_margin])

    assert np.allclose(meshes[0].bounds[:, 2], [-15.0, -gp.stretch_margin])
    assert np.allclose(meshes[1].bounds[:, 2], [-15.0, -10.0])


def test_gear_mesh_small_stretch():
    g1 = SpurGear(25, 0.5, 0, 10)
    g2 = SpurGear(80, 0.5, 0, 10)
    gp = GearPair(g1, g2, GearPair.stretch_margin / 2, 0)
    actuator = Actuator(components=[gp])
    meshes, _, space = actuator.mesh
    assert np.allclose(space.bounds[:, 2], [gp.disp, gp.disp + 5.0])

    assert np.allclose(meshes[0].bounds[:, 2], [gp.disp, gp.disp + 5.0])
    assert np.allclose(meshes[1].bounds[:, 2], [gp.disp, gp.disp + 5.0])


def test_gear_mesh_small_minus_stretch():
    g1 = SpurGear(25, 0.5, 0, 10)
    g2 = SpurGear(80, 0.5, 0, 10)
    gp = GearPair(g1, g2, -GearPair.stretch_margin / 2, 0)
    actuator = Actuator(components=[gp])
    meshes, _, space = actuator.mesh
    print(space.bounds)
    assert np.allclose(space.bounds[:, 2], [gp.disp - 5.0, gp.disp])

    assert np.allclose(meshes[0].bounds[:, 2], [gp.disp - 5.0, gp.disp])
    assert np.allclose(meshes[1].bounds[:, 2], [gp.disp - 5.0, gp.disp])
