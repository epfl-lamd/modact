from modact.util import create_actuator_from_x, create_gears_from_x


def test_create_gears_with_3d():
    x = [17.5, 80.5, 1.0, 7.0, 5.0, 0.0, 20.5, 40.5, 1.0, 7.0, 10.0, 0.0]
    gears = create_gears_from_x(x, 2, True)
    assert len(gears) == 2
    assert gears[0].gears.p.Z == 17
    assert abs(gears[0].gears.p.x - 0.25) < 1e-6
    assert gears[0].gears.g.Z == 80
    assert abs(gears[0].gears.g.x - 0) < 1e-6
    assert gears[0].disp == 5.0
    assert gears[0].gears.p.stretch == 5.0 - gears[0].stretch_margin
    assert gears[1].disp == 10.0


def test_create_gears_without_3d():
    x = [17.5, 80.5, 1.0, 7.0, 20.5, 40.5, 1.0, 7.0]
    gears = create_gears_from_x(x, 2, False)
    assert len(gears) == 2
    assert gears[1].gears.p.Z == 20
    assert abs(gears[1].gears.p.x - 0.25) < 1e-6
    assert gears[1].gears.g.Z == 40
    assert abs(gears[1].gears.g.x - 0) < 1e-6
    assert gears[0].disp == 0
    assert gears[1].disp == 0
    assert gears[0].angle is None
    assert gears[1].angle is None


def test_create_actuator():
    x = [0.0, 1.0, 20.5, 80.5, 1.0, 7.0, 20.5, 40.5, 1.0, 7.0]
    actuator = create_actuator_from_x(x, 2, False)
    assert len(actuator.components) == 3
    assert actuator.i == 40
