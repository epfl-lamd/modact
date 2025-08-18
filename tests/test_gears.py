import numpy as np

from modact.models import OperatingCondition
from modact.models.gears import GearPair, SpurGear, make_gearpair


def test_basic_gear_operations():
    pinion = SpurGear(25, 1.0, 0.0, 10)
    gear = SpurGear(80, 1.0, 0.0, 10)
    gp = GearPair(pinion, gear)
    assert abs(gear.alpha_p - np.pi / 9) < 1e-5
    assert abs(gp.contact_ratio - 1.719) < 1e-3
    assert gp.interference == 0
    excepted_volume = (25e-3 / 2) ** 2 * np.pi * 10e-3 + (
        80e-3 / 2
    ) ** 2 * np.pi * 10e-3
    assert abs(gp.volume - excepted_volume) / excepted_volume < 1e-5

    gs = gp.specific_speed
    assert abs(gs[0] + 2.238193) < 1e-5
    assert abs(gs[1] + 0.884045) < 1e-5

    op = OperatingCondition(1000, 0.318, 12, 0.3)

    # Because dynamic factors are set to 1.
    # speed should not change stress
    assert gp.sigma_h(1000, 0.318) == gp.sigma_h(100, 0.318)

    sigF0, sigF = gp.sigma_f(1000, 0.318)
    assert abs(sigF0[0] - 7.16e6) / 7.16e6 < 0.06
    assert abs(sigF0[1] - 7.03e6) / 7.03e6 < 0.06
    assert sigF0[0] > sigF0[1]
    assert sigF[0] / sigF0[0] == 1.25

    sigH0, sigH = gp.sigma_h(1000, 0.318)
    assert abs(sigH0 - 150.9e6) / 150.9e6 < 0.01
    assert abs(sigH[0] - 177.62e6) / 177.62e6 < 0.01
    assert abs(sigH[1] - 168.71e6) / 168.71e6 < 0.01
    gp.security_h(op)
    gp.security_f(op)


def test_make_method():
    gp = make_gearpair(25, 0, 80, 0, 1.0, 10)
    assert gp.gears.p.Z == 25
    assert gp.gears.g.Z == 80
    assert gp.gears.p.m == 1.0
    assert gp.gears.g.m == 1.0
