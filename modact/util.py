import math

from .actuator import Actuator
from .models import get_stepper, make_gearpair


def create_actuator_from_x(x, n_stages, with_3d):
    components = []
    ff, mot_sel = math.modf(x[0])
    ff = 0.3 + ff*0.9
    r_scale = x[1]
    stepper = get_stepper(mot_sel, ff, r_scale)
    components.append(stepper)
    gears = create_gears_from_x(x[2:], n_stages, with_3d)
    components.extend(gears)
    return Actuator(components=components)


def create_gears_from_x(x, n_stages, with_3d):
    gears = []
    steps = 6 if with_3d else 4

    for i in range(0, steps*n_stages, steps):
        x1, Z1 = math.modf(x[i])
        x1 = -0.1 + x1*0.7
        x2, Z2 = math.modf(x[i+1])
        x2 = -0.6 + x2*1.2
        m = x[i+2]
        b = x[i+3]

        disp = 0
        angle = None
        if with_3d:
            disp = x[i+4]
            angle = x[i+5]

        gears.append(make_gearpair(Z1, x1, Z2, x2, m, b, disp, angle))
    return gears
