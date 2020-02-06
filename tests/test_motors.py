from modact.models.motors import get_stepper, motor_names


def test_get_stepper_with_number():
    mot1 = get_stepper(0)
    assert mot1.name == motor_names[0]

    mot2 = get_stepper(motor_names[0])
    assert mot1.name == mot2.name
    assert mot2.fill_factor == mot2.r_scale == 1


def test_get_stepper_configured():
    mot1 = get_stepper(0, 0.5, 1.2)
    assert mot1.fill_factor == 0.5
    assert mot1.r_scale == 1.2
