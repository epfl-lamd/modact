import pytest

from modact.models import OperatingCondition


def test_operating_condition_operations():
    op1 = OperatingCondition(4.0 / 3.0, 0.24, 12, 0.3)
    op2 = OperatingCondition(8.0 / 3.0, 0.24, 12, 0.3)

    op3 = op1 + op2
    assert op3.speed == op1.speed + op2.speed
    assert op3.V == op1.V == op2.V
    assert op3.imax == op1.imax == op2.imax
    assert op3.torque == op1.torque + op2.torque

    op4 = op1 - op2
    assert op4.speed == -4.0 / 3.0
    assert op4.torque == 0

    op5 = OperatingCondition(8.0 / 3.0, 0.24, 10, 0.3)
    with pytest.raises(ValueError):
        op1 + op5
    with pytest.raises(ValueError):
        op1 - op5
