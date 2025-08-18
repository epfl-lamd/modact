import modact.problems as pb


def test_abstract_problems():
    p = pb.Problem("abstract", pb.op_set_2, pb.Objectives(), pb.Constraints(), 2)
    lb, ub = p.bounds()
    assert len(lb) == len(ub) == 14
