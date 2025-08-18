"""info.py
Get basic information for a given problem.

$ python scripts/info.py cs3
"""

import sys

import modact.problems as pb


if __name__ == "__main__":
    args = sys.argv
    problem = pb.get_problem(args[1].lower())
    print("Problem: {}".format(problem.name))

    lb, ub = problem.bounds()
    print("Bounds (nvars={})".format(len(lb)))
    print(lb)
    print(ub)

    print("Nobjs: {}".format(len(problem.weights)))
    print("Nconsts: {}".format(len(problem.c_weights)))
