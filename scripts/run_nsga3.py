"""run_nsga2.py

Simple example to demonstrate interface with pymop and pymoo.

Optimizes problem CTS2 with NSGA-III (parallel execution with dask).

$ python scripts/run_nsga3.py
"""

import multiprocessing

from pymoo.algorithms.nsga3 import NSGA3
from pymoo.factory import get_reference_directions
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter

from modact.interfaces.pymoo import PymopProblem


if __name__ == "__main__":
    n_proccess = 8
    pool = multiprocessing.Pool(n_proccess)
    problem = PymopProblem("cts2", parallelization=("starmap", pool.starmap))

    mu = 92
    ref_dirs = get_reference_directions("das-dennis", problem.n_obj, n_partitions=12)

    algorithm = NSGA3(ref_dirs, pop_size=mu, eliminate_duplicates=True)

    res = minimize(problem, algorithm, ("n_gen", 200), seed=1, verbose=True)

    plot = Scatter()
    plot.add(problem.pareto_front(), plot_type="line", color="black", alpha=0.7)
    plot.add(res.F, color="red")
    plot.save("test.png")
