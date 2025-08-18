"""run_nsga2.py

Simple example to demonstrate interface with pymop and pymoo.

Optimizes problem CS2 with NSGA-II (serial).

$ python scripts/run_nsga2.py
"""

from pymoo.algorithms.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter

from modact.interfaces.pymoo import PymopProblem

problem = PymopProblem("cs2")

algorithm = NSGA2(pop_size=100, eliminate_duplicates=True)

res = minimize(problem, algorithm, ("n_gen", 200), seed=1, verbose=True)

plot = Scatter()
plot.add(problem.pareto_front(), plot_type="line", color="black", alpha=0.7)
plot.add(res.F, color="red")
plot.show()
