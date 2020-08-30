import numpy as np
from pymoo.model.problem import Problem

import modact.problems as pb


class PymopProblem(Problem):

    def __init__(self, function, **kwargs):
        
        if isinstance(function, pb.Problem):
            self.fct = function
        else:
            self.fct = pb.get_problem(function)
        lb, ub = self.fct.bounds()
        n_var = len(lb)
        n_obj = len(self.fct.weights)
        n_constr = len(self.fct.c_weights)
        xl = lb
        xu = ub

        self.weights = np.array(self.fct.weights)
        self.c_weights = np.array(self.fct.c_weights)

        super().__init__(n_var=n_var, n_obj=n_obj, n_constr=n_constr, xl=xl,
                         xu=xu, elementwise_evaluation=True, type_var=np.double,
                         **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):
        f, g = self.fct(x)
        out["F"] = np.array(f)*-1*self.weights
        out["G"] = np.array(g)*self.c_weights
