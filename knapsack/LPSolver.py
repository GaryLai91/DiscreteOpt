import pyomo.environ as pyo
from pyomo.opt import SolverFactory


class KnapsackSolver():

    def __init__(self, value, weight, max_capacity, solver='glpk'):
        self.value = value
        self.weight = weight
        self.max_capacity = max_capacity
        self.solver = solver
    
    def solve(self):
        model = pyo.ConcreteModel()
        var_index = [i for i in range(len(self.value))]
        model.decision_var = pyo.Var(var_index, within=pyo.Binary)
        model.value = pyo.Objective(
            expr = sum(self.value[i] * model.decision_var[i] for i in var_index),
            sense = pyo.maximize
        )
        model.weight = pyo.Constraint(
            expr = sum(self.weight[i] * model.decision_var[i] for i in var_index) <= self.max_capacity
        )
        opt = SolverFactory(self.solver)
        result_obj = opt.solve(model)
        result = [int(pyo.value(model.decision_var[i])) for i in var_index]
        obj_value = int(pyo.value(model.value))
        return obj_value,result