import pyomo.environ as pyo
from pyomo.opt import SolverFactory
import math
import numpy as np


class FacilitySolver:

    def __init__(self, facility, customer):
        self.facility = {f.index: f for f in facility}
        self.customer = {c.index: c for c in customer}
        self.facility_var = [k for k in self.facility.keys()]
        self.customer_var = [c for c in self.customer.keys()]
        self.dist_matrix = self._compute_distance_matrix()
    

    def _length(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    

    def _compute_distance_matrix(self):
        matrix = {}
        for f in self.facility:
            for c in self.customer:
                matrix[f, c] = self._length(self.facility[f].location, self.customer[c].location)
        return matrix


    def _objective_value(self, m):
        # Define objective value to minimize fixed cost
        # m: pyo model object
        cost = sum(self.facility[f].setup_cost * m.facility_var[f] for f in self.facility_var)

        # Define objective value to minimize total distance
        # between facility and customer
        dist = sum(self.dist_matrix[f, c] * m.customer_var[f, c] for f in self.facility_var for c in self.customer_var)
        return cost + dist
    

    def _demand_constraint(self, m, f):
        return sum(m.customer_var[f, c] * self.customer[c].demand 
                for c in self.customer_var) <= self.facility[f].capacity

    # Can try to remove this constraint
    def _customer_constraint(self, m, c):
        return sum(m.customer_var[f, c] for f in self.facility_var) == 1

    def _facility_constraint(self, m, f, c):
        return m.customer_var[f,c] <= m.facility_var[f]
    
    def _facility_constraint_v2(self, m, f):
        return sum(m.customer_var[f, c] for c in self.customer_var) <= len(self.customer_var) * m.facility_var[f]


    def solve(self):
        model = pyo.ConcreteModel()

        # Define facility decision variables
        model.facility_var = pyo.Var(self.facility_var, within=pyo.Binary)

        # Define customer variables
        model.customer_var = pyo.Var(self.facility_var, self.customer_var, within=pyo.Binary)

        # Assign objective value
        model.obj = pyo.Objective(
            rule = self._objective_value,
            sense = pyo.minimize
        )

        # Set demand constraint
        model.demand_c = pyo.Constraint(self.facility_var, rule = self._demand_constraint)

        # Set customer constraint
        model.customer_c = pyo.Constraint(self.customer_var, rule = self._customer_constraint)
        
        # Set facility constraint
        model.facility_c = pyo.Constraint(self.facility_var, self.customer_var, rule = self._facility_constraint)

        # solve
        opt = SolverFactory('glpk')
        result_obj = opt.solve(model)

        #model.pprint()

        obj_value = pyo.value(model.obj)
        cust_idx = [pyo.value(model.customer_var[f,c]) for f in self.facility_var for c in self.customer_var]
        cust_idx = np.resize(cust_idx, (len(self.facility_var), len(self.customer_var)))
        result = np.argmax(cust_idx, axis=0)
        return obj_value, result

        
        



        

