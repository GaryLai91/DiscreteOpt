#!/usr/local/bin pypy3

import pyomo.environ as pyo
from pyomo.opt import SolverFactory, SolverStatus
import math
import numpy as np
import itertools


class VehicleRoutingSolver:
    def __init__(self, customers, vehicle_count, vehicle_capacity):
        self.customers = customers
        self.vehicle_count = vehicle_count
        self.vehicle_capacity = vehicle_capacity
        self.dist_matrix = self._compute_distance_matrix()

    def _length(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def _compute_distance_matrix(self):
        matrix = {}
        for i in self.customers:
            for j in self.customers:
                if i.index != j.index:
                    matrix[i.index, j.index] = self._length(i, j)
                else:
                    matrix[i.index, j.index] = 0
        return matrix

    def _objective_value(self, model):
        """Objective function"""
        return sum(
            model.x[i, j, k] * model.c[i, j] if i != j else 0
            for k in model.K
            for i in model.I
            for j in model.J
            if i != j
        )

    def _visits_city(self, model, J):
        """Exactly 1 visit per vehicle per customer"""
        return (
            sum(model.x[i, J, k] if i != J else 0 for i in model.I for k in model.K)
            == 1
        )

    def _depart_from_depot(self, model, K):
        """Vehicle depart from depot"""
        return sum(model.x[0, j, K] for j in model.J if j != 0) == 1

    def _enter_to_depot(self, model, K):
        """Vehicle enter to depot"""
        return sum(model.x[i, 0, K] for i in model.I if i != 0) == 1

    def _vehicle_visits(self, model, K, J):
        """Number of vehicle visiting a customer is the same"""
        sum_one = sum(model.x[i, J, K] for i in model.I)
        sum_two = sum(model.x[J, i, K] for i in model.I)
        return sum_one - sum_two == 0

    def _delivery_capacity(self, model, K):
        """Delivery capacity of each vehicle
        does not exceed max capacity"""
        return (
            sum(
                model.d[j] * model.x[i, j, K] if i != j else 0
                for i in model.I
                for j in model.J
            )
            <= self.vehicle_capacity
        )

    def _subtour_elim(self, model):
        subtours = []
        for i in range(2, len(self.customers)):
            subtours.append(itertools.combinations(range(1, len(self.customers)), i))
        return

    def solve(self):
        # for k in range(1, self.vehicle_count+1):
        for k in range(self.vehicle_count, 0, -1):
            model = pyo.ConcreteModel()

            model.I = pyo.RangeSet(0, len(self.customers) - 1)
            model.J = pyo.RangeSet(0, len(self.customers) - 1)
            model.K = pyo.RangeSet(1, k)

            # Descision Variables
            model.x = pyo.Var(
                model.I, model.J, model.K, within=pyo.Binary, initialize=0
            )

            # Cost Matrix
            model.c = pyo.Param(
                model.I, model.J, initialize=lambda model, i, j: self.dist_matrix[i, j]
            )

            # Demand Matrix
            model.d = pyo.Param(
                model.J, initialize=lambda model, j: self.customers[j].demand
            )

            # Objective value
            model.objective = pyo.Objective(
                rule=self._objective_value, sense=pyo.minimize
            )

            # Constraints
            model.con1 = pyo.Constraint(
                model.I - pyo.Set(initialize=[0]), rule=self._visits_city
            )
            model.con2 = pyo.Constraint(model.K, rule=self._depart_from_depot)
            model.con3 = pyo.Constraint(model.K, rule=self._enter_to_depot)
            model.con4 = pyo.Constraint(model.K, model.J, rule=self._vehicle_visits)
            model.con5 = pyo.Constraint(model.K, rule=self._delivery_capacity)

            # Subtour elimination
            model.subtour_elim = pyo.ConstraintList()
            subtours = []
            for i in range(2, len(self.customers)):
                subtours += itertools.combinations(range(1, len(self.customers)), i)

            for s in subtours:
                model.subtour_elim.add(
                    sum(
                        model.x[i, j, k_] if i != j else 0
                        for i, j in itertools.permutations(s, 2)
                        for k_ in range(1, k + 1)
                    )
                    <= len(s) - 1
                )

            # solve
            opt = SolverFactory("glpk")
            # opt.options["threads"] = 10
            # opt.options['timelimit'] = 1800
            result_obj = opt.solve(model, tee=False)

            if result_obj.solver.status == SolverStatus.ok:
                # model.pprint()
                obj_value = pyo.value(model.objective)
                # print(f"Number Vehicle {k} has objective value of {obj_value}.")
                # des_var = [[pyo.value(model.x[i, j, k]) for i in model.I for j in model.J] for k in model.K]
                # print(des_var)
                break
            """
            else:
                print("------------------------------------\n")
                print(f"Number Vehicle {k} has no solution.")
                print("------------------------------------\n")
            """
        obj_value = pyo.value(model.objective)
        # des_var = [[pyo.value(model.x[i, j, k]) for i in model.I for j in model.J] for k in model.K]
        des_var = [
            [
                (i, j)
                for i in model.I
                for j in model.J
                if round(pyo.value(model.x[i, j, k])) == 1
            ]
            for k in model.K
        ]
        output = ""
        for v_seq in des_var:
            v_seq = sorted(v_seq)
            v_seq_dict = dict(v_seq)
            first_elem = v_seq[0][0]
            seq = []
            for _ in range(len(v_seq)):
                seq.append((first_elem, v_seq_dict[first_elem]))
                first_elem = v_seq_dict[first_elem]
            str_seq = "0"
            for i in seq:
                str_seq += " " + str(i[1])
            str_seq += "\n"
            output += str_seq
        for empty_vehicle in range(self.vehicle_count - len(des_var)):
            output += "0 0\n"
        return obj_value, output

