import os
import sys
import numpy as np
from synthesize_dtree import CONFIG_DICT_KEYS, CONFIG_KEY_BOUNDS
import z3
import gurobipy as gp
from gurobipy import GRB
import json
from typing import Callable as function
import itertools
from learner import DTreeLearner
from configtester import ConfigTester

CONFIG_EPSILONS = {
    "cloudiness": 0.5,
    "precipitation": 0.5,
    "precipitation_deposits": 0.5,
    "wind_intensity": 0.5,
    "sun_azimuth_angle": 1.0,
    "sun_altitude_angle": 1.0,
    "npc_speed": 0.25,
}
MINUS_INF = -1e10
PLUS_INF = 1e10


class DTreeTester:
    def __init__(self):
        pass

    def candidate_to_conjuncts(self, candidate: z3.BoolRef):
        init_path = [candidate]
        stack = [init_path]
        while stack:
            curr_path = stack.pop()  # remove this path
            curr_node = curr_path.pop()  # remove last node in this path
            if z3.is_false(curr_node):
                # the leaf node in this path is false. Skip
                continue
            elif z3.is_true(curr_node):
                if not curr_path:
                    yield z3.BoolVal(True)
                else:
                    yield z3.And(*curr_path)
            elif (
                z3.is_gt(curr_node)
                or z3.is_ge(curr_node)
                or z3.is_lt(curr_node)
                or z3.is_le(curr_node)
            ):
                yield z3.And(*curr_path, curr_node)
            elif z3.is_app_of(curr_node, z3.Z3_OP_ITE):
                cond, left, right = curr_node.children()
                l_path = curr_path.copy()
                l_path.extend([cond, left])

                r_path = curr_path.copy()
                assert len(cond.children()) == 2
                lhs, rhs = cond.children()
                if z3.is_le(cond):
                    not_cond = lhs > rhs
                elif z3.is_ge(cond):
                    not_cond = lhs < rhs
                else:
                    raise RuntimeError(f"Unexpected condition {cond} for ITE")
                r_path.extend([not_cond, right])

                stack.append(r_path)
                stack.append(l_path)
            else:
                raise RuntimeError(
                    f"Candidate formula {curr_node} should have been converted to DNF."
                )

    def z3_to_gurobi(self, conjunct: z3.BoolRef, model: gp.Model, var_map: dict):
        """
        Each clause must be a linear comparison (<=, >=, <, >, =).
        """

        def to_float(num: z3.ArithRef) -> float:
            if z3.is_int_value(num):
                return float(num.as_long())
            if z3.is_rational_value(num):
                return float(num.numerator_as_long()) / float(num.denominator_as_long())
            try:
                return float(str(num))
            except Exception:
                return float(num.as_decimal(10).rstrip("?"))

        def encode_expr(expr: z3.ArithRef):
            if z3.is_add(expr):
                return gp.quicksum(encode_expr(c) for c in expr.children())

            elif z3.is_sub(expr):
                a, b = expr.children()
                return encode_expr(a) - encode_expr(b)

            elif z3.is_mul(expr):
                a, b = expr.children()
                # Only one term can be numeric in linear exprs
                if z3.is_int_value(a) or z3.is_rational_value(a):
                    return to_float(a) * encode_expr(b)
                elif z3.is_int_value(b) or z3.is_rational_value(b):
                    return to_float(b) * encode_expr(a)
                else:
                    raise RuntimeError(f"Nonlinear term {expr}")

            elif z3.is_const(expr):
                name = expr.decl().name()
                if name in var_map:
                    return var_map[name]
                else:
                    # a numeric constant disguised as const
                    return to_float(expr)

            elif z3.is_int_value(expr) or z3.is_rational_value(expr):
                return to_float(expr)

            else:
                raise RuntimeError(f"Unsupported arithmetic expr: {expr}")

        def encode_clause(clause: z3.BoolRef):
            """Encode one linear comparison."""
            if (
                z3.is_le(clause)
                or z3.is_lt(clause)
                or z3.is_ge(clause)
                or z3.is_gt(clause)
                or z3.is_eq(clause)
            ):
                lhs, rhs = clause.children()
                lhs_expr = encode_expr(lhs)
                rhs_expr = encode_expr(rhs)
                eps = 1e-6  # for strict inequalities

                if z3.is_le(clause):
                    model.addConstr(lhs_expr <= rhs_expr, name=f"le_{lhs}_{rhs}")
                elif z3.is_lt(clause):
                    model.addConstr(lhs_expr <= rhs_expr - eps, name=f"lt_{lhs}_{rhs}")
                elif z3.is_ge(clause):
                    model.addConstr(lhs_expr >= rhs_expr, name=f"ge_{lhs}_{rhs}")
                elif z3.is_gt(clause):
                    model.addConstr(lhs_expr >= rhs_expr + eps, name=f"gt_{lhs}_{rhs}")
                elif z3.is_eq(clause):
                    model.addConstr(lhs_expr == rhs_expr, name=f"eq_{lhs}_{rhs}")
            elif z3.is_and(clause):
                for child in clause.children():
                    encode_clause(child)
            else:
                raise RuntimeError(f"Unsupported clause: {clause}")

        # Start encoding
        if z3.is_true(conjunct):
            return
        elif z3.is_false(conjunct):
            # infeasible path, skip
            model.addConstr(1 == 0)
            return
        elif z3.is_and(conjunct):
            for c in conjunct.children():
                encode_clause(c)
        else:
            encode_clause(conjunct)

        # dump model to file for debugging
        model.write("gurobi_model.lp")

    def compute_feasible_bounds(self, model, var_map):
        bounds = {}
        model.setParam("OutputFlag", 0)
        for name, var in var_map.items():
            # minimize var
            model.setObjective(var, GRB.MINIMIZE)
            model.optimize()
            lb = var.X if model.status == GRB.OPTIMAL else None

            # maximize var
            model.setObjective(var, GRB.MAXIMIZE)
            model.optimize()
            ub = var.X if model.status == GRB.OPTIMAL else None

            bounds[name] = (lb, ub)
        return bounds

    def get_maximality_cex(self, conjunct: z3.BoolRef, sampler: function):
        maximality_cex = {}
        maximality_var_bounds = {}
        maximality_model = gp.Model("maximality_cex")
        maximality_var_map = {
            var: maximality_model.addVar(lb=-GRB.INFINITY, name=var)
            for var in CONFIG_DICT_KEYS
        }
        maximality_model.update()
        self.z3_to_gurobi(conjunct, maximality_model, maximality_var_map)
        for var in CONFIG_DICT_KEYS:
            lb, ub = CONFIG_KEY_BOUNDS[var]
            maximality_model.addConstr(
                maximality_var_map[var] >= lb, name=f"bound_lb_{var}"
            )
            maximality_model.addConstr(
                maximality_var_map[var] <= ub, name=f"bound_ub_{var}"
            )
        maximality_model.setParam("OutputFlag", 0)
        maximality_model.setObjective(0, GRB.MINIMIZE)
        maximality_model.optimize()
        var_bounds = {}
        if maximality_model.status == GRB.OPTIMAL:
            for var in CONFIG_DICT_KEYS:
                var_bounds[var] = (
                    maximality_var_map[var].LB,
                    maximality_var_map[var].UB,
                )
        maximality_var_bounds = self.compute_feasible_bounds(
            maximality_model, maximality_var_map
        )
        for var in CONFIG_DICT_KEYS:
            # flip a coin for upper or lower bound sampling
            sample = None
            if np.random.rand() < 0.5:
                sample = sampler(
                    max(
                        maximality_var_bounds[var][0] - CONFIG_EPSILONS[var],
                        CONFIG_KEY_BOUNDS[var][0],
                    ),
                    maximality_var_bounds[var][0] + CONFIG_EPSILONS[var],
                )
            else:
                sample = sampler(
                    maximality_var_bounds[var][1] - CONFIG_EPSILONS[var],
                    min(
                        maximality_var_bounds[var][1] + CONFIG_EPSILONS[var],
                        CONFIG_KEY_BOUNDS[var][1],
                    ),
                )
            maximality_cex[var] = sample
        in_dtree = True
        for var in CONFIG_DICT_KEYS:
            lb, ub = maximality_var_bounds[var]
            if (maximality_cex[var] < lb) or (maximality_cex[var] > ub):
                in_dtree = False
                break
        return (maximality_cex, in_dtree)

    def get_safety_cex(self, conjunct: z3.BoolRef, sampler: function = None):
        safety_cex = {}
        safety_var_bounds = {}
        safety_model = gp.Model("safety_cex")
        safety_var_map = {
            var: safety_model.addVar(
                lb=-CONFIG_KEY_BOUNDS[var][0],
                ub=CONFIG_KEY_BOUNDS[var][1],
                name=var,
            )
            for var in CONFIG_KEY_BOUNDS.keys()
        }
        safety_model.update()
        self.z3_to_gurobi(conjunct, safety_model, safety_var_map)
        safety_model.setParam("OutputFlag", 0)
        safety_model.setObjective(0, GRB.MINIMIZE)
        safety_model.optimize()
        if safety_model.status == GRB.OPTIMAL:
            for var in CONFIG_DICT_KEYS:
                safety_var_bounds[var] = (
                    safety_var_map[var].LB,
                    safety_var_map[var].UB,
                )
        else:
            return None
        safety_var_bounds = self.compute_feasible_bounds(safety_model, safety_var_map)
        for var in CONFIG_DICT_KEYS:
            lb, ub = safety_var_bounds[var]
            if sampler is not None:
                print(f"Sampling {var} in bounds ({lb}, {ub})")
                sample = sampler(lb, ub)
            else:
                sample = (lb + ub) / 2.0
            safety_cex[var] = sample
        return safety_cex


if __name__ == "__main__":
    # Example usage
    dataset_path = sys.argv[1]
    dtree_learner = DTreeLearner(base_features=CONFIG_DICT_KEYS)
    dtree = dtree_learner.learn(dataset_path)
    dtree_tester = DTreeTester(dtree)
    print("DTree Tester initialized.")
    conjuncts = list(dtree_tester.candidate_to_conjuncts(dtree))
    print(f"Extracted {len(conjuncts)} conjuncts from the decision tree.")
    counterexamples = []
    for i, conjunct in enumerate(conjuncts):
        print(f"Conjunct {i+1}: {conjunct}")
        maximality_cex, in_dtree = dtree_tester.get_maximality_cex(
            conjunct, sampler=np.random.uniform
        )
        if maximality_cex is not None:
            print(f"Maximality counterexample: {maximality_cex}")
            counterexamples.append(maximality_cex)
        else:
            print("No maximality counterexample found.")
        safety_cex = dtree_tester.get_safety_cex(conjunct, sampler=np.random.uniform)
        if safety_cex is not None:
            print(f"Safety counterexample: {safety_cex}")
            counterexamples.append(safety_cex)
        else:
            print("No safety counterexample found.")
    configtester = ConfigTester()
    for i, cex in enumerate(counterexamples):
        print(f"Testing counterexample {i+1}: {cex}")
        success, dataset = configtester.test_config(cex)
        if success:
            print(f"Counterexample {i+1} succeeded with dataset.")
        else:
            print(f"Counterexample {i+1} failed or did not run the simulation.")
