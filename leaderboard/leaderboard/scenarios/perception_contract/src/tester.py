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
    def __init__(self, dtree: z3.BoolRef):
        self.dtree = dtree

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

        def to_float(num: z3.ArithRef) -> float:
            """Safely convert Z3 numeric constant to float."""
            if z3.is_int_value(num):
                return float(num.as_long())
            if z3.is_rational_value(num):
                return float(num.numerator_as_long()) / float(num.denominator_as_long())
            try:
                return float(str(num))
            except Exception:
                return float(num.as_decimal(6).rstrip("?"))

        def extract_linear_terms(expr: z3.ArithRef):
            """
            Extract linear terms from a Z3 arithmetic expression.
            Returns: (dict[var_name -> coeff], constant)
            """
            if z3.is_add(expr):
                total_terms = {}
                total_const = 0.0
                for child in expr.children():
                    terms, const = extract_linear_terms(child)
                    total_const += const
                    for var, coef in terms.items():
                        total_terms[var] = total_terms.get(var, 0.0) + coef
                return total_terms, total_const

            elif z3.is_mul(expr):
                coeff = 1.0
                var_name = None
                for child in expr.children():
                    if z3.is_int_value(child) or z3.is_rational_value(child):
                        coeff *= to_float(child)
                    else:
                        var_name = child.decl().name()
                if var_name is not None:
                    return {var_name: coeff}, 0.0
                else:
                    # Pure numeric product
                    return {}, coeff

            elif z3.is_int_value(expr) or z3.is_rational_value(expr):
                return {}, to_float(expr)

            elif z3.is_var(expr) or z3.is_const(expr):
                return {expr.decl().name(): 1.0}, 0.0

            else:
                raise RuntimeError(f"Unsupported arithmetic expression: {expr}")

        if z3.is_true(conjunct):
            return  # no constraints

        clauses = conjunct.children() if z3.is_and(conjunct) else [conjunct]

        for clause in clauses:
            if z3.is_le(clause) or z3.is_ge(clause):
                lhs, rhs = clause.children()
                lhs_terms, lhs_const = extract_linear_terms(lhs)
                rhs_terms, rhs_const = extract_linear_terms(rhs)

                # Move everything to LHS: lhs - rhs <=/>= 0
                all_terms = lhs_terms.copy()
                for var, coef in rhs_terms.items():
                    all_terms[var] = all_terms.get(var, 0.0) - coef
                const_term = lhs_const - rhs_const

                lhs_expr = gp.quicksum(
                    var_map[var] * coef for var, coef in all_terms.items()
                )

                if z3.is_le(clause):
                    model.addConstr(lhs_expr + const_term <= 0, name=f"z3_le_{clause}")
                else:
                    model.addConstr(lhs_expr + const_term >= 0, name=f"z3_ge_{clause}")

            else:
                raise RuntimeError(f"Unsupported clause type: {clause}")

    def get_maximality_cex(self, conjunct: z3.BoolRef, sampler: function):
        maximality_cex = {}
        maximality_var_bounds = {}
        maximality_model = gp.Model("maximality_cex")
        maximality_var_map = {
            var: maximality_model.addVar(lb=-GRB.INFINITY, name=f"var_{var}")
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
        for var in CONFIG_DICT_KEYS:
            lb, ub = var_bounds[var]
            epsilon = CONFIG_EPSILONS.get(var, 0.5)
            maximality_var_bounds[var] = (
                max(lb - epsilon, CONFIG_KEY_BOUNDS[var][0]),
                min(ub + epsilon, CONFIG_KEY_BOUNDS[var][1]),
            )
            if maximality_var_bounds[var][0] > maximality_var_bounds[var][1]:
                return None
        for var in CONFIG_DICT_KEYS:
            sample = sampler(
                maximality_var_bounds[var][0], maximality_var_bounds[var][1]
            )
            maximality_cex[var] = sample
        return maximality_cex

    def get_safety_cex(conjunct: z3.BoolRef, sampler: function = None):
        safety_cex = {}
        safety_var_bounds = {}
        safety_model = gp.Model("safety_cex")
        safety_var_map = {
            var: safety_model.addVar(
                lb=-CONFIG_KEY_BOUNDS[var][0],
                ub=CONFIG_KEY_BOUNDS[var][1],
                name=f"var_{var}",
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
    pass