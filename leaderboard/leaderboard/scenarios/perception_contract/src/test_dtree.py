import os
import sys
import numpy as np
from synthesize_dtree import CONFIG_DICT_KEYS, CONFIG_KEY_BOUNDS
import z3
import gurobipy as gp
from gurobipy import GRB

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


def prepare_features(labels):
    feature_map = {}
    truth_map = {}
    for i, feature in enumerate(CONFIG_DICT_KEYS):
        feature_map[i] = z3.Real(feature)

    if all(labels):
        truth_map[0.0] = True
        labels = [0.0 if label else 0.0 for label in labels]
    else:
        truth_map[1.0] = True
        truth_map[0.0] = False

    return labels, feature_map, truth_map


def parse_tree_json(tree, feature_map, truth_map):
    """
    Parse the decision tree JSON structure into a Z3 expression.
    Args:
        tree (dict): The decision tree in JSON format.
    Returns:
        z3.BoolRef: The Z3 expression representing the decision tree.
    """
    if tree["is_leaf"]:
        return z3.BoolVal(truth_map[tree["value"]])

    left = parse_tree_json(tree["left"], feature_map, truth_map)
    right = parse_tree_json(tree["right"], feature_map, truth_map)

    z3_expr = None
    if tree["is_oblique"]:
        z3_expr = z3.Sum(
            *(w * feature_map[i] for w, i in zip(tree["weights"], tree["features"]))
        )
    else:
        z3_expr = feature_map[tree["feature_idx"]]

    z3_threshold = z3.simplify(z3.fpToReal(z3.FPVal(tree["threshold"], z3.Float64())))

    if z3.is_true(left):
        if z3.is_true(right):
            return z3.BoolVal(True)
        elif z3.is_false(right):
            return z3_expr <= z3_threshold
    if z3.is_false(left):
        if z3.is_true(right):
            return z3_expr > z3_threshold
        elif z3.is_false(right):
            return z3.BoolVal(False)

    return z3.If((z3_expr <= z3_threshold), left, right)


def candidate_to_conjuncts(candidate: z3.BoolRef):
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


def z3_to_gurobi(conjunct: z3.BoolRef, model: gp.Model, var_map: dict):
    """
    Convert a Z3 conjunctive formula to Gurobi constraints.

    Args:
        conjunct (z3.BoolRef): A Z3 formula in conjunctive form.
        model (gp.Model): Gurobi model to which constraints will be added.
        var_map (dict): Mapping from Z3 variables to Gurobi variables.
    """

    def extract_linear_terms(expr: z3.ArithRef):
        """
        Extract linear terms from a Z3 arithmetic expression.

        Args:
            expr (z3.ArithRef): A Z3 arithmetic expression.
        Returns:
            dict: A mapping from Z3 variables to their coefficients.
        """
        terms = {}
        if z3.is_add(expr):
            for child in expr.children():
                child_terms = extract_linear_terms(child)
                for var, coef in child_terms.items():
                    if var in terms:
                        terms[var] += coef
                    else:
                        terms[var] = coef
        elif z3.is_mul(expr):
            coeff = 1.0
            var = None
            for child in expr.children():
                if z3.is_int(child) or z3.is_real(child):
                    coeff *= float(child.as_decimal(6))
                else:
                    var = child
            if var is not None:
                terms[var] = coeff
        elif z3.is_int(expr) or z3.is_real(expr):
            terms[None] = float(expr.as_decimal(6))
        else:
            terms[expr] = 1.0
        return terms

    if z3.is_true(conjunct):
        return  # No constraints to add for true

    if z3.is_and(conjunct):
        clauses = conjunct.children()
    else:
        clauses = [conjunct]

    for clause in clauses:
        if z3.is_le(clause):
            lhs, rhs = clause.children()
            model.addConstr(
                gp.quicksum(
                    var_map[int(str(var))] * float(coef)
                    for var, coef in extract_linear_terms(lhs).items()
                )
                <= float(rhs.as_decimal(6)),
                name=f"z3_le_{str(clause)}",
            )
        elif z3.is_ge(clause):
            lhs, rhs = clause.children()
            model.addConstr(
                gp.quicksum(
                    var_map[int(str(var))] * float(coef)
                    for var, coef in extract_linear_terms(lhs).items()
                )
                >= float(rhs.as_decimal(6)),
                name=f"z3_ge_{str(clause)}",
            )
        else:
            raise RuntimeError(f"Unsupported clause type: {clause}")

def get_maximality_cex(conjunct: z3.BoolRef, sampler: function):
    maximality_cex = {}
    maximality_var_bounds = {}
    maximality_model = gp.Model("maximality_cex")
    maximality_var_map = {
        var: maximality_model.addVar(lb=-GRB.INFINITY, name=f"var_{var}")
        for var in CONFIG_DICT_KEYS
    }
    maximality_model.update()
    z3_to_gurobi(conjunct, maximality_model, maximality_var_map)
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
            var_bounds[var] = (maximality_var_map[var].LB, maximality_var_map[var].UB)
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
        sample = sampler(maximality_var_bounds[var][0], maximality_var_bounds[var][1])
        maximality_cex[var] = sample
    return maximality_cex


def get_safety_cex(conjunct: z3.BoolRef, sampler: function = None):
    safety_cex = {}
    safety_var_bounds = {}
    safety_model = gp.Model("safety_cex")
    safety_var_map = {
        var: safety_model.addVar(lb=-GRB.INFINITY, name=f"var_{var}")
        for var in CONFIG_DICT_KEYS
    }
    safety_model.update()
    z3_to_gurobi(conjunct, safety_model, safety_var_map)
    safety_model.setParam("OutputFlag", 0)
    safety_model.setObjective(0, GRB.MINIMIZE)
    safety_model.optimize()
    if safety_model.status == GRB.OPTIMAL:
        for var in CONFIG_DICT_KEYS:
            safety_var_bounds[var] = (safety_var_map[var].LB, safety_var_map[var].UB)
    else:
        return None
    for var in CONFIG_DICT_KEYS:
        lb, ub = safety_var_bounds[var]
        if sampler is not None:
            sample = sampler(lb, ub)
        else:
            sample = (lb + ub) / 2.0
        safety_cex[var] = sample
    return safety_cex


def get_cex_from_dtree(tree_json):
    labels = [0.0, 1.0]
    labels, feature_map, truth_map = prepare_features(labels)
    z3_tree = parse_tree_json(tree_json["tree"], feature_map, truth_map)
    conjuncts = list(candidate_to_conjuncts(z3_tree))
    cexs = []
    for conjunct in conjuncts:
        cex = get_safety_cex(conjunct, np.random.uniform)
        if cex is not None:
            cexs.append(cex)
        cex = get_maximality_cex(conjunct, np.random.uniform)
        if cex is not None:
            cexs.append(cex)
    return z3_tree


if __name__ == "__main__":
    # Example configuration
    config = sample_config_from_space(
        {"npc_min_starting_distance": 19, "npc_speed": 0.45}
    )
    output_file_path = "leaderboard/data/training/routes/Scenario4/Town01_Scenario4.xml"
    dump_config_to_xml(config, output_file_path)
    print(f"Configuration dumped to {output_file_path}")
