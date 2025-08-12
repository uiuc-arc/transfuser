import z3
import numpy as np
import json
import itertools
from leaderboard.leaderboard.scenarios.perception_contract.process_dataset import SafetyChecker


class DTreeChecker:
    def __init__(self, tree_path):
        self.tree_path = tree_path
        self.tree = self.get_pre_from_json(tree_path)
        self.solver = z3.Solver()
        self.solver.set("timeout", 10000)  # Set a timeout of 10 seconds
        self.solver.set("model", True)  # Enable model generation

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
            elif z3.is_gt(curr_node) or z3.is_ge(curr_node) \
                    or z3.is_lt(curr_node) or z3.is_le(curr_node):
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
                raise RuntimeError(f"Candidate formula {curr_node} should have been converted to DNF.")



num_timesteps = 2

def flatten(lst):
    """Flatten a list of tuples into a single list."""
    return [item for sublist in lst for item in sublist]

base_features = flatten([(f"x_{i}", f"y_{i}", f"cos_{i}", f"sin_{i}") for i in range(num_timesteps)])
base_features = ["time"] + base_features
derived_feature_map = {}

def _generate_derived_features(
        base_vars, k = 2):
    res = []
    for var in base_vars:
        var_coeff_map = {var: -1}
        expr = f"(-1*{var})"
        name = expr
        res.append((name, (var_coeff_map, expr)))

    if len(base_vars) < k:
        return res

    coeff_combinations = list(itertools.product([1, -1], repeat=k))
    var_id_iter = range(len(base_vars))
    for selected_var_ids in itertools.combinations(var_id_iter, k):
        for coeff in coeff_combinations:
            var_coeff_map = {base_vars[i]: c
                                for c, i in zip(coeff, selected_var_ids)}
            expr = " + ".join(f"({c}*{base_vars[i]})"
                                for c, i in zip(coeff, selected_var_ids))
            name = f"({expr})"
            res.append((name, (var_coeff_map, expr)))
    return res

for k in range(2, len(base_features)+1):
    derived_feature_map.update(
        _generate_derived_features(base_features, k))


_var_coeff_map = {}
_var_coeff_map.update([
    (var, {var: 1}) for var in base_features
])
_var_coeff_map.update([
    (var, coeff_map) for var, (coeff_map, _) in derived_feature_map.items()
])

def get_pre_from_json(path):
    try:
        with open(path) as json_file:
            tree = json.load(json_file)
            return parse_tree(tree)
    except json.JSONDecodeError:
        raise ValueError(f"cannot parse {path} as a json file")

def parse_tree(tree) -> z3.BoolRef:
    if tree['children'] is None:
        # At a leaf node, return the clause
        if tree['classification']:
            return z3.BoolVal(True)  # True leaf node
        else:
            return z3.BoolVal(False)  # False leaf node
    elif len(tree['children']) == 2:
        # Post-order traversal
        left = parse_tree(tree['children'][0])
        right = parse_tree(tree['children'][1])
        # Create an ITE expression tree
        z3_expr = z3.Sum(*(coeff*z3.Real(base_fvar) for base_fvar, coeff
                            in _var_coeff_map[tree['attribute']].items()))
        z3_cut = z3.simplify(z3.fpToReal(z3.FPVal(tree['cut'], z3.Float64())))
        if z3.is_true(left):
            if z3.is_true(right):
                return z3.BoolVal(True)
            elif z3.is_false(right):
                return (z3_expr <= z3_cut)
        if z3.is_false(left):
            if z3.is_true(right):
                return (z3_expr > z3_cut)
            elif z3.is_false(right):
                return z3.BoolVal(False)
        # else:
        return z3.If((z3_expr <= z3_cut), left, right)
    else:
        raise ValueError("error parsing the json object as a binary decision tree)")

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
        elif z3.is_gt(curr_node) or z3.is_ge(curr_node) \
                or z3.is_lt(curr_node) or z3.is_le(curr_node):
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
            raise RuntimeError(f"Candidate formula {curr_node} should have been converted to DNF.")

def z3_model_to_dict(model):
    """
    Convert a Z3 model to a dictionary with string keys and numerical values.
    """
    res = {}
    for k in model:
        if isinstance(model[k], z3.ArithRef):
            if isinstance(model[k], z3.RatNumRef):
                res[str(k)] = model[k].numerator().as_long() / model[k].denominator().as_long()
            elif isinstance(model[k], z3.AlgebraicNumRef):
                res[str(k)] = z3.simplify(model[k]).approx()
            else:
                res[str(k)] = z3.FPVal(model[k], z3.Float64())
        elif isinstance(model[k], z3.BoolRef):
            res[str(k)] = model[k].as_long()
        else:
            res[str(k)] = model[k]
    return res

tree = get_pre_from_json("dataset/dataset.json")
print(tree)
conjuncts = list(candidate_to_conjuncts(tree))
datapoint_feats = [f"x_{i}" for i in range(num_timesteps)] + \
    [f"y_{i}" for i in range(num_timesteps)] + \
    [f"cos_{i}" for i in range(num_timesteps)] + \
    [f"sin_{i}" for i in range(num_timesteps)]
checker = SafetyChecker()
for c in conjuncts:
    print(c, "\n")
    preds = checker.is_region_safe(c)
    checker.solver.push()
    # print("Predicates:", preds)
    checker.solver.add(preds)
    if checker.solver.check() == z3.sat:
        print("Conjunct is satisfiable, hence unsafe.")
        model = checker.solver.model()
        model = z3_model_to_dict(model)
        cex = [model[k] for k in datapoint_feats]
        print("Counterexample:", cex)
        # print("Model:", model)
        checker.solver.pop()
        continue
    else:
        print("Conjunct is not satisfiable, hence safe.")
        checker.solver.pop()
        continue