import itertools
import numpy as np


num_timesteps = 2

def flatten(lst):
    """Flatten a list of tuples into a single list."""
    return [item for sublist in lst for item in sublist]

features = flatten([(f"x_{i}", f"y_{i}", f"cos_{i}", f"sin_{i}") for i in range(num_timesteps)])
features = ["time"] + features
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

for k in range(2, len(features)+1):
    derived_feature_map.update(
        _generate_derived_features(features, k))

print(f"Generated {len(derived_feature_map)} derived features.")

names_file = "dataset/dataset.names"
file_lines = ["precondition."] + \
    [f"{var}:  continuous." for var in features] + \
    [f"{var} := {expr}." for var, (_, expr) in derived_feature_map.items()] + \
    ["precondition:  true, false."]
with open(names_file, "w") as f:
    f.write('\n'.join(file_lines))

# file_lines = ["precondition."] + [f"{name}: continuous" for name in features]


