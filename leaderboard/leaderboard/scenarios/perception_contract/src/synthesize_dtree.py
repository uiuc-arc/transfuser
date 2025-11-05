from obliquetree.utils import export_tree, visualize_tree
from obliquetree import Classifier
import os
import json
import sys
import pandas as pd
import numpy as np
import pickle
import subprocess
import itertools

CONFIG_DICT_KEYS = [
    "cloudiness",
    "precipitation",
    "precipitation_deposits",
    "wind_intensity",
    "sun_azimuth_angle",
    "sun_altitude_angle",
    "npc_speed",
]
CONFIG_KEY_BOUNDS = {
    "cloudiness": (0.0, 100.0),
    "precipitation": (0.0, 100.0),
    "precipitation_deposits": (0.0, 100.0),
    "wind_intensity": (0.0, 100.0),
    "sun_azimuth_angle": (0.0, 360.0),
    "sun_altitude_angle": (0.0, 90.0),
    "npc_speed": (0.0, 30.0),
}


def generate_derived_features(base_features, k=2):
    res = []
    for var in base_features:
        var_coeff_map = {var: -1}
        expr = f"(-1*{var})"
        name = expr
        res.append((name, (var_coeff_map, expr)))

        if len(base_features) < k:
            return res

        coeff_combinations = list(itertools.product([-2, -1, 1, 2], repeat=k))
        var_id_iter = range(len(base_features))
        for selected_var_ids in itertools.combinations(var_id_iter, k):
            for coeff in coeff_combinations:
                var_coeff_map = {
                    base_features[i]: c for c, i in zip(coeff, selected_var_ids)
                }
                expr = " + ".join(
                    f"({c}*{base_features[i]})" for c, i in zip(coeff, selected_var_ids)
                )
                name = f"({expr})"
                res.append((name, (var_coeff_map, expr)))
        return res


def generate_features(base_features):
    derived_feature_map = {}
    for k in range(2, len(base_features) + 1):
        print(f"Generating derived features of size {k}")
        derived_feature_map.update(generate_derived_features(base_features, k))

    var_coeff_map = {}

    var_coeff_map.update([(var, {var: 1}) for var in base_features])
    var_coeff_map.update(
        [(var, coeff_map) for var, (coeff_map, _) in derived_feature_map.items()]
    )

    return var_coeff_map


def write_features(base_features, derived_feature_map, path):
    file_lines = (
        ["precondition."]
        + [f"{var}:  continuous." for var in base_features]
        + [f"{var} := {expr}." for var, (_, expr) in derived_feature_map.items()]
        + ["precondition:  true, false."]
    )
    with open(path + ".names", "w") as f:
        f.write("\n".join(file_lines))


def create_config_dataset(
    positive_dataset,
    negative_dataset,
    output_path,
):
    # unpickle datasets
    with open(positive_dataset, "rb") as f:
        positive_datasets = pickle.load(f)
    with open(negative_dataset, "rb") as f:
        negative_datasets = pickle.load(f)

    positive_configs = []
    for config_d, dataset in positive_datasets:
        config = {}
        for c in config_d["weather"]:
            if c in CONFIG_DICT_KEYS:
                config[c] = config_d["weather"][c]
        for c in config_d.get("other_config", {}):
            if c in CONFIG_DICT_KEYS:
                config[c] = config_d["other_config"][c]
        config["label"] = "true"
        positive_configs.append(config)

    negative_configs = []
    for config_d, dataset in negative_datasets:
        config = {}
        for c in config_d["weather"]:
            if c in CONFIG_DICT_KEYS:
                config[c] = config_d["weather"][c]
        for c in config_d.get("other_config", {}):
            if c in CONFIG_DICT_KEYS:
                config[c] = config_d["other_config"][c]
        config["label"] = "false"
        negative_configs.append(config)

    # print stats about datasets
    print(f"Number of positive configs: {len(positive_configs)}")
    print(f"Number of negative configs: {len(negative_configs)}")

    dataset = positive_configs + negative_configs
    # dump to csv file
    df = pd.DataFrame(dataset)
    df.to_csv(output_path, index=False, header=False, float_format="%.6f")
    print(f"Config dataset saved to {output_path}")


def learn_decision_tree_exact_c5(dataset_path, path_to_c5="./c50exact/c5.0dbg"):
    script_path = os.path.dirname(os.path.abspath(__file__))
    path_to_c5 = os.path.abspath(os.path.join(script_path, path_to_c5))
    cmd = f"{path_to_c5} -I 1 -m 1 -f "
    cmd += dataset_path
    print(f"Running C5.0 with command: {cmd}")
    proc = subprocess.Popen(cmd, shell=True)
    output, error = proc.communicate()
    if proc.returncode != 0:
        raise RuntimeError(f"Error running C5.0: {error}")

    print(f"Decision tree learnt")


def learn_decision_tree_oblique(
    dataset_path,
    output_path,
):
    df = pd.read_csv(dataset_path)
    X = df.drop(columns=["label"])
    y = df["label"]

    clf = Classifier(
        use_oblique=True,
        max_depth=-1,
        random_state=42,
        n_pair=X.shape[1],
        categories=[],
    )
    clf.fit(X, y)

    export_tree(clf, output_path)
    print(f"Decision tree saved to {output_path}")


def synthesize_dtree(datasets_path):
    create_config_dataset(
        datasets_path + "/positive_datasets.pkl",
        datasets_path + "/negative_datasets.pkl",
        datasets_path + "/dataset.data",
    )
    write_features(
        base_features=CONFIG_DICT_KEYS,
        derived_feature_map=generate_features(CONFIG_DICT_KEYS),
        path=datasets_path + "/dataset",
    )
    learn_decision_tree_exact_c5(datasets_path + "/dataset")
    print(f"Decision tree synthesized at {datasets_path}/dataset.json")
    return datasets_path + "/dataset.json"


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python synthesize_dtree.py <datasets_path> ")
        sys.exit(1)

    datasets_path = sys.argv[1]
    dtree_path = synthesize_dtree(datasets_path)
    print(f"Decision tree synthesized at {dtree_path}")
