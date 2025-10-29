from obliquetree.utils import export_tree, visualize_tree
from obliquetree import Classifier
import os
import json
import sys
import pandas as pd
import numpy as np
import pickle
import subprocess

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


def write_features(dataset_path):
    features = ""
    for var in CONFIG_DICT_KEYS:
        features += f"{var}:  continuous.\n"
    features_path = dataset_path.split(".")[0] + ".names"
    with open(features_path, "w") as f:
        f.write("precondition.\n")
        f.write(features)
        f.write("precondition: true, false.\n")
    print(f"Features written to {features_path}")


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

    dataset = positive_configs + negative_configs
    # dump to csv file
    df = pd.DataFrame(dataset)
    df.to_csv(output_path, index=False, header=False, float_format="%.6f")
    print(f"Config dataset saved to {output_path}")


def learn_decision_tree_exact_c5(
    dataset_path, output_path, path_to_c5="./c50exact/c5.0dbg"
):
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


if __name__ == "__main__":
    positive_dataset = "datasets/positive_datasets.pkl"
    negative_dataset = "datasets/negative_datasets.pkl"
    config_dataset_path = "datasets/config_dataset.data"
    dtree_output_path = "datasets/config_dtree.json"

    create_config_dataset(
        positive_dataset,
        negative_dataset,
        config_dataset_path,
    )

    write_features(config_dataset_path)

    learn_decision_tree_exact_c5(
        config_dataset_path.split(".")[0],
        dtree_output_path,
    )
