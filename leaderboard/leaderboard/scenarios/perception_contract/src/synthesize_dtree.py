from obliquetree.utils import export_tree, visualize_tree
from obliquetree import Classifier
import os
import json
import sys
import pandas as pd

CONFIG_DICT_KEYS = [
    "cloudiness",
    "precipitation",
    "precipitation_deposits",
    "wind_intensity",
    "sun_azimuth_angle",
    "sun_altitude_angle",
    "npc_speed",
]


def create_config_dataset(
    positive_datasets,
    negative_datasets,
    output_path,
):

    positive_configs = []
    for dataset_path in positive_datasets:
        with open(dataset_path, "r") as f:
            dataset = json.load(f)
            config = {}
            for c in dataset["config"]["weather"]:
                if c in CONFIG_DICT_KEYS:
                    config[c] = dataset["config"]["weather"][c]
            for c in dataset["config"].get("other_config", {}):
                if c in CONFIG_DICT_KEYS:
                    config[c] = dataset["config"]["other_config"][c]
            config["label"] = 1
            positive_configs.append(config)

    negative_configs = []
    for dataset_path in negative_datasets:
        with open(dataset_path, "r") as f:
            dataset = json.load(f)
            config = {}
            for c in dataset["config"]["weather"]:
                if c in CONFIG_DICT_KEYS:
                    config[c] = dataset["config"]["weather"][c]
            for c in dataset["config"].get("other_config", {}):
                if c in CONFIG_DICT_KEYS:
                    config[c] = dataset["config"]["other_config"][c]
            config["label"] = 0
            negative_configs.append(config)

    dataset = positive_configs + negative_configs
    # dump to csv file
    df = pd.DataFrame(dataset)
    df.to_csv(output_path, index=False)
    print(f"Config dataset saved to {output_path}")


def learn_decision_tree(
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
