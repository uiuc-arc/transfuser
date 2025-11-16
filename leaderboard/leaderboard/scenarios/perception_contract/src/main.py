import os
import json
from sample_configs import sample_config_from_space
import pickle
from learner import (
    WEATHER_CONFIG_DICT_KEYS,
    DTreeLearner,
    CONFIG_KEY_BOUNDS,
    NPC_CONFIG_DICT_KEYS,
)
from tester import DTreeTester
from configtester import ConfigTester
import numpy as np
import sys
import datetime


class SOCFinder:
    def __init__(self, config_space, dataset_dir="dataset"):
        self.config_space = config_space
        self.configtester = ConfigTester()
        self.learner = DTreeLearner(base_features=list(self.config_space.keys()))
        self.tester = DTreeTester()
        self.dataset_dir = dataset_dir
        if not os.path.exists(self.dataset_dir):
            os.makedirs(self.dataset_dir)

    def run_seed_configs(self, num_seed_configs=10):
        positive_configs = []
        negative_configs = []
        for i in range(num_seed_configs):
            # Sample a new configuration
            config = sample_config_from_space(None, self.config_space)
            print(f"Testing configuration {i+1}/{num_seed_configs}: {config}")
            # Test the configuration
            success, dataset = self.configtester.test_config(config)
            if success:
                print(f"Configuration {i+1} succeeded with dataset.")
                positive_configs.append(config)
            elif not success and dataset is not None:
                print(f"Configuration {i+1} failed with dataset.")
                negative_configs.append(config)
            else:
                print(f"Configuration {i+1} failed to run the simulation.")
        # Save the dataset to file
        path = os.path.join(self.dataset_dir, "dataset.data")
        with open(path, "w") as f:
            for config in positive_configs:
                line = ",".join(
                    [str(config["weather"][key]) for key in WEATHER_CONFIG_DICT_KEYS]
                    + [str(config["other_config"][key]) for key in NPC_CONFIG_DICT_KEYS]
                    + ["true"]
                )
                f.write(line + "\n")
            for config in negative_configs:
                line = ",".join(
                    [str(config["weather"][key]) for key in WEATHER_CONFIG_DICT_KEYS]
                    + [str(config["other_config"][key]) for key in NPC_CONFIG_DICT_KEYS]
                    + ["false"]
                )
                f.write(line + "\n")
        print(f"Dataset saved to {path}")

    def learner_tester_loop(self, num_iterations=100):
        iters = 0
        stopping_criteria_met = False
        dataset_path = os.path.join(self.dataset_dir, "dataset")
        while iters < num_iterations and not stopping_criteria_met:
            iters += 1
            print(f"Starting iteration {iters+1}/{num_iterations}")
            # Learn decision tree
            dtree = self.learner.learn(dataset_path)
            print("Learned decision tree:")
            print(dtree)
            # Test decision tree
            conjuncts = self.tester.candidate_to_conjuncts(dtree)
            cexs = []
            more_dps = []
            for conjunct in conjuncts:
                maximality_cex, in_dtree = self.tester.get_maximality_cex(
                    conjunct, sampler=np.random.uniform
                )
                success, dataset = self.configtester.test_config(maximality_cex)
                if not in_dtree and success:
                    print(f"Found positive counterexample: {maximality_cex}")
                    # recheck if in dtree already
                    cexs.append((maximality_cex, success))
                elif in_dtree and not success and dataset is not None:
                    print(f"Found negative counterexample: {maximality_cex}")
                    cexs.append((maximality_cex, success))
                elif in_dtree and success:
                    print(
                        f"Maximality counterexample is positive and in DT: {maximality_cex}"
                    )
                    more_dps.append((maximality_cex, success))
                elif not in_dtree and not success and dataset is not None:
                    print(
                        f"Maximality counterexample is negative and not in DT: {maximality_cex}"
                    )
                    more_dps.append((maximality_cex, success))
                else:
                    print(f"Counterexample {maximality_cex} did not yield a dataset.")
            if len(cexs) == 0:
                print("No counterexamples found, stopping.")
                stopping_criteria_met = True
            else:
                print(f"Adding {len(cexs)} counterexamples to dataset.")
                with open(dataset_path, "a") as f:
                    existing_points = set()
                    with open(dataset_path, "r") as rf:
                        for line in rf:
                            existing_points.add(line.strip())
                    for cex, success in cexs:
                        label = "true" if success else "false"
                        string = (
                            ",".join(
                                [
                                    str(cex["weather"][key])
                                    for key in WEATHER_CONFIG_DICT_KEYS
                                ]
                                + [
                                    str(cex["other_config"][key])
                                    for key in NPC_CONFIG_DICT_KEYS
                                ]
                            )
                            + f",{label}\n"
                        )
                        if string.strip() not in existing_points:
                            print(f"Adding counterexample: {string.strip()}")
                            f.write(string)
                        else:
                            print(f"Counterexample already exists: {string.strip()}")
        print("Learner-tester loop finished.")
        print(f"Final DTree saved to {dataset_path}.json")


if __name__ == "__main__":
    num_iterations = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    num_seed_configs = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    default_data_dir = "expts/expt_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_dir = sys.argv[3] if len(sys.argv) > 3 else default_data_dir
    runner = SOCFinder(config_space=CONFIG_KEY_BOUNDS, dataset_dir=dataset_dir)
    try:
        runner.run_seed_configs(num_seed_configs=num_seed_configs)
        runner.learner_tester_loop(num_iterations=num_iterations)
    except KeyboardInterrupt:
        print("Interrupted by user, exiting.")
