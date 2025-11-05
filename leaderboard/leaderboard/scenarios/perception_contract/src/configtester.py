import os
import sys
import numpy as np
import json
import subprocess
import os
import json
from sample_configs import sample_config_from_space, dump_config_to_xml
import tempfile


class ConfigTester:
    ROOT_PATH = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../..")
    )

    def __init__(self):
        self.runner_cmd = (
            "python leaderboard/leaderboard/leaderboard_evaluator_local.py "
            "--scenarios=${SCENARIOS}  "
            "--routes=${ROUTES} "
            "--repetitions=${REPETITIONS} "
            "--track=${CHALLENGE_TRACK_CODENAME} "
            "--checkpoint=${CHECKPOINT_ENDPOINT} "
            "--agent=${TEAM_AGENT} "
            "--agent-config=${TEAM_CONFIG} "
            "--debug=${DEBUG_CHALLENGE} "
        )

    def run_simulation(self, dataset_path="datasets_v1/dataset.json"):
        """
        Run the simulation with specified parameters.
        """

        runner_cmd = self.runner_cmd + f"--dataset-path={dataset_path} "
        try:
            # run the command
            process = subprocess.Popen(
                runner_cmd, shell=True, cwd=ConfigTester.ROOT_PATH
            )
            _, error = process.communicate()
            if process.returncode != 0:
                print(f"Error running simulation: {error}")
            else:
                print("Simulation completed successfully.")
            return process.returncode
        except Exception as e:
            print(f"Exception occurred while running simulation: {str(e)}")
            return -1

    def test_config(self, config):
        """
        Test the configuration by dumping it to XML and running a simulation.
        """
        output_file_path = (
            "leaderboard/data/training/routes/Scenario4/Town01_Scenario4.xml"
        )
        dump_config_to_xml(config, output_file_path)
        print(f"Configuration dumped to {output_file_path}")
        with tempfile.NamedTemporaryFile(
            delete=False, suffix=".json"
        ) as temp_dataset_file:
            dataset_path = temp_dataset_file.name
            status = self.run_simulation(dataset_path=dataset_path)
            if status != 0:
                print(f"Simulation failed to run.")
                return (False, None)
            with open(dataset_path, "r") as f:
                dataset = json.load(f)
            if dataset["label"]:
                print(f"Simulation succeeded.")
                return (True, dataset)
            else:
                print(f"Simulation failed.")
                return (False, dataset)


if __name__ == "__main__":
    tester = ConfigTester()
    sample_config = sample_config_from_space(
        {"npc_min_starting_distance": 19, "npc_speed": 0.45}
    )
    print(f"Testing sample configuration: {sample_config}")
    success, dataset = tester.test_config(sample_config)
    if success:
        print("Configuration succeeded with dataset.")
    else:
        print("Configuration failed or simulation did not run.")
