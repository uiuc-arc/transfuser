import os
import sys
import re
import numpy as np
import subprocess


def collect_dataset(min_velocity=3.0, max_velocity=15.0, increment=0.5):
    """
    Collect dataset by running scenarios with varying velocities.
    
    Args:
        min_velocity (float): Minimum velocity for the scenario.
        max_velocity (float): Maximum velocity for the scenario.
        increment (float): Incremental step for velocity.
    """

    file_path="scenario_runner/srunner/scenarios/object_crash_intersection.py"
    cmd="python leaderboard/leaderboard/leaderboard_evaluator_local.py \
    --scenarios=${SCENARIOS}  \
    --routes=${ROUTES} \
    --repetitions=${REPETITIONS} \
    --track=${CHALLENGE_TRACK_CODENAME} \
    --checkpoint=${CHECKPOINT_ENDPOINT} \
    --agent=${TEAM_AGENT} \
    --agent-config=${TEAM_CONFIG} \
    --debug=${DEBUG_CHALLENGE}"

    for velocity in np.arange(min_velocity, max_velocity + increment, increment):
        # Update the velocity in the scenario file
        content = ""
        with open(file_path, 'r') as file:
            content = file.read()
        line_to_replace = 459
        new_line = f"        self._other_actor_target_velocity = {velocity}"
        content_lines = content.split('\n')
        content_lines[line_to_replace] = new_line
        content = '\n'.join(content_lines)
        with open(file_path, 'w') as file:
            file.write(content)
        
        dataset_path = f"dataset_velocity_{velocity}.json"
        content = ""
        with open("leaderboard/leaderboard/scenarios/scenario_manager_local.py", 'r') as file:
            content = file.read()
        # Replace the dataset path in the scenario manager
        line_nums = [371, 372, 378]
        content_lines = content.split('\n')
        new_line = f"        if os.path.exists('datasets_v1/{dataset_path}'):"
        content_lines[line_nums[0]] = new_line
        new_line = f"            with open('datasets_v1/{dataset_path}', 'r') as f:"
        content_lines[line_nums[1]] = new_line
        new_line = f"        with open('datasets_v1/{dataset_path}', 'w') as f:"
        content_lines[line_nums[2]] = new_line

        content = '\n'.join(content_lines)
        with open("leaderboard/leaderboard/scenarios/scenario_manager_local.py", 'w') as file:
            file.write(content)

        # Run the scenario
        print(f"Running scenario with velocity {velocity}")
        try:
            subprocess.run(cmd, shell=True, check=True)
            print(f"Scenario completed with velocity {velocity}")
        except Exception as e:
            print(f"Exception occurred while running scenario with velocity {velocity}: {str(e)}")

if __name__ == "__main__":
    min_velocity, max_velocity, increment = 2.0, 15.0, 0.5
    if len(sys.argv) > 1:
        min_velocity = float(sys.argv[1])
    if len(sys.argv) > 2:
        max_velocity = float(sys.argv[2])
    if len(sys.argv) > 3:
        increment = float(sys.argv[3])

    collect_dataset(min_velocity, max_velocity, increment)

