import os
import sys
import numpy as np
import json
import subprocess


def run_simulation():
    """
    Run the simulation with specified parameters.
    """

    cmd = "python leaderboard/leaderboard/leaderboard_evaluator_local.py \
    --scenarios=${SCENARIOS}  \
    --routes=${ROUTES} \
    --repetitions=${REPETITIONS} \
    --track=${CHALLENGE_TRACK_CODENAME} \
    --checkpoint=${CHECKPOINT_ENDPOINT} \
    --agent=${TEAM_AGENT} \
    --agent-config=${TEAM_CONFIG} \
    --debug=${DEBUG_CHALLENGE}"

    try:
        subprocess.run(cmd, shell=True, check=True)
        print("Simulation completed successfully.")
    except Exception as e:
        print(f"Exception occurred while running simulation: {str(e)}")
