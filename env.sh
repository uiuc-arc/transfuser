export CARLA_ROOT=${1:-/home/adharsh/repos/transfuser/carla}
export WORK_DIR=${2:-/home/adharsh/repos/transfuser}

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
# export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export SCENARIOS=${WORK_DIR}/leaderboard/data/training/scenarios/Scenario4/Town01_Scenario4.json
export ROUTES=${WORK_DIR}/leaderboard/data/training/routes/Scenario4/Town01_Scenario4.xml
export REPETITIONS=1
export CHALLENGE_TRACK_CODENAME=SENSORS
export CHECKPOINT_ENDPOINT=${WORK_DIR}/results/latentTF_longest6.json
export TEAM_AGENT=${WORK_DIR}/team_code_transfuser/submission_agent.py
export TEAM_CONFIG=${WORK_DIR}/model_ckpt/latentTF
export DEBUG_CHALLENGE=1
export RESUME=1
export DATAGEN=0

export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:24
export SAVE_PATH=logs