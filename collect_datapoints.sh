#!/bin/bash

min_velocity=3.0
max_velocity=15.0

# increment by 0.5
increment=0.5
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

echo "Generating velocity values from $min_velocity to $max_velocity with increment $increment"
for (( i=$(echo "$min_velocity / $increment" | bc -l); i<=$(echo "$max_velocity / $increment" | bc -l); i++ )); do
    echo "Setting velocity to $i * $increment"
    velocity=$(echo "$i * $increment" | bc -l)
    sed 's/        self._other_actor_target_velocity = 7/        self._other_actor_target_velocity = '"$velocity"'/' -i "$file_path"
    echo "Running scenario with velocity $velocity"
    eval "$cmd"
    echo "Scenario completed with velocity $velocity"
done
