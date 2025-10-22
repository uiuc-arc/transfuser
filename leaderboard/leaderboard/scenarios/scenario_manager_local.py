#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementations.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import signal
import sys
import time

import py_trees
import carla
import numpy as np
import json
import math
import os

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.autoagents.agent_wrapper_local import AgentWrapper, AgentError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.result_writer import ResultOutputProvider

from leaderboard.scenarios.perception_contract.safety import is_wp_safe
from leaderboard.scenarios.perception_contract.process_dataset import SafetyChecker

def get_rotation_matrix(roll, pitch, yaw):
    """
    Get rotation matrix from roll, pitch, yaw angles (in degrees)
    CARLA uses the convention: Z-Y-X (yaw-pitch-roll)
    """
    # Convert to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch) 
    yaw_rad = math.radians(yaw)
    
    # Rotation matrices for each axis
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll_rad), -math.sin(roll_rad)],
                    [0, math.sin(roll_rad), math.cos(roll_rad)]])
    
    R_y = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)],
                    [0, 1, 0],
                    [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])
    
    R_z = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0],
                    [math.sin(yaw_rad), math.cos(yaw_rad), 0],
                    [0, 0, 1]])
    
    # Combined rotation matrix (Z * Y * X)
    rotation_matrix = R_z @ R_y @ R_x
    return rotation_matrix


def get_roll_pitch_yaw_from_matrix(matrix):
    """
    Extract roll, pitch, yaw angles from a rotation matrix.
    From the CARLA source code: (c is cos, s is sin, y is yaw, p is pitch, r is roll)
      std::array<float, 16> transform = {
          cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr, location.x,
          cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr, location.y,
          sp, -cp * sr, cp * cr, location.z,
          0.0, 0.0, 0.0, 1.0};
    """
    pitch = math.asin(matrix[2, 0])
    cos_pitch_sign = np.sign(math.cos(pitch))
    roll = math.atan2(
        -matrix[2, 1] * cos_pitch_sign, matrix[2, 2] * cos_pitch_sign)
    yaw = math.atan2(
        matrix[1, 0] * cos_pitch_sign, matrix[0, 0] * cos_pitch_sign)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def get_yaw(vehicle_current_transform, map):
    """
    Get the yaw angle of a vehicle along a route (in radians).
    The yaw angle is the angle between the vehicle's forward 
    vector and the route's tangent vector.
    """
    vehicle_current_forward = vehicle_current_transform.get_forward_vector()
    vehicle_current_forward = np.array(
        [vehicle_current_forward.x, vehicle_current_forward.y, 0]
    )

    nearest_waypoint = map.get_waypoint(vehicle_current_transform.location)
    waypoint_forward = nearest_waypoint.transform.get_forward_vector()
    waypoint_forward = np.array([waypoint_forward.x, waypoint_forward.y, 0])

    # get angle between the vehicle's forward vector and the waypoint's forward vector
    dot_product = np.dot(
        vehicle_current_forward,
        waypoint_forward,
    )
    linalg_ = np.linalg.norm(vehicle_current_forward) * np.linalg.norm(
        waypoint_forward
    )
    if linalg_ == 0:
        __dot = 1
    else:
        __dot = np.clip(dot_product / linalg_, -1.0, 1.0)
    theta = np.arccos(__dot)
    __cross = np.cross(
        vehicle_current_forward,
        waypoint_forward,
    )
    if __cross[2] < 0:
        theta *= -1.0
    return theta


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, run and stop a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. If needed, cleanup with manager.stop_scenario()
    """


    def __init__(self, timeout, debug_mode=False):
        """
        Setups up the parameters, which will be filled at load_scenario()
        """
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = float(timeout)

        # Used to detect if the simulation is down
        watchdog_timeout = max(5, self._timeout - 2)
        self._watchdog = Watchdog(watchdog_timeout)

        # Avoid the agent from freezing the simulation
        agent_timeout = watchdog_timeout - 1
        self._agent_watchdog = Watchdog(agent_timeout)

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

        # Register the scenario tick as callback for the CARLA world
        # Use the callback_id inside the signal handler to allow external interrupts
        signal.signal(signal.SIGINT, self.signal_handler)
        self.dataset = []

    def signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._running = False

    def cleanup(self):
        """
        Reset all parameters
        """
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

    def load_scenario(self, scenario, agent, rep_number):
        """
        Load a new scenario
        """

        GameTime.restart()
        self._agent = AgentWrapper(agent)
        self.scenario_class = scenario
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors
        self.repetition_number = rep_number

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()

        self._watchdog.start()
        self._running = True

        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp)

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent and tick the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()
            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            try:
                ego_action, waypoints, frame_num = self._agent()

            # Special exception inside the agent that isn't caused by the agent
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            except Exception as e:
                raise AgentError(e)
            # 
            # Perception contract
            # 
            def unique(list):
                """
                Remove duplicates from a list while preserving order.
                """
                seen = set()
                return [x for x in list if not (x in seen or seen.add(x))]
            
            ego_transform = self.ego_vehicles[0].get_transform()
            ego_matrix_inv = ego_transform.get_inverse_matrix()

            origin_transform = carla.Transform(carla.Location(), carla.Rotation())
            ego_bbox_origin = self.ego_vehicles[0].bounding_box.get_world_vertices(origin_transform)
            ego_bbox_origin = [(vertex.x, vertex.y) for vertex in ego_bbox_origin]
            ego_bbox_origin = unique(ego_bbox_origin)

            ego_bbox = self.ego_vehicles[0].bounding_box.get_world_vertices(ego_transform)
            ego_bbox = [(vertex.x, vertex.y) for vertex in ego_bbox]
            ego_bbox = unique(ego_bbox)
            # print("Ego bounding box: ", ego_bbox_origin)
            # print("------------------------------------------------")


            if self.other_actors[0].is_alive and hasattr(self.other_actors[0], 'bounding_box'):
                non_ego_matrix = self.other_actors[0].get_transform().get_matrix()
                relative_matrix = np.dot(ego_matrix_inv, non_ego_matrix)
                roll, pitch, yaw = get_roll_pitch_yaw_from_matrix(relative_matrix)
                new_transform = carla.Transform(
                    carla.Location(
                        x=relative_matrix[0, 3],
                        y=relative_matrix[1, 3],
                        z=relative_matrix[2, 3]
                    ),
                    carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
                )

                bbox = self.other_actors[0].bounding_box
                bbox = self.other_actors[0].bounding_box.get_world_vertices(self.other_actors[0].get_transform())
                bbox = [(vertex.x, vertex.y) for vertex in bbox]
                bbox = unique(bbox)

                relative_bbox = self.other_actors[0].bounding_box.get_world_vertices(new_transform)
                relative_bbox = [(vertex.x, vertex.y) for vertex in relative_bbox]
                relative_bbox = unique(relative_bbox)

                other_forward = self.other_actors[0].get_transform().get_forward_vector()
                other_forward = [other_forward.x, other_forward.y]
                # print("Other forward vector: ", other_forward)
                # ego_rotation_matrix_inv = np.array(ego_matrix_inv)[:3, :3]
                # other_forward = np.dot(ego_rotation_matrix_inv, other_forward)

                # ego_yaw = self.ego_vehicles[0].get_transform().rotation.yaw
                # ego_yaw = get_yaw(self.ego_vehicles[0].get_transform(), CarlaDataProvider.get_map())
                ego_speed = self.ego_vehicles[0].get_velocity()
                ego_speed = np.sqrt(ego_speed.x**2 + ego_speed.y**2)
                npc_speeds = [3, 15]
                if len(waypoints) > 0:
                    # checker = SafetyChecker(
                    #     bbox, npc_speeds, other_forward, scenario_start_time=1.0, fps=20,
                    #     time=2)
                    # datapoint = checker.get_datapoint(timestamp.elapsed_seconds, waypoints, ego_speed)
                    # self.dataset = np.vstack((self.dataset, datapoint))
                    # print("Dataset shape: ", self.dataset.shape)
                    # print("Safety check result: ", datapoint[2])
                    datapoint = {
                        "timestamp": timestamp.elapsed_seconds,
                        # "ego_bbox": ego_bbox,
                        # "npc_bbox": bbox,
                        # "npc_forward": other_forward,
                        # "ego_speed": ego_speed,
                        # "waypoints": waypoints.tolist(),
                        "npc_bbox_relative": relative_bbox,
                        "ego_bbox_origin": ego_bbox_origin
                    }
                    print("i = ", frame_num, ", timestamp = ", timestamp.elapsed_seconds)
                    self.dataset.append(datapoint)
                else:
                    print("No waypoints available for safety check.")
                print("=================================================")
            self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(
                    self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

            spectator = CarlaDataProvider.get_world().get_spectator()
            ego_trans = self.ego_vehicles[0].get_transform()
            
            # For third-person view
            # location = ego_trans.transform(carla.Location(x=-4.5, z=2.3))
            # spectator.set_transform(carla.Transform(location, carla.Rotation(pitch=-15.0, yaw=ego_trans.rotation.yaw)))
            
            # For bird's eye view
            spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=50), carla.Rotation(pitch=-90)))

        if self._running and self.get_running_status():
            CarlaDataProvider.get_world().tick(self._timeout)

    def get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function triggers a proper termination of a scenario
        """
        self._watchdog.stop()

        self.end_system_time = time.time()
        self.end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = self.end_game_time - self.start_game_time

        current_dataset = None
        if os.path.exists('datasets_v1') is False:
            os.makedirs('datasets_v1')

        if os.path.exists('datasets_v1/dataset_velocity_0.5.json'):
            with open('datasets_v1/dataset_velocity_0.5.json', 'r') as f:
                current_dataset = json.load(f)

        if current_dataset is not None:
            self.dataset = current_dataset + self.dataset

        with open('datasets_v1/dataset_velocity_0.5.json', 'w') as f:
            json.dump(self.dataset, f, indent=4)

        if self.get_running_status():
            if self.scenario is not None:
                self.scenario.terminate()

            if self._agent is not None:
                self._agent.cleanup()
                self._agent = None

            self.analyze_scenario()

    def analyze_scenario(self):
        """
        Analyzes and prints the results of the route
        """
        global_result = '\033[92m'+'SUCCESS'+'\033[0m'

        for criterion in self.scenario.get_criteria():
            if criterion.test_status != "SUCCESS":
                global_result = '\033[91m'+'FAILURE'+'\033[0m'

        if self.scenario.timeout_node.timeout:
            global_result = '\033[91m'+'FAILURE'+'\033[0m'

        ResultOutputProvider(self, global_result)
