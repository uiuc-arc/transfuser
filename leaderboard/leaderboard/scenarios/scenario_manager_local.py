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
import math

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.autoagents.agent_wrapper_local import AgentWrapper, AgentError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.result_writer import ResultOutputProvider

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
                ego_action, waypoints = self._agent()

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
            # ego_matrix = ego_transform.get_matrix()
            # r1, p1, y1 = get_roll_pitch_yaw_from_matrix(np.array(ego_matrix))
            # print("Ego rotation: (roll, pitch, yaw): ", ego_transform.rotation.roll, 
            #       ego_transform.rotation.pitch, ego_transform.rotation.yaw)
            # print("Ego rotation (roll, pitch, yaw): ", r1, p1, y1)
            ego_matrix_inv = ego_transform.get_inverse_matrix()



            origin_transform = carla.Transform(carla.Location(), carla.Rotation())
            ego_bbox_origin = self.ego_vehicles[0].bounding_box.get_world_vertices(origin_transform)
            ego_bbox_origin = [(vertex.x, vertex.y) for vertex in ego_bbox_origin]
            ego_bbox_origin = unique(ego_bbox_origin)
            print("Ego bounding box: ", ego_bbox_origin)
            print("------------------------------------------------")
            

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

                non_ego_transform = self.other_actors[0].get_transform()
                vector_diff = non_ego_transform.location - ego_transform.location
                diff_transform = carla.Transform(vector_diff, carla.Rotation())
                # print("Other actor transform: ", non_ego_transform)
                bbox = self.other_actors[0].bounding_box.get_world_vertices(new_transform)
                bbox = [(vertex.x, vertex.y) for vertex in bbox]
                bbox = unique(bbox)
                print("Bicycle bounding box: ", bbox)
                print("------------------------------------------------")
            print("Waypoints: ", waypoints)
            print("------------------------------------------------")
            other_forward = self.other_actors[0].get_transform().get_forward_vector()
            other_forward = np.array([other_forward.x, other_forward.y, other_forward.z])
            ego_rotation_matrix_inv = np.array(ego_matrix_inv)[:3, :3]
            other_forward = np.dot(ego_rotation_matrix_inv, other_forward)
            print("Other forward vector before normalization: ", other_forward)
            other_forward = np.array(other_forward[0:2])
            print("Other forward vector: ", other_forward)
            print("-------------------------------------------------")
            ego_forward = self.ego_vehicles[0].get_transform().get_forward_vector()
            ego_forward = np.array([ego_forward.x, ego_forward.y, ego_forward.z])
            print("Ego forward vector: ", ego_forward)
            print("-------------------------------------------------")
            ego_yaw = self.ego_vehicles[0].get_transform().rotation.yaw
            ego_yaw = np.deg2rad(ego_yaw)  # Convert to radians and adjust for CARLA's coordinate system
            print("Ego yaw in radians: ", ego_yaw)
            print("-------------------------------------------------")
            ego_speed = self.ego_vehicles[0].get_velocity()
            ego_speed = np.sqrt(ego_speed.x**2 + ego_speed.y**2)
            print("Ego speed: ", ego_speed)
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
