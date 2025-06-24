"""
This module contains functions relevant for the perception contract synthesis and checking.
"""

from typing import List, Dict, Any
import carla
import math
import numpy as np

class EgoModel():
    def __init__(self, dt=1. / 4):
        self.dt = dt

        # Kinematic bicycle model. Numbers are the tuned parameters from World on Rails
        self.front_wb = -0.090769015
        self.rear_wb = 1.4178275

        self.steer_gain = 0.36848336
        self.brake_accel = -4.952399
        self.throt_accel = 0.5633837

    def forward(self, locs, yaws, spds, acts):
        # Kinematic bicycle model. Numbers are the tuned parameters from World on Rails
        steer = acts[..., 0:1].item()
        throt = acts[..., 1:2].item()
        brake = acts[..., 2:3].astype(np.uint8)

        if (brake):
            accel = self.brake_accel
        else:
            accel = self.throt_accel * throt

        wheel = self.steer_gain * steer

        beta = math.atan(self.rear_wb / (self.front_wb + self.rear_wb) * math.tan(wheel))
        yaws = yaws.item()
        spds = spds.item()
        next_locs_0 = locs[0].item() + spds * math.cos(yaws + beta) * self.dt
        next_locs_1 = locs[1].item() + spds * math.sin(yaws + beta) * self.dt
        next_yaws = yaws + spds / self.rear_wb * math.sin(beta) * self.dt
        next_spds = spds + accel * self.dt
        next_spds = next_spds * (next_spds > 0.0)  # Fast ReLU

        next_locs = np.array([next_locs_0, next_locs_1])
        next_yaws = np.array(next_yaws)
        next_spds = np.array(next_spds)

        return next_locs, next_yaws, next_spds

def control_pid(waypoints, speed, is_stuck):
    ''' Predicts vehicle control with a PID controller.
    Args:
        waypoints (tensor): output of self.plan()
        velocity (tensor): speedometer input
    '''

    desired_speed = np.linalg.norm(waypoints[0] - waypoints[1]) * 2.0

    if is_stuck:
        desired_speed = np.array(4.0) # default speed of 14.4 km/h

    brake = ((desired_speed < 0.4) or ((speed / desired_speed) > self.config.brake_ratio))

    delta = np.clip(desired_speed - speed, 0.0, self.config.clip_delta)
    throttle = self.speed_controller.step(delta)
    throttle = np.clip(throttle, 0.0, self.config.clip_throttle)
    throttle = throttle if not brake else 0.0
    aim = (waypoints[1] + waypoints[0]) / 2.0
    angle = np.degrees(np.arctan2(aim[1], aim[0])) / 90.0
    if (speed < 0.01):
        angle = 0.0  # When we don't move we don't want the angle error to accumulate in the integral
    if brake:
        angle = 0.0
    
    steer = self.turn_controller.step(angle)

    steer = np.clip(steer, -1.0, 1.0) #Valid steering values are in [-1,1]

    return steer, throttle, brake


def is_point_in_bounding_box(point: List[float], bbox: carla.BoundingBox) -> bool:
    """
    Check if a point is inside a given bounding box.
    :param point: A list representing the point [x, y, z].
    :param bbox: The bounding box to check against.
    :return: True if the point is inside the bounding box, False otherwise.
    """
    location = carla.Location(x=point[0], y=point[1], z=1.0)
    return bbox.contains(location)


def get_cumulative_area(waypoints: List[List[float]],
                       ego_vehicle: carla.Vehicle):
    """
    Get the total area covered by the vehicle moving through the waypoints.
    :param waypoints: List of waypoints represented as [x, y, z] coordinates.
    :param ego_vehicle: The ego vehicle in the simulation.
    :return: The cumulative area covered by the vehicle.
    """
    bbox = ego_vehicle.bounding_box
    world_bbox = bbox.get_world_vertices(ego_vehicle.get_transform())
    total_area = 0.0
    for i in range(len(waypoints)):
        if i == 0:
            continue
        new_bbox = carla.BoundingBox(
            carla.Location(x=waypoints[i][0], y=waypoints[i][1], z=waypoints[i][2]),
            bbox.extent
        )