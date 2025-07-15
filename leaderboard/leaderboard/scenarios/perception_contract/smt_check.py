from typing import List, Tuple

import scipy.spatial as sp
import numpy as np
from leaderboard.scenarios.perception_contract.controller import control_pid, EgoModel
from z3 import *

class SafetyChecker:

    def __init__(self, npc_starting=None, npc_speeds=None, npc_forward=None, scenario_start_time=1.0, fps=20, time=2, debug=False):

        self.scenario_start_time = scenario_start_time  # Absolute time when the scenario starts
        self.scenario_end_time = (scenario_start_time + time) # Absolute time when the scenario ends
        
        npc_starting = [(7.773118495941162, 3.3192648887634277), (8.145111083984375, 3.299520492553711), (8.058249473571777, 1.6644301414489746), (7.686257362365723, 1.6841745376586914)]
        npc_forward = ([-0.05287253, -0.99528052])
        npc_speeds = [3, 15]
        
        self.npc_starting = npc_starting  # Tuple representing the starting positions of NPC
        self.npc_speeds = npc_speeds  # Min, Max speeds of NPC
        self.npc_forward = npc_forward  # Tuple representing the forward vectors of NPC
        self.fps = fps  # Frames per second for the simulation
        self.sim_time = time  # Time in seconds to check for safety
        self.ego_bbox = [(-2.446798801422119, -1.0641616582870483), (-2.446798801422119, 1.0641626119613647), (2.4548845291137695, 1.0641626119613647), (2.4548845291137695, -1.0641616582870483)]

        self.solver = Solver()
        self.solver.set("timeout", 10000)
        self.debug = debug

    def is_in_conv_polytope(self, point, polytope):
        """
        Check if a point is inside a convex polytope defined by its vertices.
        Args:
            point (tuple): A tuple representing the point (could be 2D, 3D, etc.).
            polytope (list): A list of points that represent the vertices of the convex polytope.
        Returns:
            z3.Expr: A Z3 expression that is True if the point is inside the polytope.
        """
        hull = sp.ConvexHull(polytope)
        equations = hull.equations

        if len(equations[0]) != (len(point) + 1):
            print(f"Point: {point}, Polytope: {polytope}")
            raise ValueError("Point dimension does not match polytope dimension.")

        # Create a list of inequalities for the point to be inside the polytope
        inequalities = []
        for eq in equations:
            coefficients = eq[:-1]  # All but the last coefficient
            constant = eq[-1]  # The last coefficient is the constant term
            inequality = sum(coeff * point[i] for i, coeff in enumerate(coefficients)) <= -constant
            inequalities.append(inequality)

        # Combine all inequalities into a single expression
        return And(*inequalities)

    def is_in_ego_bbox(self, x, y, ego_point):
        """
        If the center of the ego vehicle follows the trajectory, 
        is the point (x, y) inside the ego vehicle's bounding box at time t?
        """
        ego_x, ego_y, ego_yaw = ego_point
        # Transform the ego bounding box by adding the center position
        ego_bbox_transformed = []
        for vertex in self.ego_bbox:
            transformed_vertex = (vertex[0] + ego_x, vertex[1] + ego_y)
            ego_bbox_transformed.append(transformed_vertex)
        # Rotate the bounding box vertices by the yaw angle
        ego_bbox_transformed = [
            (
                vertex[0] * np.cos(ego_yaw) - vertex[1] * np.sin(ego_yaw),
                vertex[0] * np.sin(ego_yaw) + vertex[1] * np.cos(ego_yaw)
            )
            for vertex in ego_bbox_transformed
        ]
        return self.is_in_conv_polytope((x, y), ego_bbox_transformed)

    def is_in_other_bbox_volume(self, x:z3.ArithRef, y:z3.ArithRef, s:z3.ArithRef, t:z3.ArithRef) -> z3.BoolRef:
        """
        Check if the point (x, y) is inside the bounding box of the NPC at time t, assuming translation based on speed s.
        Args: (All symbolic)
            x (z3.ArithRef): x-coordinate of the point.
            y (z3.ArithRef): y-coordinate of the point.
            s (z3.ArithRef): Speed of the NPC.
            t (z3.ArithRef): Absolute time at which to check the bounding box.
        Returns:
            z3.BoolRef: A Z3 expression that is True if the point is inside the bounding box of the space 
                        occupied by the NPC
        """

        predicates = []

        npc_x_0 = z3.Real("npc_x_0")
        npc_y_0 = z3.Real("npc_y_0")
        npc_x_1 = z3.Real("npc_x_1")
        npc_y_1 = z3.Real("npc_y_1")
        npc_x_2 = z3.Real("npc_x_2")
        npc_y_2 = z3.Real("npc_y_2")
        npc_x_3 = z3.Real("npc_x_3")
        npc_y_3 = z3.Real("npc_y_3")

        predicates.append(
            z3.Implies(t < self.scenario_start_time,
                       z3.And(s == 0,
                               npc_x_0 == self.npc_starting[0][0],
                               npc_y_0 == self.npc_starting[0][1],
                               npc_x_1 == self.npc_starting[1][0],
                               npc_y_1 == self.npc_starting[1][1],
                               npc_x_2 == self.npc_starting[2][0],
                               npc_y_2 == self.npc_starting[2][1],
                               npc_x_3 == self.npc_starting[3][0],
                               npc_y_3 == self.npc_starting[3][1])))
        predicates.append(
            z3.Implies(z3.And(t >= self.scenario_start_time, 
                                t <= self.scenario_end_time),
                        z3.And(s >= self.npc_speeds[0],
                               s <= self.npc_speeds[1])))
        predicates.append(
            z3.Implies(t > self.scenario_end_time,
                       s == 0)
        )

        predicates.append(
            z3.Implies(t >= self.scenario_start_time,
                       z3.And(
                            npc_x_0 == (self.npc_starting[0][0] + self.npc_forward[0] * s * ((t - self.scenario_start_time))),
                            npc_y_0 == (self.npc_starting[0][1] + self.npc_forward[1] * s * ((t - self.scenario_start_time))),
                            npc_x_1 == (self.npc_starting[1][0] + self.npc_forward[0] * s * ((t - self.scenario_start_time))),
                            npc_y_1 == (self.npc_starting[1][1] + self.npc_forward[1] * s * ((t - self.scenario_start_time))),
                            npc_x_2 == (self.npc_starting[2][0] + self.npc_forward[0] * s * ((t - self.scenario_start_time))),
                            npc_y_2 == (self.npc_starting[2][1] + self.npc_forward[1] * s * ((t - self.scenario_start_time))),
                            npc_x_3 == (self.npc_starting[3][0] + self.npc_forward[0] * s * ((t - self.scenario_start_time))),
                            npc_y_3 == (self.npc_starting[3][1] + self.npc_forward[1] * s * ((t - self.scenario_start_time)))
                       ))
        )

        """
        M of coordinates (x,y) is inside the rectangle iff
        (0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD) where . is the dot product,
        where A is the first vertex of the rectangle, B is the second vertex, and D is the fourth vertex.
        """
        A = (npc_x_0, npc_y_0)
        B = (npc_x_1, npc_y_1)
        D = (npc_x_3, npc_y_3)
        M = (x, y)
        AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
        AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
        AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
        AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)
        predicates.append(z3.And(AM_AB >= 0, AM_AB <= AB_AB, AM_AD >= 0, AM_AD <= AD_AD))
        # Check if the point (x, y) is inside the bounding box of the NPC
        return And(*predicates)
    
    def is_in_ego_volume(self, x:z3.ArithRef, y:z3.ArithRef, ego_point:List[z3.ArithRef]) -> z3.BoolRef:
        if len(ego_point) != 4:
            raise ValueError("Ego point must be a list of 4 elements: [x, y, cos(yaw), sin(yaw)]")
        ego_x, ego_y, cos_yaw, sin_yaw = ego_point
        predicates = []

        # get the vertices of the ego bounding box
        ego_bbox_transformed = []
        for vertex in self.ego_bbox:
            transformed_vertex = (vertex[0] + ego_x, vertex[1] + ego_y)
            ego_bbox_transformed.append(transformed_vertex)

        # Rotate the bounding box vertices by the yaw angle
        rotated_bbox = []
        for vertex in ego_bbox_transformed:
            rotated_x = vertex[0] * cos_yaw - vertex[1] * sin_yaw
            rotated_y = vertex[0] * sin_yaw + vertex[1] * cos_yaw
            rotated_bbox.append((rotated_x, rotated_y))

        # Check if the point (x, y) is inside the rotated bounding box
        A = rotated_bbox[0]
        B = rotated_bbox[1]
        D = rotated_bbox[3]  # Fourth vertex
        M = (x, y)
        AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
        AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
        AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
        AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)
        predicates.append(z3.And(AM_AB >= 0,
                                  AM_AB <= AB_AB,
                                  AM_AD >= 0,
                                  AM_AD <= AD_AD))
        return And(*predicates)


    def is_region_safe(self, conjunct:z3.BoolRef, pred_len:int = 2) -> z3.BoolRef:
        t = Real("t")
        s = Real("s")

        x = Real("x")
        y = Real("y")

        x_pred = [Real(f"x_{i}") for i in range(pred_len)]
        y_pred = [Real(f"y_{i}") for i in range(pred_len)]
        cos_yaw_pred = [Real(f"cos_{i}") for i in range(pred_len)]
        sin_yaw_pred = [Real(f"sin_{i}") for i in range(pred_len)]

        predicates = []

        ego_preds = []

        for i in range(pred_len):
            predicates.append(
                cos_yaw_pred[i] ** 2 + sin_yaw_pred[i] ** 2 == 1)
            ego_preds.append(
                And(
                    self.is_in_ego_volume(x, y, [x_pred[i], y_pred[i], cos_yaw_pred[i], sin_yaw_pred[i]]),
                    self.is_in_other_bbox_volume(x, y, s, t +  (i / self.fps)),
                )
            )

        predicates.append(Or(*ego_preds))
        predicates.append(t >= 0)
        predicates.append(conjunct)

        return And(*predicates)


    def is_in_other_bbox(self, x, y, s, prediction_at_time, timestep):
        smin = self.npc_speeds[0]
        smax = self.npc_speeds[1]
        sim_timestep = (prediction_at_time * self.fps) + timestep
        x_forward = 0.0 if sim_timestep < self.scenario_start_time else self.npc_forward[0]
        y_forward = 0.0 if sim_timestep < self.scenario_start_time else self.npc_forward[1]

        move_vector_min = (x_forward * smin * (timestep / self.fps),
                            y_forward * smin * (timestep / self.fps))
        move_vector_max = (x_forward * smax * (timestep / self.fps),
                            y_forward * smax * (timestep / self.fps))
        
        npc_min_bbox = []
        npc_max_bbox = []

        for vertex in self.npc_starting:
            transformed_vertex_min = (vertex[0] + move_vector_min[0], vertex[1] + move_vector_min[1], smin)
            transformed_vertex_max = (vertex[0] + move_vector_max[0], vertex[1] + move_vector_max[1], smax)
            npc_min_bbox.append(transformed_vertex_min)
            npc_max_bbox.append(transformed_vertex_max)

        other_bbox_transformed = npc_min_bbox + npc_max_bbox
        return self.is_in_conv_polytope((x, y, s), other_bbox_transformed)

    def is_safe(self, predicted_point, prediction_at_time, t):
        x = Real("x")
        y = Real("y")
        s = Real("s")
        predicate = Exists(
            [x, y, s],
            And(
                self.is_in_ego_bbox(x, y, predicted_point),
                self.is_in_other_bbox(x, y, s, prediction_at_time, t),
                s >= self.npc_speeds[0],
                s <= self.npc_speeds[1]
            )
        )
        return predicate

    def get_predicates(self, pred_wps, ego_speed, prediction_at_time):
        predicates = []
        predicted_points = self.get_trajectory(pred_wps, ego_speed)
        for t in range(self.sim_time * self.fps):
            pred = self.is_safe(predicted_points[t], prediction_at_time, t)
            predicates.append(pred)
        return predicates, predicted_points
        
    def get_collision_predicate(self, start_time, pred_wps, ego_speed):
        predicates, ego_trajectory = self.get_predicates(pred_wps, ego_speed, start_time)
        return Or(*predicates), ego_trajectory

    def get_trajectory(self, pred_wps, ego_speed):
        current_locs = np.array([0.0, 0.0])     # Ego vehicle starts at the origin
        ego_yaw = 0.0                           # The wps are predicted assuming the vehicle is at the origin and along x=0
        ego_model = EgoModel(dt=1./self.fps)
        steer, throttle, brake = control_pid(pred_wps, ego_speed)
        yaw = np.array([ego_yaw])
        speed = np.array([ego_speed])
        action = np.array(np.stack([steer, throttle, brake], axis=-1))

        dataset = []
        
        for i in range(self.sim_time * self.fps):
            pred_wps_copy = pred_wps.copy()
            # Adjust the predicted waypoints to be relative to the ego vehicle
            pred_wps_copy[:, 0] -= current_locs[0]
            pred_wps_copy[:, 1] -= current_locs[1]
            cos_yaw = np.cos(-ego_yaw)
            sin_yaw = np.sin(-ego_yaw)
            rotation_matrix = np.array([
                [cos_yaw, -sin_yaw],
                [sin_yaw, cos_yaw]
            ])
            pred_wps_copy = np.dot(pred_wps_copy, rotation_matrix.T)  # Rotate waypoints to match ego vehicle's orientation
            forward_wps = pred_wps_copy[pred_wps_copy[:, 0] > 0]  # Only consider waypoints in front of the ego vehicle
            
            # Need at least two waypoints to calculate control
            if len(forward_wps) < 2:
                break

            steer, throttle, brake = control_pid(forward_wps, ego_speed)
            yaw = np.array([ego_yaw])
            speed = np.array([ego_speed])
            action = np.array(np.stack([steer, throttle, brake], axis=-1))
            new_locs, new_yaw, new_speed = ego_model.forward(
                current_locs, yaw, speed, action)
            
            current_locs = new_locs
            ego_yaw = new_yaw
            ego_speed = new_speed
            datapoint = ([current_locs[0], current_locs[1], ego_yaw])
            dataset.append(datapoint)


        if len(dataset) < self.sim_time * self.fps:
            last_point = dataset[-1]
            pad_size = self.sim_time * self.fps - len(dataset)
            # Pad the dataset with the last point to ensure it has the correct length
            dataset.extend([last_point] * pad_size)

        return dataset
    
    def get_datapoint(self, start_time, pred_wps, ego_speed):
        safety, ego_trajectory = self.get_collision_predicate(start_time, pred_wps, ego_speed)
        self.solver.add(safety)
        result = self.solver.check()
        if result == sat:
            if self.debug:
                print("Unsafe: There exists a collision.")
                model = self.solver.model()
                print("Model:", model)
            return (start_time, ego_trajectory, False) # Unsafe, there exists a collision
        else:
            return (start_time, ego_trajectory, True)  # Safe, no collision exists for the given parameters

# Goal:
# In the end we should be able to do this checking:
# Exists([Real("x"), Real("y"), Real("t"), Real("s")], And(is_in_ego_bbox(x, y, t), is_in_other_bbox(x, y, t, s),
# other_speed_min <= s, s <= other_speed_max)
# If this is sat, then there is a collision. Hence, unsafe. 

# pred_wps = np.array([[ 1.735676,   0.01005581],
#  [ 3.6433578,  -0.00992398],
#  [ 5.6090746,  -0.03921339],
#  [ 7.597434,  -0.07129178]])
# npc_starting = [(7.773118495941162, 3.3192648887634277), (8.145111083984375, 3.299520492553711), (7.686257362365723, 1.6841745376586914), (8.058249473571777, 1.6644301414489746)]
# npc_forward = ([-0.05287253, -0.99528052])

# solver = Solver()
# checker = SafetyChecker(
# )
# x = Real("x")
# y = Real("y")
# s = Real("s")
# t = Real("t")
# safety_preds = checker.is_in_other_bbox_volume(x, y, s, t)
# print("Safety predicates:", safety_preds)
# solver.add(safety_preds)
# result = solver.check()
# if result == sat:
#     print("Unsafe: There exists a collision.")
#     model = solver.model()
#     print("Model:", model)
# else:
#     print("Safe: No collision exists for the given parameters.")
# result = checker.get_datapoint()
# print(result)
# solver.add(
#     checker.get_collision_predicate()
# )
# result = solver.check()
# if result == sat:
#     print("Unsafe: There exists a collision.")
# else:
#     print("Safe: No collision exists for the given parameters.")
