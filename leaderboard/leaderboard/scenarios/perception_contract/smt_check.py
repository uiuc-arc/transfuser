import scipy.spatial as sp
import numpy as np
from leaderboard.scenarios.perception_contract.controller import control_pid, EgoModel
from z3 import *

class SafetyChecker:
    def __init__(self, pred_wps, ego_speed, npc_starting, npc_speeds, npc_forward, fps=20, time=2, debug=False):

        self.npc_starting = npc_starting  # Tuple representing the starting positions of NPC
        self.npc_speeds = npc_speeds  # Min, Max speeds of NPC
        self.npc_forward = npc_forward  # Tuple representing the forward vectors of NPC
        self.fps = fps  # Frames per second for the simulation
        self.time = time  # Time in seconds to check for safety
        self.ego_bbox = ([(-2.446798801422119, -1.0641616582870483), (-2.446798801422119, 1.0641626119613647), (2.4548845291137695, -1.0641616582870483), (2.4548845291137695, 1.0641626119613647)])

        trajectory = self.get_trajectory(pred_wps, ego_speed)
        self.ego_trajectory = [(loc[0], loc[1], loc[2]) for loc in trajectory]

        self.solver = Solver()

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


    def is_in_ego_bbox(self, x, y, t):
        """
        If the center of the ego vehicle follows the trajectory, 
        is the point (x, y) inside the ego vehicle's bounding box at time t?
        """
        ego_x, ego_y, ego_yaw = self.ego_trajectory[t]
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
        
    def is_in_other_bbox(self, x, y, s, t):
        smin = self.npc_speeds[0]
        smax = self.npc_speeds[1]
        move_vector_min = (self.npc_forward[0] * smin * (t / self.fps),
                            self.npc_forward[1] * smin * (t / self.fps))
        move_vector_max = (self.npc_forward[0] * smax * (t / self.fps),
                            self.npc_forward[1] * smax * (t / self.fps))
        
        npc_min_bbox = []
        npc_max_bbox = []

        for vertex in self.npc_starting:
            transformed_vertex_min = (vertex[0] + move_vector_min[0], vertex[1] + move_vector_min[1], smin)
            transformed_vertex_max = (vertex[0] + move_vector_max[0], vertex[1] + move_vector_max[1], smax)
            npc_min_bbox.append(transformed_vertex_min)
            npc_max_bbox.append(transformed_vertex_max)

        other_bbox_transformed = npc_min_bbox + npc_max_bbox
        return self.is_in_conv_polytope((x, y, s), other_bbox_transformed)


    def is_safe(self, t, smin=3, smax=10):
        x = Real("x")
        y = Real("y")
        s = Real("s")
        predicate = Exists(
            [x, y, s],
            And(
                self.is_in_ego_bbox(x, y, t),
                self.is_in_other_bbox(x, y, s, t),
                s >= self.npc_speeds[0],
                s <= self.npc_speeds[1],
                t >= 0,
                t <= self.time * self.fps,
            )
        )
        return predicate

    def get_predicates(self, time=2, fps=20):
        predicates = []
        for t in range(min(time * fps, len(self.ego_trajectory))):
            pred = self.is_safe(t)
            predicates.append(pred)
        return predicates
    
    def get_collision_predicate(self, time=2, fps=20):
        return Or(*self.get_predicates(time, fps))
    
    def get_trajectory(self, pred_wps, ego_speed):
        current_locs = np.array([0.0, 0.0])     # Ego vehicle starts at the origin
        ego_yaw = 0.0                           # The wps are predicted assuming the vehicle is at the origin and along x=0
        ego_model = EgoModel(dt=1./self.fps)
        steer, throttle, brake = control_pid(pred_wps, ego_speed)
        yaw = np.array([ego_yaw])
        speed = np.array([ego_speed])
        action = np.array(np.stack([steer, throttle, brake], axis=-1))

        dataset = []
        
        for i in range(self.time * self.fps):
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
            datapoint = np.array([current_locs[0], current_locs[1], ego_yaw])
            dataset.append(datapoint)

        return np.array(dataset)
    
    def get_datapoint(self):
        safety = self.get_collision_predicate(time=self.time, fps=self.fps)
        self.solver.add(safety)
        result = self.solver.check()
        if result == sat:
            if self.debug:
                print("Unsafe: There exists a collision.")
                model = self.solver.model()
                print("Model:", model)
            return (self.ego_trajectory, False) # Unsafe, there exists a collision
        else:
            return (self.ego_trajectory, True)  # Safe, no collision exists for the given parameters


# Goal:
# In the end we should be able to do this checking:
# Exists([Real("x"), Real("y"), Real("t"), Real("s")], And(is_in_ego_bbox(x, y, t), is_in_other_bbox(x, y, t, s),
# other_speed_min <= s, s <= other_speed_max)
# If this is sat, then there is a collision. Hence, unsafe. 

pred_wps = np.array([[ 1.735676,   0.01005581],
 [ 3.6433578,  -0.00992398],
 [ 5.6090746,  -0.03921339],
 [ 7.597434,  -0.07129178]])
npc_starting = [(7.773118495941162, 3.3192648887634277), (8.145111083984375, 3.299520492553711), (7.686257362365723, 1.6841745376586914), (8.058249473571777, 1.6644301414489746)]
npc_forward = [-0.05287253, -0.99528052]

solver = Solver()
checker = SafetyChecker(
    pred_wps=pred_wps,
    ego_speed = 3.7946761,
    npc_starting=npc_starting, 
    npc_speeds=(3.47, 10),
    npc_forward=npc_forward,
    fps=20,
    time=2
)

solver.add(
    checker.get_collision_predicate(time=checker.time, fps=checker.fps)
)
result = solver.check()
if result == sat:
    print("Unsafe: There exists a collision.")
else:
    print("Safe: No collision exists for the given parameters.")
