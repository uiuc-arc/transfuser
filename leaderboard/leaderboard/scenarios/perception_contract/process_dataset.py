from typing import List, Tuple, Optional
import sys
import scipy.spatial as sp
import numpy as np
from leaderboard.scenarios.perception_contract.controller import control_pid, EgoModel
import z3
import json
import pandas as pd
import tqdm


class SafetyChecker:

    EGO_VEHICLE_X_AXIS_LENGTH = 4.90168333  # Length of the ego vehicle in meters

    def __init__(
        self,
    ):
        pass

    @staticmethod
    def filter_points(vertices):
        """
        If len(vertices) is > 4, filter them to keep the four unique points
        """
        if len(vertices) > 4:
            unique_vertices = []
            for vertex in vertices:
                for unique_vertex in unique_vertices:
                    if np.allclose(vertex, unique_vertex, atol=1e-2):
                        break
                else:
                    unique_vertices.append(vertex)
            return unique_vertices[:4]
        return vertices

    @staticmethod
    def find_vertices_along_distance(vertices, distance):
        """
        Find vertex pairs that are at least 'distance' apart.
        """
        for i in range(len(vertices)):
            for j in range(i + 1, len(vertices)):
                dist = np.linalg.norm(np.array(vertices[i]) - np.array(vertices[j]))
                if np.isclose(dist, distance, atol=1e-2) or dist >= distance:
                    return (vertices[i], vertices[j])

    @staticmethod
    def to_xyyaw(vertices, x_dim):
        """
        Convert vertices to x, y, yaw format.
        """
        vertices = np.array(SafetyChecker.filter_points(vertices))
        point_a, point_b = SafetyChecker.find_vertices_along_distance(vertices, x_dim)
        tan_yaw = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])
        yaw = np.arctan(tan_yaw)
        if point_b[0] < point_a[0]:
            yaw += np.pi
        x = np.mean(vertices[:, 0]).item()
        y = np.mean(vertices[:, 1]).item()
        return x, y, yaw

    @staticmethod
    def is_in_conv_polytope(point: Tuple, polytope: List[Tuple]) -> z3.ExprRef:
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
            inequality = (
                sum(coeff * point[i] for i, coeff in enumerate(coefficients))
                <= -constant
            )
            inequalities.append(inequality)

        # Combine all inequalities into a single expression
        return z3.And(*inequalities)

    @staticmethod
    def is_in_ego_bbox(
        ego_bbox: List[Tuple],
        x: z3.ArithRef,
        y: z3.ArithRef,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
    ) -> z3.ExprRef:
        """
        If the center of the ego vehicle follows the trajectory,
        is the point (x, y) inside the ego vehicle's bounding box at time t?
        """
        # ego_x, ego_y, ego_yaw = ego_point
        # Transform the ego bounding box by adding the center position
        ego_bbox_transformed = []
        for vertex in ego_bbox:
            transformed_vertex = (vertex[0] + ego_x, vertex[1] + ego_y)
            ego_bbox_transformed.append(transformed_vertex)
        # Rotate the bounding box vertices by the yaw angle
        ego_bbox_transformed = [
            (
                vertex[0] * np.cos(ego_yaw) - vertex[1] * np.sin(ego_yaw),
                vertex[0] * np.sin(ego_yaw) + vertex[1] * np.cos(ego_yaw),
            )
            for vertex in ego_bbox_transformed
        ]
        return SafetyChecker.is_in_conv_polytope((x, y), ego_bbox_transformed)

    @staticmethod
    def is_in_other_bbox(
        x: z3.ArithRef,
        y: z3.ArithRef,
        s: z3.ArithRef,
        s_min: float,
        s_max: float,
        prediction_at_time: float,
        timestep: int,  # num frames from prediction_at_time
        scenario_start_time: float,
        fps: int,
        npc_starting: List[Tuple[float, float]],
        npc_forward: Tuple[float, float],
    ) -> z3.ExprRef:

        x_forward = npc_forward[0]
        y_forward = npc_forward[1]

        effective_time = np.maximum(
            0, (prediction_at_time + (timestep * (1.0 / fps))) - scenario_start_time
        )

        move_vector_min = (
            x_forward * s_min * effective_time,
            y_forward * s_min * effective_time,
        )
        move_vector_max = (
            x_forward * s_max * effective_time,
            y_forward * s_max * effective_time,
        )

        npc_min_bbox = []
        npc_max_bbox = []

        for vertex in npc_starting:
            transformed_vertex_min = (
                vertex[0] + move_vector_min[0],
                vertex[1] + move_vector_min[1],
                s_min,
            )
            transformed_vertex_max = (
                vertex[0] + move_vector_max[0],
                vertex[1] + move_vector_max[1],
                s_max,
            )
            npc_min_bbox.append(transformed_vertex_min)
            npc_max_bbox.append(transformed_vertex_max)

        other_bbox_transformed = npc_min_bbox + npc_max_bbox
        return SafetyChecker.is_in_conv_polytope((x, y, s), other_bbox_transformed)

    @staticmethod
    def is_safe(
        ego_bbox: List[Tuple],
        predicted_point: List[float],
        prediction_at_time: float,
        t: int,
        s_min: float,
        s_max: float,
        scenario_start_time: float,
        fps: int,
        npc_starting: List[Tuple[float, float]],
        npc_forward: Tuple[float, float],
    ) -> z3.ExprRef:
        x = z3.Real("x")
        y = z3.Real("y")
        s = z3.Real("s")
        predicate = z3.Exists(
            [x, y, s],
            z3.And(
                SafetyChecker.is_in_ego_bbox(ego_bbox, x, y, *predicted_point),
                SafetyChecker.is_in_other_bbox(
                    x,
                    y,
                    s,
                    s_min,
                    s_max,
                    prediction_at_time,
                    t,
                    scenario_start_time,
                    fps,
                    npc_starting,
                    npc_forward,
                ),
            ),
        )
        return predicate

    @staticmethod
    def get_predicates(
        ego_bbox: List[Tuple],
        pred_wps: np.ndarray,
        ego_speed: float,
        prediction_at_time: float,
        num_frames: int,
        fps: int,
        s_min: float,
        s_max: float,
        scenario_start_time: float,
        npc_starting: List[Tuple[float, float]],
        npc_forward: Tuple[float, float],
    ):
        predicates = []
        predicted_points = SafetyChecker.get_trajectory(
            ego_bbox, pred_wps, ego_speed, num_frames, fps
        )
        for t in range(num_frames):
            pred = SafetyChecker.is_safe(
                ego_bbox,
                predicted_points[t],
                prediction_at_time,
                t,
                s_min,
                s_max,
                scenario_start_time,
                fps,
                npc_starting,
                npc_forward,
            )
            predicates.append(pred)
        return predicates, predicted_points

    @staticmethod
    def get_collision_predicate(
        ego_bbox,
        start_time,
        pred_wps,
        ego_speed,
        num_frames,
        fps,
        s_min,
        s_max,
        scenario_start_time,
        npc_starting,
        npc_forward,
    ) -> z3.ExprRef:
        predicates, ego_trajectory = SafetyChecker.get_predicates(
            ego_bbox,
            pred_wps,
            ego_speed,
            start_time,
            num_frames,
            fps,
            s_min,
            s_max,
            scenario_start_time,
            npc_starting,
            npc_forward,
        )
        return z3.Or(*predicates), ego_trajectory

    @staticmethod
    def get_trajectory(
        ego_bbox: List[Tuple[float, float]],
        pred_wps: np.ndarray,
        ego_speed: float,
        num_frames: int,
        fps: int,
    ) -> List[List[float]]:
        """
        Get the trajectory of the ego vehicle based on predicted waypoints and speed.
        Args:
            pred_wps (np.ndarray): Predicted waypoints for the ego vehicle.
            ego_speed (float): Speed of the ego vehicle.
            num_frames (int): Duration of the simulation in seconds.
            fps (int): Frames per second for the simulation.
        Returns:
            List: A list of tuples representing the trajectory (x, y, yaw) of the ego vehicle for
                (num_frames) time steps.
        """
        ego_x, ego_y, ego_yaw = SafetyChecker.to_xyyaw(
            ego_bbox, SafetyChecker.EGO_VEHICLE_X_AXIS_LENGTH
        )
        current_locs = np.array([ego_x, ego_y])
        ego_yaw = ego_yaw.item()  # Convert to float
        ego_model = EgoModel(dt=1.0 / fps)

        dataset: List[Tuple[float, float, float]] = []

        for _ in range(num_frames):
            pred_wps_copy = pred_wps.copy()
            # Adjust the predicted waypoints to be relative to the ego vehicle
            pred_wps_copy[:, 0] -= current_locs[0]
            pred_wps_copy[:, 1] -= current_locs[1]
            cos_yaw = np.cos(-ego_yaw)
            sin_yaw = np.sin(-ego_yaw)
            rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            pred_wps_copy = np.dot(
                pred_wps_copy, rotation_matrix.T
            )  # Rotate waypoints to match ego vehicle's orientation
            forward_wps = []
            for wp in pred_wps_copy:
                if wp[0] < 0.0 and wp[1] < 0.0:
                    continue
                forward_wps.append(wp)

            # Need at least two waypoints to calculate control
            if len(forward_wps) < 2:
                break

            steer, throttle, brake = control_pid(forward_wps, ego_speed)
            yaw = np.array([ego_yaw])
            speed = np.array([ego_speed])
            action = np.array(np.stack([steer, throttle, brake], axis=-1))
            new_locs, new_yaw, new_speed = ego_model.forward(
                current_locs, yaw, speed, action
            )

            current_locs = new_locs
            ego_yaw = new_yaw
            ego_speed = new_speed
            datapoint = [current_locs[0], current_locs[1], ego_yaw.item()]
            dataset.append(datapoint)

        if len(dataset) == 0:
            dataset = [[ego_x, ego_y, ego_yaw.item()] for _ in range(num_frames)]
        elif len(dataset) < num_frames:
            last_point = dataset[-1]
            pad_size = num_frames - len(dataset)
            # Pad the dataset with the last point to ensure it has the correct length
            dataset.extend([last_point] * pad_size)

        return dataset

    @staticmethod
    def get_datapoint(
        ego_bbox,
        start_time,
        pred_wps,
        ego_speed,
        solver,
        debug,
        num_frames,
        fps,
        s_min,
        s_max,
        scenario_start_time,
        npc_starting,
        npc_forward,
    ):
        safety, ego_trajectory = SafetyChecker.get_collision_predicate(
            ego_bbox,
            start_time,
            pred_wps,
            ego_speed,
            num_frames,
            fps,
            s_min,
            s_max,
            scenario_start_time,
            npc_starting,
            npc_forward,
        )
        solver.add(safety)
        result = solver.check()
        if result == z3.sat:
            if debug:
                print("Unsafe: There exists a collision.")
                model = solver.model()
                print("Model:", model)
            return (
                start_time,
                ego_trajectory,
                False,
            )  # Unsafe, there exists a collision
        else:
            return (
                start_time,
                ego_trajectory,
                True,
            )  # Safe, no collision exists for the given parameters


if __name__ == "__main__":
    dataset = None
    dataset_path = "transformed_dataset.csv" if len(sys.argv) < 2 else sys.argv[1]
    try:
        dataset = pd.read_csv(dataset_path)
    except FileNotFoundError:
        print(f"Dataset file {dataset_path} not found.")
        sys.exit(1)
    dataset_with_classification = []
    for index, row in dataset.iterrows():
        print(index, "/", len(dataset))
        ego_bbox = json.loads(row["ego_bbox"])
        pred_wps = np.array(json.loads(row["waypoints"]))
        ego_speed = 4.0  # Assuming a constant speed for the ego vehicle
        start_time = row["timestamp"]
        num_frames = 2
        fps = 2
        s_min = 3.0
        s_max = 15.0
        scenario_start_time = 1.0
        npc_starting = json.loads(row["npc_bbox"])
        npc_forward = json.loads(row["npc_forward"])
        solver = z3.Solver()
        checker = SafetyChecker()
        result = checker.get_datapoint(
            ego_bbox=ego_bbox,  # var
            start_time=start_time,  # var
            pred_wps=pred_wps,
            ego_speed=ego_speed,  # var
            solver=solver,
            debug=False,
            num_frames=num_frames,
            fps=fps,
            s_min=s_min,
            s_max=s_max,
            scenario_start_time=scenario_start_time,
            npc_starting=npc_starting,  # fixed
            npc_forward=npc_forward,  # fixed
        )
        xs = [f"x_{i}" for i in range(num_frames)]
        ys = [f"y_{i}" for i in range(num_frames)]
        cyaws = [f"sin_yaw_{i}" for i in range(num_frames)]
        syaws = [f"cos_yaw_{i}" for i in range(num_frames)]
        boxs = [(xs[i], ys[i], cyaws[i], syaws[i]) for i in range(num_frames)]
        datapoint = {
            "timestamp": start_time,
        }
        for i, (x, y, cyaw, syaw) in enumerate(zip(xs, ys, cyaws, syaws)):
            datapoint[x] = result[1][i][0]
            datapoint[y] = result[1][i][1]
            datapoint[cyaw] = np.cos(result[1][i][2])
            datapoint[syaw] = np.sin(result[1][i][2])
        datapoint["label"] = "true" if result[2] else "false"
        dataset_with_classification.append(datapoint)

    # Save the dataset with classification
    output_path = "classified_dataset.csv"
    output_df = pd.DataFrame(dataset_with_classification)
    output_df.to_csv(output_path, index=False)
    print(f"Dataset with classification saved to {output_path}")


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
# ego_bbox = [(-2.446798801422119, -1.0641616582870483), (-2.446798801422119, 1.0641626119613647), (2.4548845291137695, 1.0641626119613647), (2.4548845291137695, -1.0641616582870483)]

# solver = z3.Solver()
# checker = SafetyChecker(
# )
# x = z3.Real("x")
# y = z3.Real("y")
# s = z3.Real("s")
# t = z3.Real("t")

# result = checker.get_datapoint(
#     ego_bbox=ego_bbox, # var
#     start_time=1.0, # var
#     pred_wps=pred_wps, # var, pretransform
#     ego_speed=4.0, # var
#     solver=solver,
#     debug=True,
#     num_frames=2,
#     fps=20,
#     s_min=3.0,
#     s_max=15.0,
#     scenario_start_time=1.0,
#     npc_starting=npc_starting, # fixed
#     npc_forward=npc_forward # fixed
# )
# print("Result:", result)

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
