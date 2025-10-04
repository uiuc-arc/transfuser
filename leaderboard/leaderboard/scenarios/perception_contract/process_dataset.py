from typing import List, Tuple, Optional
import sys
import scipy.spatial as sp
import numpy as np
from leaderboard.scenarios.perception_contract.controller import control_pid, EgoModel
import z3
import json
import pandas as pd
import tqdm
import copy
from sys import exit

class SafetyChecker:

    EGO_VEHICLE_X_AXIS_LENGTH = 4.90168333  # Length of the ego vehicle in meters
    ego_bbox = [
        (-2.4508416652679443, 1.0641621351242065),
        (-2.4508416652679443, -1.0641621351242065),
        (2.4508416652679443, -1.0641621351242065),
        (2.4508416652679443, 1.0641621351242065),
    ]

    def __init__(
        self,
    ):
        pass

    @staticmethod
    def is_in_box(point, vertices):
        """
        (0<=AM⋅AB<=AB⋅AB)∧(0<=AM⋅AD<=AD⋅AD)
        """
        A = vertices[0]
        B = vertices[1]
        D = vertices[3]
        M = point
        AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
        AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
        AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
        AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)

        return z3.And(AM_AB >= 0, AM_AB <= AB_AB, AM_AD >= 0, AM_AD <= AD_AD)

    @staticmethod
    def filter_points(vertices):
        """
        If len(vertices) is > 4, filter them to keep the four unique points
        """
        if len(vertices) >= 4:
            unique_vertices = []
            if len(vertices) > 4:
                for vertex in vertices:
                    for unique_vertex in unique_vertices:
                        if np.allclose(vertex, unique_vertex, atol=1e-1):
                            break
                    else:
                        unique_vertices.append(vertex)
            else:
                unique_vertices = vertices

            sorted_unique_vertices = []
            center = np.mean(unique_vertices, axis=0)
            sorted_indices = np.argsort(
                np.arctan2(
                    [v[1] - center[1] for v in unique_vertices],
                    [v[0] - center[0] for v in unique_vertices],
                )
            )
            sorted_unique_vertices = [unique_vertices[i] for i in sorted_indices]
            # print(f"Filtered vertices: {sorted_unique_vertices}")
            if len(sorted_unique_vertices) != 4:
                raise ValueError("Incorrect number of vertices, expected 4.")
            return sorted_unique_vertices
        elif len(vertices) < 4:
            raise ValueError("Less than 4 vertices provided, expected 4.")

    @staticmethod
    def find_vertices_along_distance(vertices, distance):
        """
        Find vertex pairs that are at least 'distance' apart.
        """
        if len(vertices) > 4:
            raise ValueError("more than 4 vertices provided, expected 4.")
        sides = [[0, 1], [1, 2], [2, 3], [3, 0]]
        for side in sides:
            dist = np.linalg.norm(np.array(vertices[side[0]]) - np.array(vertices[side[1]]))
            if np.isclose(dist, distance, atol=1e-2):
                # print(f"Found side vertices at distance {dist}: {vertices[side[0]]}, {vertices[side[1]]}")
                return (vertices[side[0]], vertices[side[1]])
        print(
            "No vertices found that are at least 'distance' apart. Given vertices:",
            vertices,
            "Distance:",
            distance,
        )
        raise ValueError("Bad vertices")

    @staticmethod
    def to_xyyaw(vertices, x_dim):
        """
        Convert vertices to x, y, yaw format.
        """
        vertices = np.array(SafetyChecker.filter_points(vertices))
        # print(f"Converting vertices: {vertices}")
        point_a, point_b = SafetyChecker.find_vertices_along_distance(vertices, x_dim)
        yaw = np.arctan2((point_b[1] - point_a[1]) , (point_b[0] - point_a[0]))
        yaw = yaw % np.pi
        x = np.mean(vertices[:, 0]).item()
        y = np.mean(vertices[:, 1]).item()
        # print(f"Converted vertices to x: {x}, y: {y}, yaw: {np.rad2deg(yaw)} degrees")
        return x, y, yaw

    # @staticmethod
    # def is_in_conv_polytope(point: Tuple, polytope: List[Tuple]) -> z3.ExprRef:
    #     """
    #     Check if a point is inside a convex polytope defined by its vertices.
    #     Args:
    #         point (tuple): A tuple representing the point (could be 2D, 3D, etc.).
    #         polytope (list): A list of points that represent the vertices of the convex polytope.
    #     Returns:
    #         z3.Expr: A Z3 expression that is True if the point is inside the polytope.
    #     """
    #     hull = sp.ConvexHull(polytope)
    #     equations = hull.equations

    #     if len(equations[0]) != (len(point) + 1):
    #         print(f"Point: {point}, Polytope: {polytope}")
    #         raise ValueError("Point dimension does not match polytope dimension.")

    #     # Create a list of inequalities for the point to be inside the polytope
    #     inequalities = []
    #     for eq in equations:
    #         coefficients = eq[:-1]  # All but the last coefficient
    #         constant = eq[-1]  # The last coefficient is the constant term
    #         inequality = (
    #             sum(coeff * point[i] for i, coeff in enumerate(coefficients))
    #             <= -constant
    #         )
    #         inequalities.append(inequality)

    #     # Combine all inequalities into a single expression
    #     return z3.And(*inequalities)

    @staticmethod
    def is_in_ego_bbox(
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
        # Rotate the bounding box vertices by the yaw angle
        # Then translate them to the ego vehicle's position
        rotation_matrix = np.array([
            [np.cos(ego_yaw), -np.sin(ego_yaw)],
            [np.sin(ego_yaw), np.cos(ego_yaw)]
        ])
        transformed_ego_bbox = []
        for vertex in SafetyChecker.ego_bbox:
            rotated_vertex = np.dot(rotation_matrix, np.array(vertex))
            transformed_vertex = (rotated_vertex[0] + ego_x, rotated_vertex[1] + ego_y)
            transformed_ego_bbox.append(transformed_vertex)

        print(f"Transformed ego bbox: {list(map(lambda v: (v[0].item(), v[1].item()), transformed_ego_bbox))}") 
        res = SafetyChecker.is_in_box((x, y), transformed_ego_bbox)
        return res

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
        scenario_end_time: float,
        fps: int,
        npc_starting: List[Tuple[float, float]],
        npc_forward: Tuple[float, float],
    ) -> z3.ExprRef:

        predicates = []
        effective_time = prediction_at_time + (timestep * (1.0 / fps))

        if effective_time < scenario_start_time:
            predicates.append(s == 0)
        elif scenario_start_time <= effective_time <= scenario_end_time:
            predicates.append(z3.And(s >= s_min, s <= s_max))
        else:
            predicates.append(s == 0)

        t = np.maximum(0, effective_time - scenario_start_time)
        
        npc_x_0 = (npc_starting[0][0] + npc_forward[0] * s * t)
        npc_y_0 = (npc_starting[0][1] + npc_forward[1] * s * t)
        npc_x_1 = (npc_starting[1][0] + npc_forward[0] * s * t)
        npc_y_1 = (npc_starting[1][1] + npc_forward[1] * s * t)
        npc_x_2 = (npc_starting[2][0] + npc_forward[0] * s * t)
        npc_y_2 = (npc_starting[2][1] + npc_forward[1] * s * t)
        npc_x_3 = (npc_starting[3][0] + npc_forward[0] * s * t)
        npc_y_3 = (npc_starting[3][1] + npc_forward[1] * s * t)        

        # predicates.append(
        #     z3.And(
        #         npc_x_0 == (npc_starting[0][0] + npc_forward[0] * s * t),
        #         npc_y_0 == (npc_starting[0][1] + npc_forward[1] * s * t),
        #         npc_x_1 == (npc_starting[1][0] + npc_forward[0] * s * t),
        #         npc_y_1 == (npc_starting[1][1] + npc_forward[1] * s * t),
        #         npc_x_2 == (npc_starting[2][0] + npc_forward[0] * s * t),
        #         npc_y_2 == (npc_starting[2][1] + npc_forward[1] * s * t),
        #         npc_x_3 == (npc_starting[3][0] + npc_forward[0] * s * t),
        #         npc_y_3 == (npc_starting[3][1] + npc_forward[1] * s * t),
        #     ),
        # )

        npc_vertices = [(npc_x_0, npc_y_0), (npc_x_1, npc_y_1), (npc_x_2, npc_y_2), (npc_x_3, npc_y_3)]

        res = SafetyChecker.is_in_box((x, y), npc_vertices)
        predicates.append(res)
        return z3.And(*predicates)

    @staticmethod
    def is_unsafe_at_t(
        predicted_point: List[float],
        prediction_at_time: float,
        t: int,
        s_min: float,
        s_max: float,
        scenario_start_time: float,
        scenario_end_time: float,
        fps: int,
        npc_starting: List[Tuple[float, float]],
        npc_forward: Tuple[float, float],
    ) -> z3.ExprRef:
        x = z3.Real("x")
        y = z3.Real("y")
        s = z3.Real("s")
        predicate = z3.And(
                SafetyChecker.is_in_ego_bbox(x, y, *predicted_point),
                SafetyChecker.is_in_other_bbox(
                    x,
                    y,
                    s,
                    s_min,
                    s_max,
                    prediction_at_time,
                    t,
                    scenario_start_time,
                    scenario_end_time,
                    fps,
                    npc_starting,
                    npc_forward,
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
        scenario_end_time: float,
        npc_starting: List[Tuple[float, float]],
        npc_forward: Tuple[float, float],
    ):
        predicates = []
        predicted_points = SafetyChecker.get_trajectory(
            ego_bbox, pred_wps, ego_speed, fps
        )
        for t, dp in enumerate(predicted_points):
            pred = SafetyChecker.is_unsafe_at_t(
                dp[0],
                prediction_at_time,
                t,  # t starts from 0, first frame is when everything is still
                s_min,
                s_max,
                scenario_start_time,
                scenario_end_time,
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
        scenario_end_time,
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
            scenario_end_time,
            npc_starting,
            npc_forward,
        )
        return predicates, ego_trajectory

    @staticmethod
    def convert_wps_to_global_coordinates(
        pred_wps: np.ndarray,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
    ) -> np.ndarray:
        """
        Convert predicted waypoints to the vehicle's coordinate system.
        """
        pred_wps_copy = pred_wps.copy()
        # rotate waypoints to match ego vehicle's orientation
        cos_yaw = np.cos(ego_yaw)
        sin_yaw = np.sin(ego_yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_wps = np.dot(pred_wps_copy, rotation_matrix.T)
        # translate waypoints to be relative to the ego vehicle's position
        rotated_wps[:, 0] += ego_x
        rotated_wps[:, 1] += ego_y
        return rotated_wps
    
    @staticmethod
    def convert_wps_to_vehicle_coordinates(
        pred_wps: np.ndarray,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
    ) -> np.ndarray:
        """
        Convert predicted waypoints to the vehicle's coordinate system.
        """
        pred_wps_copy = pred_wps.copy()
        # translate waypoints to be relative to the ego vehicle's position
        pred_wps_copy[:, 0] -= ego_x
        pred_wps_copy[:, 1] -= ego_y
        # rotate waypoints to match ego vehicle's orientation
        cos_yaw = np.cos(-ego_yaw)
        sin_yaw = np.sin(-ego_yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_wps = np.dot(pred_wps_copy, rotation_matrix.T)
        return rotated_wps
    
    @staticmethod
    def convert_points_to_vehicle_coordinates(
        vertices: List[Tuple[float, float]],
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
    ) -> List[Tuple[float, float]]:
        points = []
        translated_points = []
        for vertex in vertices:
            translated_vertex = (vertex[0] - ego_x, vertex[1] - ego_y)
            translated_points.append(translated_vertex)
            cos_yaw = np.cos(-ego_yaw)
            sin_yaw = np.sin(-ego_yaw)
            rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            rotated_vertex = np.dot(rotation_matrix, np.array(translated_vertex))
            points.append((rotated_vertex[0], rotated_vertex[1]))
        return points

    @staticmethod
    def get_trajectory(
        ego_bbox: List[Tuple[float, float]],
        pred_wps: np.ndarray,
        ego_speed: float,
        # num_frames: int,
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
        print(f"Local waypoints: {pred_wps}")
        global_waypoints = SafetyChecker.convert_wps_to_global_coordinates(
            pred_wps, ego_x, ego_y, ego_yaw
        )
        print(f"Global waypoints: {global_waypoints}")
        current_locs = np.array([ego_x, ego_y])
        ego_yaw = ego_yaw.item()  # Convert to float
        ego_model = EgoModel(dt=1.0 / fps)

        dataset: List[Tuple[float, float, float]] = [SafetyChecker.filter_points(ego_bbox)]
        g_dataset: List[List[Tuple[float, float]]] = [[current_locs[0], current_locs[1], ego_yaw]]

        while True:
            if ego_speed == 0.0:
                break
            local_waypoints = SafetyChecker.convert_wps_to_vehicle_coordinates(
                global_waypoints, current_locs[0], current_locs[1], ego_yaw
            )
            forward_wps = []
            for wp in local_waypoints:
                if wp[0] < 0.0:
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
            g_dataset.append([current_locs[0], current_locs[1], ego_yaw.item()])
            new_vertices = []
            for vertex in SafetyChecker.ego_bbox:
                rotated_vertex = np.dot(
                    np.array([[np.cos(ego_yaw), -np.sin(ego_yaw)], [np.sin(ego_yaw), np.cos(ego_yaw)]]),
                    np.array(vertex)
                )
                translated_vertex = (rotated_vertex[0] + current_locs[0], rotated_vertex[1] + current_locs[1])
                new_vertices.append(translated_vertex)
            dataset.append(new_vertices)

        # convert every point relative to initial ego vehicle position
        final_dataset = []
        initial_point = dataset[0]
        initial_x, initial_y, initial_yaw = SafetyChecker.to_xyyaw(
            initial_point, SafetyChecker.EGO_VEHICLE_X_AXIS_LENGTH
        )
        for idx, point in enumerate(dataset):
            new_point = SafetyChecker.convert_points_to_vehicle_coordinates(
                point, initial_x, initial_y, initial_yaw
            )
            final_dataset.append((g_dataset[idx], new_point))

        return final_dataset

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
        scenario_end_time,
        npc_starting,
        npc_forward,
    ):
        safety_preds, ego_trajectory = SafetyChecker.get_collision_predicate(
            ego_bbox,
            start_time,
            pred_wps,
            ego_speed,
            num_frames,
            fps,
            s_min,
            s_max,
            scenario_start_time,
            scenario_end_time,
            npc_starting,
            npc_forward,
        )
        for i, pred in enumerate(safety_preds):
            solver.push()
            solver.add(pred)
            result = solver.check()
            if result == z3.sat:
                if debug:
                    print("Unsafe: There exists a collision at timestep", i)
                    model = solver.model()
                    x = str(model.eval(z3.Real("x")).as_decimal(10))
                    y = str(model.eval(z3.Real("y")).as_decimal(10))
                    s = str(model.eval(z3.Real("s")).as_decimal(10))
                    if x.endswith("?"):
                        x = float(x[:-1])
                    else:
                        x = float(x)
                    if y.endswith("?"):
                        y = float(y[:-1])
                    else:
                        y = float(y)
                    new_point = SafetyChecker.convert_points_to_vehicle_coordinates(
                        [(x, y)],
                        ego_trajectory[0][0][0],
                        ego_trajectory[0][0][1],
                        ego_trajectory[0][0][2],
                    )
                    if s.endswith("?"):
                        s = float(s[:-1])
                    else:
                        s = float(s)

                    npc_vertices = []
                    effective_time = start_time + (i * (1.0 / fps))
                    effective_time = np.maximum(0, effective_time - scenario_start_time)
                    for vertex in npc_starting:
                        new_vertex = vertex[0] + s * effective_time * npc_forward[0], \
                        vertex[1] + s * effective_time * npc_forward[1]
                        npc_vertices.append(new_vertex)

                    print(f"NPC vertices: {list(map(lambda v: (v[0].item(), v[1].item()), npc_vertices))}")

                    converted_vertices = SafetyChecker.convert_points_to_vehicle_coordinates(
                        npc_vertices,
                        ego_trajectory[0][0][0],
                        ego_trajectory[0][0][1],
                        ego_trajectory[0][0][2],
                    )
                    print(f"Converted vertices: {converted_vertices}")
                    ego_vertices = []
                    for vertex in ego_trajectory[i][1]:
                        r_vertex = np.dot(
                            np.array([[np.cos(ego_trajectory[i][0][2]), -np.sin(ego_trajectory[i][0][2])],
                                      [np.sin(ego_trajectory[i][0][2]), np.cos(ego_trajectory[i][0][2])]]),
                            np.array(vertex)
                        )
                        rotated_vertex = (r_vertex[0] + ego_trajectory[i][0][0], r_vertex[1] + ego_trajectory[i][0][1])
                        ego_vertices.append((rotated_vertex[0], rotated_vertex[1]))
                    print(f"Ego vertices: {ego_trajectory[i][1]}")
                    

                solver.pop()
                solver.reset()
                return (
                    start_time,
                    converted_vertices + new_point,
                    False,
                )  # Unsafe, there exists a collision
            elif result == z3.unknown:
                if debug:
                    print("Unknown: The solver could not determine the safety.")
                    # print(solver.sexpr())
                solver.pop()
                solver.reset()
                return (
                    start_time,
                    ego_trajectory,
                    None,
                )
        return (
            start_time,
            ego_trajectory,
            True,
        )


if __name__ == "__main__":
    dataset = None
    dataset_path = "datasets_pc/dataset_47.5.csv" if len(sys.argv) < 2 else sys.argv[1]
    try:
        dataset = pd.read_csv(dataset_path)
    except FileNotFoundError:
        print(f"Dataset file {dataset_path} not found.")
        sys.exit(1)
    dataset_with_classification = []
    unknowns = []
    true_dps, false_dps = 0, 0
    for index, row in dataset.iterrows():
        print(index, "/", len(dataset))
        ego_bbox = json.loads(row["ego_bbox"])
        pred_wps = np.array(json.loads(row["waypoints"]))
        ego_speed = 4.0  # Assuming a constant speed for the ego vehicle
        start_time = row["timestamp"]
        num_frames = 2
        fps = 2
        s_min = 2.0
        s_max = 15.0
        scenario_start_time = 43.4
        scenario_end_time = 53.4
        npc_starting = [
            # [31.21482849121094, 57.392520904541016],
            # [31.2152862548828, 37.02000427246094],
            # [292.8581085205078, 37.02200698852539],
            # [292.85765075683594, 57.39452362060547],
            [161.2135467529297, 37.02000427246094],
            [162.85638427734375, 37.02199935913086],
            [162.85592651367188, 37.39451599121094],
            [161.2130889892578, 37.392520904541016],
        ]
        npc_forward = [-0.9999862909317017, -0.0012179769109934568]
        solver = z3.Solver()
        solver.set("timeout", 10000)  # Set a timeout for the solver
        solver.set("unsat_core", True)  # Enable unsat core extraction
        solver.set("smt.arith.nl", True)
        checker = SafetyChecker()
        if start_time < scenario_start_time:
            print(
                f"Skipping datapoint at index {index} with start_time {start_time} < scenario_start_time {scenario_start_time}"
            )
            continue
        result = checker.get_datapoint(
            ego_bbox=ego_bbox,  # var
            start_time=start_time,  # var
            pred_wps=pred_wps,  # var
            ego_speed=ego_speed,  # var
            solver=solver,
            debug=True,
            num_frames=num_frames,
            fps=fps,
            s_min=s_min,
            s_max=s_max,
            scenario_start_time=scenario_start_time,
            scenario_end_time=scenario_end_time,
            npc_starting=npc_starting,  # fixed
            npc_forward=npc_forward,  # fixed
        )
        if result[2] is None:
            print(f"Unknown result for datapoint at index {index}, skipping.")
            unknowns.append(index)
            continue

        if result[2]:
            traj = result[1]
            for bbox in traj:
                for vertex in bbox[1]:
                    dataset_with_classification.append({
                        # "timestamp": start_time,
                        # "speed": ego_speed,
                        "x": vertex[0],
                        "y": vertex[1],
                        "label": "true"
                    })
            true_dps += 1
        else:
            cex = result[1]
            for vertex in cex:
                    dataset_with_classification.append({
                        "x": vertex[0],
                        "y": vertex[1],
                        "label": "false"
                    })
            false_dps += 1

        print(f"True datapoints: {true_dps}, False datapoints: {false_dps}, Unknowns: {len(unknowns)}")

    # Save the dataset with classification
    output_path = "classified_dataset.csv"
    output_df = pd.DataFrame(dataset_with_classification)
    output_df.to_csv(output_path, index=False)
    print(f"Dataset with classification saved to {output_path}")

    unknowns_path = "unknowns.csv"
    if unknowns:
        unknown_dps = dataset.iloc[unknowns]
        unknowns_df = pd.DataFrame(unknown_dps)
        unknowns_df.to_csv(unknowns_path, index=False)
        print(f"Unknown indices saved to {unknowns_path}")
    else:
        print("No unknown indices found.")


