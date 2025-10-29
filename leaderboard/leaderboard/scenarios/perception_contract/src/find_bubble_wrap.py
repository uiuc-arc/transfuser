import gurobipy as gp
from gurobipy import GRB
import json
import sys
import numpy as np
from typing import List, Tuple
import matplotlib.pyplot as plt

EPSILON = 1e-1


def add_no_intersection_constraint(
    model: gp.Model, bubble_corners: List[gp.Var], npc_corners: List[float], **kwargs
):
    """
    Add constraints ensuring that the bubble and NPC boxes do not intersect.

    Args:
        model (gp.Model): Gurobi model.
        bubble_corners (List[Tuple[gp.Var, gp.Var]]): 4 corners of axis-aligned bubble.
        npc_corners (List[Tuple[float, float]]): 4 corners of possibly rotated NPC box.
        kwargs: optional, may include:
            M (float): Big-M value for disjunction (default: 1e4)
            margin (float): minimum separation distance (default: 1e-3)
    """

    M = kwargs.get("M", 1e4)
    margin = kwargs.get("margin", 1e-3)

    def compute_axes(corners):
        axes = []
        for i in range(2):  # only two unique edges per rectangle
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            ex, ey = x2 - x1, y2 - y1
            # perpendicular vector (-ey, ex)
            nx, ny = -ey, ex
            norm = (nx**2 + ny**2) ** 0.5
            if norm > 1e-9:
                nx /= norm
                ny /= norm
            axes.append((nx, ny))
        return axes

    bubble_axes = [(1.0, 0.0), (0.0, 1.0)]
    npc_axes = compute_axes(npc_corners)
    all_axes = bubble_axes + npc_axes

    sep_binaries = []

    for idx, (ax_x, ax_y) in enumerate(all_axes):
        sep = model.addVar(vtype=GRB.BINARY, name=f"sep_axis_{idx}")
        sep_binaries.append(sep)

        def projection_vars(corners, prefix):
            proj_vars = []
            for j, (x, y) in enumerate(corners):
                v = model.addVar(lb=-GRB.INFINITY, name=f"{prefix}_{idx}_{j}")
                if isinstance(x, gp.Var) or isinstance(y, gp.Var):
                    model.addConstr(v == ax_x * x + ax_y * y)
                else:
                    model.addConstr(v == ax_x * float(x) + ax_y * float(y))
                proj_vars.append(v)
            return proj_vars

        bubble_proj = projection_vars(bubble_corners, "bubble_proj")
        npc_proj = projection_vars(npc_corners, "npc_proj")

        bubble_min = model.addVar(lb=-GRB.INFINITY, name=f"bubble_min_{idx}")
        bubble_max = model.addVar(lb=-GRB.INFINITY, name=f"bubble_max_{idx}")
        npc_min = model.addVar(lb=-GRB.INFINITY, name=f"npc_min_{idx}")
        npc_max = model.addVar(lb=-GRB.INFINITY, name=f"npc_max_{idx}")

        model.addGenConstrMin(bubble_min, bubble_proj)
        model.addGenConstrMax(bubble_max, bubble_proj)
        model.addGenConstrMin(npc_min, npc_proj)
        model.addGenConstrMax(npc_max, npc_proj)

        model.addConstr(bubble_max <= npc_min - margin + M * sep)
        model.addConstr(npc_max <= bubble_min - margin + M * (1 - sep))

    model.addConstr(sum(sep_binaries) >= 1, "non_intersection")


def find_bubble_wrap(dataset_path, car_dims, default_bubble_size=None):

    with open(dataset_path, "r") as f:
        dataset = json.load(f)
        dataset = dataset["data"]
        sorted_dataset = sorted(dataset, key=lambda x: x["timestamp"])
        dataset = sorted_dataset
        dataset = dataset[:378]
        for idx, entry in enumerate(dataset):
            new_bbox = []
            for corner in entry["npc_bbox_relative"]:
                new_corner = (corner[0], corner[1])
                new_bbox.append(new_corner)
            dataset[idx]["npc_bbox_relative"] = new_bbox

        print(dataset)

    if default_bubble_size is None:
        default_bubble_size = {"x": 3, "y": 1}

    x_0 = car_dims["length"] / 2
    y_0 = car_dims["width"] / 2

    model = gp.Model("find_bubble_wrap")
    model.setParam("OutputFlag", 0)
    model.setParam("TimeLimit", 600)

    # Bubble variables
    x = model.addVar(lb=x_0 + EPSILON, ub=x_0 + default_bubble_size["x"], name="x")
    y_1 = model.addVar(lb=y_0, ub=default_bubble_size["y"] + y_0, name="y_1")
    y_2 = model.addVar(lb=-default_bubble_size["y"] - y_0, ub=-y_0, name="y_2")

    # Assume the bubble is axis-aligned with the car
    upper_left = (-x_0, y_1)
    lower_left = (-x_0, y_2)
    upper_right = (x, y_1)
    lower_right = (x, y_2)
    bubble_corners = [upper_left, upper_right, lower_right, lower_left]

    fig, ax = plt.subplots(figsize=(8, 6))
    for db_idx, entry in enumerate(dataset):
        npc_corners = entry["npc_bbox_relative"]
        if len(npc_corners) > 4:
            filtered_corners = []
            for corner in npc_corners:
                if corner not in filtered_corners and all(
                    np.allclose(corner, fc, atol=0.1) is False
                    for fc in filtered_corners
                ):
                    filtered_corners.append(corner)
            npc_corners = filtered_corners
        if len(npc_corners) != 4:
            continue  # Skip invalid entries

        sorted_unique_vertices = []
        center = np.mean(npc_corners, axis=0)
        sorted_indices = np.argsort(
            np.arctan2(
                [v[1] - center[1] for v in npc_corners],
                [v[0] - center[0] for v in npc_corners],
            )
        )
        for idx in sorted_indices:
            sorted_unique_vertices.append(npc_corners[idx])

        npc = sorted_unique_vertices
        npc = np.vstack([npc, npc[0]])  # close the box
        ax.plot(npc[:, 0], npc[:, 1], "r-", lw=1.5, alpha=0.6)

        add_no_intersection_constraint(model, bubble_corners, sorted_unique_vertices)

    # Example objective: maximize bubble area
    model.setObjective((x + x_0) * (y_1 - y_2), GRB.MAXIMIZE)
    model.optimize()

    model.write("model.lp")

    # Plot the car (for reference)
    car_rect = np.array(
        [
            [-x_0, -y_0],
            [-x_0, y_0],
            [x_0, y_0],
            [x_0, -y_0],
            [-x_0, -y_0],
        ]
    )
    ax.plot(car_rect[:, 0], car_rect[:, 1], "k-", lw=1, label="Car")

    bubble = None
    if model.status in (GRB.OPTIMAL, GRB.SUBOPTIMAL):
        bx, by1, by2 = x.X, y_1.X, y_2.X
        bubble = np.array(
            [
                [-x_0, by1],
                [bx, by1],
                [bx, by2],
                [-x_0, by2],
                [-x_0, by1],
            ]
        )
        ax.plot(bubble[:, 0], bubble[:, 1], "b-", lw=2.5, label="Bubble Wrap")
        ax.fill(bubble[:, 0], bubble[:, 1], color="blue", alpha=0.15)
        # plot the projections on each axis

        for idx in range(4):
            bubble_min = model.getVarByName(f"bubble_min_{idx}")
            bubble_max = model.getVarByName(f"bubble_max_{idx}")
            npc_min = model.getVarByName(f"npc_min_{idx}")
            npc_max = model.getVarByName(f"npc_max_{idx}")
            # plot projections
            ax.plot(
                [bubble_min.X, bubble_max.X],
                [15 + idx, 15 + idx],
                "b-",
                lw=4,
                label="Bubble Projection" if idx == 0 else "",
            )
            ax.plot(
                [npc_min.X, npc_max.X],
                [14 + idx, 14 + idx],
                "r-",
                lw=4,
                label="NPC Projection" if idx == 0 else "",
            )

    ax.set_aspect("equal", "box")
    ax.set_title("NPCs and Optimized Bubble Wrap")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_ylim(-20, 20)
    ax.set_xlim(-20, 20)
    ax.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("plot.png")
    plt.show()

    if model.status == GRB.OPTIMAL or model.status == GRB.SUBOPTIMAL:
        return {
            "x": x.X,
            "y_1": y_1.X,
            "y_2": y_2.X,
            "area": (x.X + x_0) * (y_1.X - y_2.X),
        }
    else:
        if model.status == GRB.INFEASIBLE:
            print("Model is infeasible.")
        else:
            print(f"Optimization ended with status {model.status}.")
        return None


if __name__ == "__main__":
    dataset_path = "datasets_v1/dataset_velocity_0.5.json"
    car_dimensions = {"length": 4.5, "width": 2.16}
    bubble_wrap = find_bubble_wrap(dataset_path, car_dimensions)
    if bubble_wrap is not None:
        print("Bubble Wrap Dimensions:")
        print(f"x: {bubble_wrap['x']}")
        print(f"y_1: {bubble_wrap['y_1']}")
        print(f"y_2: {bubble_wrap['y_2']}")
        print(f"Area: {bubble_wrap['area']}")
    else:
        print("No bubble wrap found.")
