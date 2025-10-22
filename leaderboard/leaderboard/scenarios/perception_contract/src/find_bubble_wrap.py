import gurobipy as gp
from gurobipy import GRB
import json
import sys
import numpy as np
from typing import List, Tuple

def add_no_intersection_constraint(model, bubble_corners, npc_corners, M=1e4, timestamp=None):
    """
    Add constraints to ensure bubble and NPC bounding boxes don't intersect.
    Uses Separating Axis Theorem (SAT).
    
    bubble_corners: 4 corners of axis-aligned bubble [upper_left, upper_right, lower_right, lower_left]
    npc_corners: 4 corners of potentially rotated NPC bbox
    """
    
    def get_edge_normals(corners):
        """Get outward-pointing normals for each edge"""
        normals = []
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            # Edge vector
            ex = x2 - x1
            ey = y2 - y1
            # Perpendicular (normal) - we use (-ey, ex) for outward normal
            normals.append((-ey, ex))
        return normals
    
    # For axis-aligned bubble, we only need 2 axes (x and y)
    # For rotated NPC, we need its 2 unique edge normals
    bubble_axes = [(1, 0), (0, 1)]  # Simply use cardinal axes for axis-aligned rectangle
    npc_normals = get_edge_normals(npc_corners)
    
    # NPC has 4 normals, but opposite edges have parallel normals (negatives of each other)
    # We only need 2 unique directions
    npc_axes = [npc_normals[0], npc_normals[1]]
    
    # All axes to test
    all_axes = bubble_axes + npc_axes
    
    separation_binaries = []
    
    for idx, (ax_x, ax_y) in enumerate(all_axes):
        # Normalize axis for numerical stability
        axis_length = (ax_x**2 + ax_y**2)**0.5
        if axis_length > 1e-8:
            ax_x = ax_x / axis_length
            ax_y = ax_y / axis_length
        
        # Binary variable: 1 if this axis separates the shapes
        sep_var = model.addVar(vtype=GRB.BINARY, name=f"sep_axis_{idx}_{timestamp}")
        separation_binaries.append(sep_var)
        
        # Project all bubble corners onto this axis
        bubble_proj_vars = []
        for j, (x, y) in enumerate(bubble_corners):
            v = model.addVar(lb=-GRB.INFINITY, name=f"bubble_proj_{idx}_{j}_{timestamp}")
            model.addConstr(v == ax_x * x + ax_y * y)
            bubble_proj_vars.append(v)
        
        # Project all NPC corners onto this axis
        npc_proj_vars = []
        for j, (x, y) in enumerate(npc_corners):
            v = model.addVar(lb=-GRB.INFINITY, name=f"npc_proj_{idx}_{j}_{timestamp}")
            model.addConstr(v == ax_x * x + ax_y * y)
            npc_proj_vars.append(v)
        
        # Find min and max projections for bubble
        bubble_min = model.addVar(lb=-GRB.INFINITY, name=f"bubble_min_{idx}_{timestamp}")
        bubble_max = model.addVar(lb=-GRB.INFINITY, name=f"bubble_max_{idx}_{timestamp}")
        model.addGenConstrMin(bubble_min, bubble_proj_vars)
        model.addGenConstrMax(bubble_max, bubble_proj_vars)
        
        # Find min and max projections for NPC
        npc_min = model.addVar(lb=-GRB.INFINITY, name=f"npc_min_{idx}_{timestamp}")
        npc_max = model.addVar(lb=-GRB.INFINITY, name=f"npc_max_{idx}_{timestamp}")
        model.addGenConstrMin(npc_min, npc_proj_vars)
        model.addGenConstrMax(npc_max, npc_proj_vars)
        
        # Add separation constraints with small gap (1e-3) for numerical stability
        # If sep_var = 1: bubble is completely on one side (bubble_max <= npc_min)
        # If sep_var = 0: npc is completely on one side (npc_max <= bubble_min)
        gap = 1e-3
        model.addConstr(bubble_max <= npc_min - gap + M * (1 - sep_var), 
                       name=f"sep_{idx}_bubble_left")
        model.addConstr(npc_max <= bubble_min - gap + M * sep_var, 
                       name=f"sep_{idx}_npc_left")
    
    # At least one axis must show separation
    model.addConstr(gp.quicksum(separation_binaries) >= 1, "non_intersection")

def find_buble_wrap(dataset_path, car_dims, default_bubble_size=None):
    with open(dataset_path, "r") as f:
        dataset = json.load(f)

    if default_bubble_size is None:
        default_bubble_size = {"x": 1, "y": 4}

    x_0 = car_dims["width"] / 2
    y_0 = car_dims["length"] / 2

    model = gp.Model("find_bubble_wrap")
    model.setParam("OutputFlag", 0)
    model.setParam("TimeLimit", 600)

    # Bubble variables
    x_1 = model.addVar(lb=-default_bubble_size["x"] - x_0, ub=-x_0, name="x_1")
    x_2 = model.addVar(lb=x_0, ub=default_bubble_size["x"] + x_0, name="x_2")
    y = model.addVar(lb=y_0, ub=default_bubble_size["y"] + y_0, name="y")

    # Assume the bubble is axis-aligned with the car
    upper_left = (x_1, y)
    upper_right = (x_2, y)
    lower_left = (x_1, -y_0)
    lower_right = (x_2, -y_0)
    bubble_corners = [upper_left, upper_right, lower_right, lower_left]

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

        add_no_intersection_constraint(model, bubble_corners, npc_corners, db_idx)

    # Example objective: maximize bubble area
    model.setObjective((x_2 - x_1) * (y - (-y_0)), GRB.MAXIMIZE)
    model.optimize()


    print(f"Total variables: {model.NumVars}, Total constraints: {model.NumConstrs}")
    print(f"Constraints:")
    model.write("model.lp")

    if model.status == GRB.OPTIMAL or model.status == GRB.SUBOPTIMAL:
        return {
            "x_1": x_1.X,
            "x_2": x_2.X,
            "y": y.X,
            "area": (x_2.X - x_1.X) * (y.X + y_0),
        }
    else:
        print(f"Model ended with status {model.status}")
        return None


if __name__ == "__main__":
    dataset_path = "datasets_v1/dataset_velocity_0.5.json"
    car_dimensions = {"length": 4.5, "width": 2.16}
    bubble_wrap = find_buble_wrap(dataset_path, car_dimensions)
    if bubble_wrap is not None:
        print("Optimal Bubble Wrap Dimensions:")
        print(f"x_1: {bubble_wrap['x_1']}")
        print(f"x_2: {bubble_wrap['x_2']}")
        print(f"y: {bubble_wrap['y']}")
        print(f"Area: {bubble_wrap['area']}")
    else:
        print("No optimal bubble wrap found.")
