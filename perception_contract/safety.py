import carla
import numpy as np
import math
from controller import control_pid, EgoModel
import matplotlib.pyplot as plt
from sat import check_collision

def is_wp_safe(pred_wps, ego_bbox, ego_yaw, ego_speed, other_bbox, other_forward, other_speed, safety_distance=1.0, time=2, fps=20):
    """
    Check if the predicted waypoints are safe from other vehicles.

    Args:
        pred_wps (np.ndarray): Predicted waypoints of shape (4, 2).
        current_bbox (np.ndarray): Current bounding box of the ego_vehicle, centered at (0, 0).
        other_bbox (np.ndarray): Bounding box of the other vehicle, relative to the ego vehicle.
        other_speed (float): Speed of the other vehicle.
        other_forward (np.ndarray): Unit forward vector of the other vehicle.
        safety_distance (float): Minimum distance around the ego vehicle that needs to be clear.
        time (int): Time in seconds to check for safety.
        fps (int): Frames per second for the simulation.
    Returns:
        bool: True if the waypoints are safe, False otherwise.
    """
    pred_wps_copy = pred_wps.copy()
    # Assuming the bboxes are in the format ([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
    other_bboxes = {}
    for i in range(time * fps):
        other_distance = other_speed * (i / fps)
        move_vector = other_forward * other_distance
        other_bboxes[i] = [
            (vertex[0] + move_vector[0], vertex[1] + move_vector[1]) for vertex in other_bbox
        ]

    ego_bboxes = {}
    current_locs = np.array([0.0, 0.0])  # Ego vehicle starts at the origin
    ego_model = EgoModel()
    for i in range(time * fps):
        # Remove waypoints that are behind the ego vehicle (already passed)
        if pred_wps[0][0] <= current_locs[0] and pred_wps[0][1] <= current_locs[1]:
            pred_wps = pred_wps[1:]  # Remove the first waypoint
        # Need at least two waypoints to calculate control
        if len(pred_wps) < 2:
            break
        steer, throttle, brake = control_pid(pred_wps, ego_speed)
        yaw = np.array([ego_yaw])
        speed = np.array([ego_speed])
        action = np.array(np.stack([steer, throttle, brake], axis=-1))
        new_locs, new_yaw, new_speed = ego_model.forward(
            current_locs, yaw, speed, action)
        print(f"Time {i/fps:.2f}s: New locations: {new_locs}, New yaw: {new_yaw}, New speed: {new_speed}")
        diff_vector = np.array(new_locs)
        ego_bboxes[i] = [
            (vertex[0] + diff_vector[0], vertex[1] + diff_vector[1]) for vertex in ego_bbox
        ]
        current_locs = new_locs
        ego_yaw = new_yaw
        ego_speed = new_speed

    print("Ego BBoxes:")
    for i, bbox in ego_bboxes.items():
        print(f"Time {i/fps:.2f}s: {bbox}")
    print("Other BBoxes:")
    for i, bbox in other_bboxes.items():
        print(f"Time {i/fps:.2f}s: {bbox}")

    # Add visualization of bounding boxes
    fig, ax = plt.subplots()
    for i in range(min((time * fps), len(ego_bboxes), len(other_bboxes))):
        ego_bbox_i = [ego_bboxes[i][0], ego_bboxes[i][1], ego_bboxes[i][3], ego_bboxes[i][2]]  # Ensure correct order
        other_bbox_i = [other_bboxes[i][0], other_bboxes[i][1], other_bboxes[i][3], other_bboxes[i][2]]  # Ensure correct order
        
        # Plot ego bounding box
        ego_polygon = plt.Polygon(ego_bbox_i, fill=None, edgecolor='blue', linewidth=1, label='Ego Vehicle' if i == 0 else "")
        ax.add_patch(ego_polygon)
        
        # Plot other bounding box
        other_polygon = plt.Polygon(other_bbox_i, fill=None, edgecolor='red', linewidth=1, label='Other Vehicle' if i == 0 else "")
        ax.add_patch(other_polygon)
    # Draw the predicted waypoints
    pred_wps = np.array(pred_wps_copy)
    ax.plot(pred_wps[:, 0], pred_wps[:, 1], marker='o', color='green', label='Predicted Waypoints')
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Bounding Boxes Over Time')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    # ax.legend()
    plt.grid()
    plt.savefig('bounding_boxes.png', dpi=300)


    for i in range(min((time * fps), len(ego_bboxes), len(other_bboxes))):
        ego_bbox_i = ego_bboxes[i]
        other_bbox_i = other_bboxes[i]
        
        # Check if the bounding boxes intersect
        if check_collision(ego_bbox_i, other_bbox_i):
            print(f"Collision detected at time {i/fps:.2f}s")
            return False

    print("No collisions detected.")
    return True


pred_wps = np.array([[ 1.735676,   0.01005581],
 [ 3.6433578,  -0.00992398],
 [ 5.6090746,  -0.03921339],
 [ 7.597434,  -0.07129178]])
ego_bbox = np.array([(-2.446798801422119, -1.0641616582870483), (-2.446798801422119, 1.0641626119613647), (2.4548845291137695, -1.0641616582870483), (2.4548845291137695, 1.0641626119613647)])
ego_yaw = 0.054454
ego_speed = 3.7946761
other_bbox = np.array([(7.773118495941162, 3.3192648887634277), (8.145111083984375, 3.299520492553711), (7.686257362365723, 1.6841745376586914), (8.058249473571777, 1.6644301414489746)])
other_forward = np.array([-0.05287253, -0.99528052])
other_speed = 7

is_wp_safe(pred_wps, ego_bbox, ego_yaw, ego_speed, other_bbox, other_forward, other_speed)
    

