import carla
import numpy as np
import math
from leaderboard.scenarios.perception_contract.controller import control_pid, EgoModel
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from leaderboard.scenarios.perception_contract.sat import check_collision
import scipy.spatial as sp


def plot_3d_bboxes(obj_boxes, color, ax, label, fps=20.0):
    for t, box in enumerate(obj_boxes):
        # Add the z (time) coordinate
        box = [box[0], box[1], box[3], box[2]]
        box_3d = [(x, y, t/fps) for x, y in box]
        poly = Poly3DCollection([box_3d], alpha=0.6)
        poly.set_facecolor(color)
        ax.add_collection3d(poly)

def is_wp_safe(pred_wps, ego_dimensions, ego_speed, other_bbox, other_forward, other_speed, other_speed_max=10, time=2, fps=20, debug=False):
    """
    Check if the predicted waypoints are safe from other vehicles.

    Args:
        pred_wps (np.ndarray): Predicted waypoints of shape (4, 2).
        ego_dimensions (np.ndarray): Current bounding box of the ego_vehicle, centered at (0, 0).
        other_bbox (np.ndarray): Bounding box of the other vehicle, relative to the ego vehicle.
        other_forward (np.ndarray): Unit forward vector of the other vehicle.
        other_speed (float): Speed of the other vehicle.
        other_speed_max (float): Maximum speed of the other vehicle.
        time (int): Time in seconds to check for safety.
        fps (int): Frames per second for the simulation.
        debug (bool): If True, will print debug information and plot the bounding boxes.
    Returns:
        bool: True if the waypoints are safe, False otherwise.
        ego_bounding_boxes (np.ndarray): Bounding boxes of the ego vehicle over time.
    """

    other_bboxes = {}
    for i in range(time * fps):
        other_distance = other_speed * (i / fps)
        move_vector = other_forward * other_distance
        other_bboxes[i] = [
            (vertex[0] + move_vector[0], vertex[1] + move_vector[1]) for vertex in other_bbox
        ]

    ego_bboxes = {}
    current_locs = np.array([0.0, 0.0])     # Ego vehicle starts at the origin
    ego_yaw = 0.0                           # The wps are predicted assuming the vehicle is at the origin and along x=0
    ego_model = EgoModel(dt=1./fps)
    steer, throttle, brake = control_pid(pred_wps, ego_speed)
    yaw = np.array([ego_yaw])
    speed = np.array([ego_speed])
    action = np.array(np.stack([steer, throttle, brake], axis=-1))
    
    ego_bounding_boxes = np.array([
        [current_locs[0], current_locs[1], yaw[0]],
    ])
    for i in range(time * fps):
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
        
        diff_vector = np.array(new_locs)
        ego_bboxes[i] = [
            [vertex[0] + diff_vector[0], vertex[1] + diff_vector[1]] for vertex in ego_dimensions
        ]
        current_locs = new_locs
        ego_yaw = new_yaw
        ego_speed = new_speed
        datapoint = np.array([current_locs[0], current_locs[1], ego_yaw])
        ego_bounding_boxes = np.append(ego_bounding_boxes, [datapoint], axis=0)

    if debug:
        print("Ego BBoxes:")
        for i, bbox in enumerate(ego_bounding_boxes):
            print(f"Time {i/fps:.2f}s: {bbox}")
        print("Other BBoxes:")
        for i, bbox in other_bboxes.items():
            print(f"Time {i/fps:.2f}s: {bbox}")
            
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plot_3d_bboxes(list(ego_bboxes.values()), 'blue', ax, 'Ego Vehicle', fps)
        plot_3d_bboxes(list(other_bboxes.values()), 'red', ax, 'Other Vehicle', fps)

        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_zlabel('Time (s)')
        ax.set_title('Bounding Boxes Over Time')
        ax.set_xlim(-15, 15)
        ax.set_ylim(15, -15)
        ax.set_zlim(0, 3)
        plt.grid()
        plt.savefig('bounding_boxes_3d.png', dpi=300)
        plt.show()

        fig.clear()

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
        pred_wps = np.array(pred_wps)
        ax.plot(pred_wps[:, 0], pred_wps[:, 1], marker='o', color='green', label='Predicted Waypoints')
        ax.set_xlim(-15, 15)
        ax.set_ylim(15, -15)
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Bounding Boxes Over Time')
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        plt.grid()
        plt.savefig('bounding_boxes.png', dpi=300)
        plt.show()

    for i in range(min((time * fps), len(ego_bboxes), len(other_bboxes))):
        ego_bbox_i = ego_bboxes[i]
        other_bbox_i = other_bboxes[i]
        
        # Check if the bounding boxes intersect
        if check_collision(ego_bbox_i, other_bbox_i):
            print(f"Collision detected at time {i/fps:.2f}s")
            return (False, ego_bounding_boxes)

    print("No collisions detected.")
    return (True, ego_bounding_boxes)


pred_wps = np.array([[ 1.735676,   0.01005581],
 [ 3.6433578,  -0.00992398],
 [ 5.6090746,  -0.03921339],
 [ 7.597434,  -0.07129178]])
ego_bbox = np.array([(-2.446798801422119, -1.0641616582870483), (-2.446798801422119, 1.0641626119613647), (2.4548845291137695, -1.0641616582870483), (2.4548845291137695, 1.0641626119613647)])
ego_speed = 3.7946761
other_bbox = np.array([(7.773118495941162, 3.3192648887634277), (8.145111083984375, 3.299520492553711), (7.686257362365723, 1.6841745376586914), (8.058249473571777, 1.6644301414489746)])
other_forward = np.array([-0.05287253, -0.99528052])
other_speed = 3

is_wp_safe(pred_wps, ego_bbox, ego_speed, other_bbox, other_forward, other_speed, debug=True)
    

