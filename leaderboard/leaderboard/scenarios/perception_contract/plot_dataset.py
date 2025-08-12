from matplotlib import pyplot as plt
import numpy as np
import z3
from dtree_learner import DTreeLearner
from dtree_checker import DTreeChecker
import sympy
from sympy import *
import pandas as pd

np.random.seed(42)

bbox = [
    (-2.4508416652679443, 1.0641621351242065),
    (-2.4508416652679443, -1.0641621351242065),
    (2.4508416652679443, -1.0641621351242065),
    (2.4508416652679443, 1.0641621351242065),
]

npc_starting = np.array(
    [
        [162.85765075683594, 37.392520904541016],
        [162.85765075683594, 37.02000427246094],
        [161.21482849121094, 37.02000427246094],
        [161.21482849121094, 37.392520904541016],
    ]
)
npc_forward = np.array([-0.9999862909317017, -0.0012179769109934568])

datapoints = pd.read_csv("dataset/new_dataset0.data", header=None)
true_dps = datapoints[datapoints.iloc[:, -1] == True]\
# slice first 100 rows
true_dps = true_dps.iloc[:50, :]
false_dps = datapoints[datapoints.iloc[:, -1] == False]


def plot_points(points):
    fig, ax = plt.subplots()
    for point in points:
        # plot first point
        # rotate then translate the bounding box
        newbox = np.array(bbox)
        cos_yaw0 = point[3]
        sin_yaw0 = point[2]
        rotation_matrix = np.array([[cos_yaw0, -sin_yaw0], [sin_yaw0, cos_yaw0]])
        rotated_bbox = np.dot(newbox, rotation_matrix.T)
        for i in range(len(rotated_bbox)):
            rotated_bbox[i][0] += point[0]  # x_0
            rotated_bbox[i][1] += point[1]
        rotated_bbox = np.vstack((rotated_bbox, rotated_bbox[0]))  # close the polygon
        # print(f"Rotated Box: {rotated_bbox}")

        ax.plot(rotated_bbox[:, 0], rotated_bbox[:, 1], color="blue")
        ax.fill(rotated_bbox[:, 0], rotated_bbox[:, 1], color="blue", alpha=0.3)

        # plot second point
        box2 = []  # rotate then translate
        rotated_box2 = np.array(bbox)
        cos_yaw1 = point[7]
        sin_yaw1 = point[6]
        rotation_matrix2 = np.array([[cos_yaw1, -sin_yaw1], [sin_yaw1, cos_yaw1]])
        rotated_bbox2 = np.dot(rotated_box2, rotation_matrix2.T)
        for i in range(len(rotated_bbox2)):
            rotated_bbox2[i][0] += point[4]  # x_1
            rotated_bbox2[i][1] += point[5]
        rotated_bbox2 = np.vstack((rotated_bbox2, rotated_bbox2[0]))  # close the polygon
        # print(f"Rotated Box 2: {rotated_bbox2}")

        ax.plot(rotated_bbox2[:, 0], rotated_bbox2[:, 1], color="red")
        ax.fill(rotated_bbox2[:, 0], rotated_bbox2[:, 1], color="red", alpha=0.3)

    for i in range(40):
        time = i / 2.0
        box = []
        for j in range(npc_starting.shape[0]):
            box.append(
                (
                    npc_starting[j][0] + npc_forward[0] * time,
                    npc_starting[j][1] + npc_forward[1] * time,
                )
            )

        # plot the box. the box is four points in the form of (x, y)
        box = np.array(box)
        box = np.vstack((box, box[0]))  # close the polygon
        ax.plot(box[:, 0], box[:, 1], "blue")
        ax.fill(box[:, 0], box[:, 1], alpha=0.3)

    ax.set_xlim(120, 170)
    ax.set_ylim(10, 50)
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="box")

    plt.grid()
    plt.show()

if __name__ == "__main__":
    # Plot the points from the dataset
    points = []
    for _, row in true_dps.iterrows():
        point = (
            # row[0],  # time
            row[1],  # x_0
            row[2],  # y_0
            row[3],  # sin_yaw_0
            row[4],  # cos_yaw_0
            row[5],  # x_1
            row[6],  # y_1
            row[7],  # sin_yaw_1
            row[8]   # cos_yaw_1
        )
        points.append(point)
    print(points)
    plot_points(points)

