from matplotlib import pyplot as plt
import numpy as np
import z3
from dtree_learner import DTreeLearner
from dtree_checker import DTreeChecker
import sympy
from sympy import *
import copy
np.random.seed(42)

bbox = [
    (-2.4508416652679443, 1.0641621351242065),
    (-2.4508416652679443, -1.0641621351242065),
    (2.4508416652679443, -1.0641621351242065),
    (2.4508416652679443, 1.0641621351242065),
]

npc_starting = np.array(
    [
        # [292.85765075683594, 57.392520904541016],
        # [292.85765075683594, 37.02000427246094],
        # [31.21482849121094, 37.02000427246094],
        # [31.21482849121094, 57.392520904541016],
        [161.2135467529297, 37.02000427246094],
        [162.85638427734375, 37.02199935913086],
        [162.85592651367188, 37.39451599121094],
        [161.2130889892578, 37.392520904541016],
    ]
)
npc_forward = np.array([-0.9999862909317017, -0.0012179769109934568])


def get_trig_values():
    angles1 = np.random.uniform(-np.pi, np.pi, 10000)
    angles2 = np.random.uniform(-np.pi, np.pi, 10000)
    sin1 = np.sin(angles1)
    cos1 = np.cos(angles1)
    sin2 = np.sin(angles2)
    cos2 = np.cos(angles2)
    return sin1, cos1, sin2, cos2


def get_satisfying_points(inequalities, num_points=1000, sympy_vars=None):
    sin1, cos1, sin2, cos2 = get_trig_values()
    satisfying_points = []

    for pair in zip(sin1, cos1, sin2, cos2):
        while len(satisfying_points) < num_points:
            x0 = np.random.uniform(100, 250, size=1000)
            y0 = np.random.uniform(0, 100, size=1000)
            x1 = np.random.uniform(100, 250, size=1000)
            y1 = np.random.uniform(0, 100, size=1000)
            for i in range(len(x0)):
                x0_i = x0[i]
                y0_i = y0[i]
                x1_i = x1[i]
                y1_i = y1[i]

                vals = []
                for ineq in inequalities:
                    val = ineq.subs({
                        sympy_vars[0]: x0_i,
                        sympy_vars[1]: y0_i,
                        sympy_vars[2]: pair[0],
                        sympy_vars[3]: pair[1],
                        sympy_vars[4]: x1_i,
                        sympy_vars[5]: y1_i,
                        sympy_vars[6]: pair[2],
                        sympy_vars[7]: pair[3],
                    })
                    vals.append(val)
                if len(vals) == 2:
                    if vals[0] > 0 and vals[1] > 0:
                        satisfying_points.append(
                            (x0_i, y0_i, pair[0], pair[1], x1_i, y1_i, pair[2], pair[3])
                        )
                elif len(vals) == 3:
                    if vals[0] > 0 and vals[1] <= 0 and vals[2] <= 0:
                        satisfying_points.append(
                            (x0_i, y0_i, pair[0], pair[1], x1_i, y1_i, pair[2], pair[3])
                        )

                # val = ((1*pair[1]) + (-1*x1_i) + (-1*y1_i) + (1*pair[3])) - 153.1451

                # val = (
                #     ((1*pair[0]) + (1*x1_i) + (-1*pair[2]) + (1*pair[1]))
                # )

                # val = (
                #     1 * x0_i
                #     + 1 * pair[0]
                #     + -1 * pair[1]
                #     + 1 * x1_i
                #     + -1 * pair[2]
                #     + 1 * pair[3]
                # )
                # val = 1*y0_i +  -1*pair[0] + 1*pair[1] + 1*x1_i + -1*y1_i + -1*pair[2] + 1*pair[3]
                # val = y0_i + pair[0] - pair[1] + x1_i - y1_i - pair[2] + pair[3]
                # val = val - (1357993089561023/17592186044416)
                # if val <= 0:
                #     satisfying_points.append(
                #         (x0_i, y0_i, pair[0], pair[1], x1_i, y1_i, pair[2], pair[3])
                #     )

    return satisfying_points


def plot_points(points):
    fig, ax = plt.subplots()
    for point in points:
        # plot first point
        box = copy.deepcopy(bbox)
        cos_yaw0 = point[3]
        sin_yaw0 = point[2]
        rotation_matrix = np.array([[cos_yaw0, -sin_yaw0], [sin_yaw0, cos_yaw0]])
        rotated_box = np.dot(box, rotation_matrix.T)
        transformed_box = []
        for i in range(len(rotated_box)):
            transformed_box.append((rotated_box[i][0] + point[0], rotated_box[i][1] + point[1]))
        rotated_box = np.array(transformed_box)
        rotated_box = np.vstack((rotated_box, rotated_box[0]))  # close the polygon
        # plot the rotated box with blue color
        ax.plot(rotated_box[:, 0], rotated_box[:, 1], color="blue")
        ax.fill(rotated_box[:, 0], rotated_box[:, 1], color="blue", alpha=0.3)

        # plot second point
        box2 = copy.deepcopy(bbox)  # rotate then translate
        rotated_box2 = np.array(box2)
        cos_yaw1 = point[7]
        sin_yaw1 = point[6]
        rotation_matrix2 = np.array([[cos_yaw1, -sin_yaw1], [sin_yaw1, cos_yaw1]])
        rotated_box2 = np.dot(rotated_box2, rotation_matrix2.T)
        box2 = []
        # Translate the bounding box vertices by the second point's coordinates
        for i in range(len(rotated_box2)):
            box2.append((rotated_box2[i][0] + point[4], rotated_box2[i][1] + point[5]))
        box2 = np.array(box2)
        rotated_box2 = np.vstack((box2, box2[0]))  # close the polygon
        # plot the rotated box with red color
        ax.plot(rotated_box2[:, 0], rotated_box2[:, 1], color="red")
        ax.fill(rotated_box2[:, 0], rotated_box2[:, 1], color="red", alpha=0.3)

    for i in range(2):
        speed = 2.0
        time = i / 2.0
        box = []
        for j in range(npc_starting.shape[0]):
            box.append(
                (
                    npc_starting[j][0] + npc_forward[0] * time * speed,
                    npc_starting[j][1] + npc_forward[1] * time * speed,
                )
            )

        # plot the box. the box is four points in the form of (x, y)
        box = np.array(box)
        box = np.vstack((box, box[0]))  # close the polygon
        ax.plot(box[:, 0], box[:, 1], "blue")
        ax.fill(box[:, 0], box[:, 1], alpha=0.3)

    ax.set_xlim(90, 260)
    ax.set_ylim(-10, 110) 
    ax.set_aspect("equal", adjustable="box")

    plt.grid()
    plt.show()


def plot_conjunct(conjunct):
    print("Conjunct:", conjunct)
    time, x0, y0, sin_yaw0, cos_yaw0, x1, y1, sin_yaw1, cos_yaw1 = symbols(
        "time x0 y0 sin_yaw0 cos_yaw0 x1 y1 sin_yaw1 cos_yaw1", real=True
    )

    inequality1 = (
        x0 + y0 + sin_yaw0 - cos_yaw0 - y1 + cos_yaw1
        > 6721618221725815 / 70368744177664
    )
    inequality2 = sin_yaw0 * sin_yaw0 + cos_yaw0 * cos_yaw0 - 1 == 0
    inequality3 = sin_yaw1 * sin_yaw1 + cos_yaw1 * cos_yaw1 - 1 == 0

    # Fix intervals for each variable.
    # Push the interval through the inequality (x in terms of rest of vars, push intervals through, and I get a range for x)

    # Define your constraints
    # Inequality
    ineq = x0 + y0 + sin_yaw0 - cos_yaw0 - y1 + cos_yaw1 > sympy.Rational(
        6721618221725815, 70368744177664
    )

    # Equalities (unit circle constraints)
    eq1 = sin_yaw0**2 + cos_yaw0**2 - 1 == sympy.Rational(0, 1)
    eq2 = sin_yaw1**2 + cos_yaw1**2 - 1 == sympy.Rational(0, 1)

    # Method 1: Using solve() for the equality constraints first
    # Solve the unit circle constraints
    equality_solutions = solve([eq1, eq2], [sin_yaw0, cos_yaw0, sin_yaw1, cos_yaw1])
    print("Solutions to equality constraints:")
    print(equality_solutions)

    # Method 2: Using reduce_inequalities for mixed systems
    from sympy import reduce_inequalities

    # Combine all constraints
    system = And(ineq, sympy.Eq(eq1, 0), sympy.Eq(eq2, 0))

    # For polynomial systems, you can try:
    try:
        solution = reduce_inequalities(
            system, [x0, y0, sin_yaw0, cos_yaw0, y1, cos_yaw1, sin_yaw1]
        )
        print("System solution:")
        print(solution)
    except Exception as e:
        print(f"Direct solution failed: {e}")

    ego_bbox = [
        (-2.4508416652679443, 1.0641621351242065),
        (-2.4508416652679443, -1.0641621351242065),
        (2.4508416652679443, -1.0641621351242065),
        (2.4508416652679443, 1.0641621351242065),
    ]


if __name__ == "__main__":
    # plot_npc_boxes(npc_starting, npc_forward)
    # exit(0)
    # Example usage
    state_feature = ["time"]
    prediction_features = [
        "x_0",
        "y_0",
        "sin_yaw_0",
        "cos_yaw_0",
        "x_1",
        "y_1",
        "sin_yaw_1",
        "cos_yaw_1",
    ]
    base_features = state_feature + prediction_features
    # dtree_learner = DTreeLearner(base_features=base_features)
    # dtree_learner.generate_features()
    # pre = dtree_learner.get_pre_from_json("out_v1/dataset.json")
    # dtree_checker = DTreeChecker()
    # conjuncts = dtree_checker.candidate_to_conjuncts(pre)

    # plot_conjunct(list(conjuncts)[0])

    time, x0, y0, sin_yaw0, cos_yaw0, x1, y1, sin_yaw1, cos_yaw1 = symbols(
        "time x0 y0 sin_yaw0 cos_yaw0 x1 y1 sin_yaw1 cos_yaw1", real=True
    )

    # And(-6912353406506815/35184372088832 <
    # 1*time +
    # -1*x_0 +
    # 1*y_0 +
    # 1*sin_yaw_0 +
    # 1*cos_yaw_0 +
    # -1*y_1 +
    # -1*sin_yaw_1,
    # -7963433746102537/70368744177664 >=
    # 1*time +
    # 1*y_0 +
    # 1*sin_yaw_0 +
    # -1*x_1 +
    # -1*y_1 +
    # -1*sin_yaw_1 +
    # -1*cos_yaw_1,
    # -171803639642325/549755813888 >=
    # 1*time +
    # -1*x_0 +
    # -1*sin_yaw_0 +
    # -1*cos_yaw_0 +
    # -1*x_1 +
    # -1*y_1 +
    # -1*sin_yaw_1 +
    # -1*cos_yaw_1)

    # And(-6912353406506815/35184372088832 <
    # 1*time +
    # -1*x_0 +
    # 1*y_0 +
    # 1*sin_yaw_0 +
    # 1*cos_yaw_0 +
    # -1*y_1 +
    # -1*sin_yaw_1,
    # -7963433746102537/70368744177664 <
    # 1*time +
    # 1*y_0 +
    # 1*sin_yaw_0 +
    # -1*x_1 +
    # -1*y_1 +
    # -1*sin_yaw_1 +
    # -1*cos_yaw_1)

    inequality = sympy.Rational(6912353406506815/35184372088832) + (
        1 * time
        + (-1 * x0)
        + 1 * y0
        + 1 * sin_yaw0
        + 1 * cos_yaw0
        + (-1 * y1)
        + (-1 * sin_yaw1)
    )
    inequality1 = sympy.Rational(7963433746102537/70368744177664) + (
        1 * time
        + 1 * y0
        + 1 * sin_yaw0
        + (-1 * x1)
        + (-1 * y1)
        + (-1 * sin_yaw1)
        + (-1 * cos_yaw1)
    )
    # inequality2 = sympy.Rational(171803639642325/549755813888) + (
    #     1 * time
    #     + (-1 * x0)
    #     + (-1 * sin_yaw0)
    #     + (-1 * cos_yaw0)
    #     + (-1 * x1)
    #     + (-1 * y1)
    #     + (-1 * sin_yaw1)
    #     + (-1 * cos_yaw1)
    # )
    for i in np.arange(44, 52, 1.0):
        inequality = inequality.subs(time, i)
        inequality1 = inequality1.subs(time, i)
        # inequality2 = inequality2.subs(time, i)

        points = get_satisfying_points([inequality, inequality1], num_points=1000, sympy_vars=[
            x0, y0, sin_yaw0, cos_yaw0, x1, y1, sin_yaw1, cos_yaw1
        ])
        print("Number of satisfying points:", len(points))
        # print(points)
        plot_points(points)
