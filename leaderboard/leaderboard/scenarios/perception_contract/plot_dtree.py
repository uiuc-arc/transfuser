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

def get_satisfying_points_z3(conjunct, num_points=1000):
    """
    Generate random points that satisfy the decision tree conditions using Z3.
    Args:
        tree (z3.ExprRef): The decision tree expression.
        num_points (int): Number of points to generate.
    Returns:
        list: List of satisfying points.
    """
    solver = z3.Solver()
    satisfying_points = []
    
    for _ in range(num_points):
        solver.reset()
        time = z3.Real('time')
        x0 = z3.Real('x_0')
        y0 = z3.Real('y_0')
        sin_yaw0 = z3.Real('sin_yaw_0')
        cos_yaw0 = z3.Real('cos_yaw_0')
        x1 = z3.Real('x_1')
        y1 = z3.Real('y_1')
        sin_yaw1 = z3.Real('sin_yaw_1')
        cos_yaw1 = z3.Real('cos_yaw_1')
        # Add the conjuncts to the solver
        solver.add(conjunct)
        # Add constraints for the variables
        solver.add(time == 45)  # Example constraint for time
        solver.add(x0 >= 155, x0 <= 165)
        solver.add(y0 >= 35, y0 <= 45)
        solver.add(x1 >= 155, x1 <= 165)
        solver.add(y1 >= 35, y1 <= 45)
        solver.add(z3.And(sin_yaw0**2 + cos_yaw0**2 == 1, sin_yaw1**2 + cos_yaw1**2 == 1))  # Normalize yaw angles
        for points in satisfying_points:
            solver.add(
                # time != points[0],
                x0 != points[0],
                y0 != points[1],
                sin_yaw0 != points[2],
                cos_yaw0 != points[3],
                x1 != points[4],
                y1 != points[5],
                sin_yaw1 != points[6],
                cos_yaw1 != points[7]
            )
        if solver.check() == z3.sat:
            model = solver.model()
            placeholder_dict = {
                # "time": None,
                "x_0": None,
                "y_0": None,
                "sin_yaw_0": None,
                "cos_yaw_0": None,
                "x_1": None,
                "y_1": None,
                "sin_yaw_1": None,
                "cos_yaw_1": None
            }
            point = DTreeChecker.z3_model_to_dict(model, placeholder_dict)
            satisfying_points.append(
                (
                    # point["time"],
                    point["x_0"],
                    point["y_0"],
                    point["sin_yaw_0"],
                    point["cos_yaw_0"],
                    point["x_1"],
                    point["y_1"],
                    point["sin_yaw_1"],
                    point["cos_yaw_1"]
                )
            )
        else:
            print("No satisfying points found.")
    return satisfying_points



def get_satisfying_points(inequalities, num_points=1000, sympy_vars=None):
    sin1, cos1, sin2, cos2 = get_trig_values()
    satisfying_points = []

    for pair in zip(sin1, cos1, sin2, cos2):
        while len(satisfying_points) < num_points:
            x0 = np.random.uniform(140,180, size=1000)
            y0 = np.random.uniform(10, 50, size=1000)
            x1 = np.random.uniform(140, 180, size=1000)
            y1 = np.random.uniform(10, 50, size=1000)
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

                elif len(vals) == 17:
                    # print("vals:", vals)
                    if (
                        vals[0] < 0 and vals[1] < 0 and vals[2] < 0 and vals[5] < 0 and vals[6] < 0 and vals[7] < 0 and vals[8] < 0 and vals[10] < 0 and vals[12] < 0
                        and vals[3] >= 0 and vals[4] >= 0 and vals[9] >= 0 and vals[11] >= 0 and vals[13] < 0 and vals[14] >= 0 and vals[15] >= 0 and vals[16] >= 0 
                    ):
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
        # plot the rotated box with light blue color
        ax.plot(rotated_box2[:, 0], rotated_box2[:, 1], color="lightblue")
        ax.fill(rotated_box2[:, 0], rotated_box2[:, 1], color="lightblue", alpha=0.3)

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

    ax.set_xlim(150, 170)
    ax.set_ylim(30, 50) 
    ax.set_aspect("equal", adjustable="box")

    plt.grid()
    plt.show()


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

    tree = DTreeLearner(base_features=base_features)
    checker = DTreeChecker()
    tree.generate_features()
    pre = tree.get_pre_from_json("out/dataset.json")
    conjuncts = checker.candidate_to_conjuncts(pre)
    for conjunct in conjuncts:
        print("Conjunct:", conjunct)
        points = get_satisfying_points_z3(conjunct, num_points=200)
        print("Number of satisfying points:", len(points))
        print("Sample points:", points[:5])
        plot_points(points)
    exit(0)

    time, x0, y0, sin_yaw0, cos_yaw0, x1, y1, sin_yaw1, cos_yaw1 = symbols(
        "time x0 y0 sin_yaw0 cos_yaw0 x1 y1 sin_yaw1 cos_yaw1", real=True
    )

# And(4173497033683307/2251799813685248 <
#     1*sin_yaw_0 + 1*cos_yaw_0 + 1*sin_yaw_1 + 1*cos_yaw_1,
#     5598906722681881/35184372088832 <
#     1*sin_yaw_0 +
#     -1*cos_yaw_0 +
#     1*x_1 +
#     1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -5486010627960445/35184372088832 <
#     1*sin_yaw_0 +
#     -1*cos_yaw_0 +
#     -1*x_1 +
#     1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     5403855501938991/72057594037927936 >=
#     1*x_0 +
#     1*y_0 +
#     -1*sin_yaw_0 +
#     -1*cos_yaw_0 +
#     -1*x_1 +
#     -1*y_1 +
#     1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -7721038755158389/4503599627370496 >=
#     1*x_0 +
#     -1*sin_yaw_0 +
#     1*cos_yaw_0 +
#     -1*x_1 +
#     -1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -5737587654652269/4611686018427387904 <
#     1*x_0 +
#     1*sin_yaw_0 +
#     1*cos_yaw_0 +
#     -1*x_1 +
#     -1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -5117262364395813/2251799813685248 <
#     1*y_0 +
#     1*sin_yaw_0 +
#     -1*y_1 +
#     -1*sin_yaw_1 +
#     1*cos_yaw_1,
#     5142606371298841/4503599627370496 >=
#     1*x_0 +
#     1*sin_yaw_0 +
#     -1*cos_yaw_0 +
#     -1*x_1 +
#     1*cos_yaw_1,
#     -3275056560124605/72057594037927936 <
#     1*x_0 +
#     -1*sin_yaw_0 +
#     -1*x_1 +
#     1*sin_yaw_1 +
#     1*cos_yaw_1,
#     4473102601773831/4503599627370496 < 1*sin_yaw_1,
#     -4576233682160727/2251799813685248 <
#     1*x_0 +
#     1*y_0 +
#     -1*sin_yaw_0 +
#     1*cos_yaw_0 +
#     -1*x_1 +
#     -1*y_1 +
#     1*sin_yaw_1 +
#     1*cos_yaw_1,
#     8964233329028809/1152921504606846976 >=
#     1*x_0 +
#     1*y_0 +
#     -1*sin_yaw_0 +
#     1*cos_yaw_0 +
#     -1*x_1 +
#     -1*y_1 +
#     1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -1472218791852071/72057594037927936 <
#     1*sin_yaw_0 + 1*cos_yaw_0 + -1*sin_yaw_1 + -1*cos_yaw_1,
#     5573962044328313/9007199254740992 >=
#     1*y_0 + -1*y_1 + 1*sin_yaw_1,
#     -5844760337402361/2251799813685248 >=
#     1*x_0 +
#     1*y_0 +
#     -1*sin_yaw_0 +
#     -1*x_1 +
#     -1*y_1 +
#     -1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -7963433746102537/70368744177664 >=
#     1*time +
#     1*y_0 +
#     1*sin_yaw_0 +
#     -1*x_1 +
#     -1*y_1 +
#     -1*sin_yaw_1 +
#     -1*cos_yaw_1,
#     -171803639642325/549755813888 >=
#     1*time +
#     -1*x_0 +
#     -1*sin_yaw_0 +
#     -1*cos_yaw_0 +
#     -1*x_1 +
#     -1*y_1 +
#     -1*sin_yaw_1 +
#     -1*cos_yaw_1)

    dp = [43.415601875633,157.20595195636687,47.47542603319065,0.9983838127765461,-0.056830998458292974,157.10282731127654,49.4727655905957,0.9987791454934611,-0.04939856806984898]
    dp1 = [45.915601912885904,157.40512600532313,39.6000760683702,0.9993329400088262,-0.03651951551315697,157.32720039495842,41.59855739158773,0.9992010413955557,-0.0399659714511868]

    inequality = sympy.Rational(4173497033683307, 2251799813685248) - (
        1 * sin_yaw0 + 1 * cos_yaw0 + 1 * sin_yaw1 + 1 * cos_yaw1
    ) # < 0
    print(inequality)
    inequality1 = sympy.Rational(5598906722681881, 35184372088832) - (
        1 * sin_yaw0
        + -1 * cos_yaw0
        + 1 * x1
        + 1 * sin_yaw1
        + -1 * cos_yaw1
    ) # < 0
    inequality2 = sympy.Rational(-5486010627960445, 35184372088832) - (
        1 * sin_yaw0
        + -1 * cos_yaw0
        + -1 * x1
        + 1 * sin_yaw1
        + -1 * cos_yaw1
    ) # < 0
    inequality3 = sympy.Rational(5403855501938991, 72057594037927936) - (
        1 * x0
        + 1 * y0
        + -1 * sin_yaw0
        + -1 * cos_yaw0
        + -1 * x1
        + -1 * y1
        + 1 * sin_yaw1
        + -1 * cos_yaw1
    ) # >= 0
    inequality4 = sympy.Rational(-7721038755158389, 4503599627370496) - (
        1 * x0
        + -1 * sin_yaw0
        + 1 * cos_yaw0
        + -1 * x1
        + -1 * sin_yaw1
        + -1 * cos_yaw1
    ) # >= 0
    inequality5 = sympy.Rational(-5737587654652269, 4611686018427387904) - (
        1 * x0
        + 1 * sin_yaw0
        + 1 * cos_yaw0
        + -1 * x1
        + -1 * sin_yaw1
        + -1 * cos_yaw1
    ) # < 0
    inequality6 = sympy.Rational(-5117262364395813, 2251799813685248) - (
        1 * y0 + 1 * sin_yaw0 + -1 * y1 + -1 * sin_yaw1 + 1 * cos_yaw1
    ) # < 0
    inequality7 = sympy.Rational(5142606371298841, 4503599627370496) - (
        1 * x0 + 1 * sin_yaw0 + -1 * cos_yaw0 + -1 * x1 + 1 * cos_yaw1
    )   # >= 0
    inequality8 = sympy.Rational(-3275056560124605, 72057594037927936) - (
        1 * x0 + -1 * sin_yaw0 + -1 * x1 + 1 * sin_yaw1 + 1 * cos_yaw1
    ) # < 0
    inequality9 = sympy.Rational(4473102601773831, 4503599627370496) - (1 * sin_yaw1) # < 0
    inequality10 = sympy.Rational(-4576233682160727, 2251799813685248) - (
        1 * x0
        + 1 * y0
        + -1 * sin_yaw0
        + 1 * cos_yaw0
        + -1 * x1
        + -1 * y1
        + 1 * sin_yaw1
        + 1 * cos_yaw1
    ) # < 0
    inequality11 = sympy.Rational(8964233329028809, 1152921504606846976) - (
        1 * x0
        + 1 * y0
        + -1 * sin_yaw0
        + 1 * cos_yaw0
        + -1 * x1
        + -1 * y1
        + 1 * sin_yaw1
        + -1 * cos_yaw1
    ) # >= 0
    inequality12 = sympy.Rational(-1472218791852071, 72057594037927936) - (
        1 * sin_yaw0 + 1 * cos_yaw0 + -1 * sin_yaw1 + -1 * cos_yaw1
    ) # < 0
    inequality13 = sympy.Rational(5573962044328313, 9007199254740992) - (
        1 * y0 + -1 * y1 + 1 * sin_yaw1
    ) # >= 0
    inequality14 = sympy.Rational(-5844760337402361, 2251799813685248) - (
        1 * x0
        + 1 * y0
        + -1 * sin_yaw0
        + -1 * x1
        + -1 * y1
        + -1 * sin_yaw1
        + -1 * cos_yaw1
    ) # >= 0
    inequality15 = sympy.Rational(-7963433746102537, 70368744177664) - (
        1 * time
        + 1 * y0
        + 1 * sin_yaw0
        + -1 * x1
        + -1 * y1
        + -1 * sin_yaw1
        + -1 * cos_yaw1
    ) # >= 0
    inequality16 = sympy.Rational(-171803639642325, 549755813888) - (
        1 * time
        + -1 * x0
        + -1 * sin_yaw0
        + -1 * cos_yaw0
        + -1 * x1
        + -1 * y1
        + -1 * sin_yaw1
        + -1 * cos_yaw1
    ) # >= 0

    inequalities = [
        inequality, inequality1, inequality2, inequality3, inequality4, inequality5,
        inequality6, inequality7, inequality8, inequality9, inequality10, inequality11,
        inequality12, inequality13, inequality14, inequality15, inequality16
    ]

    time = symbols("time", real=True)
    x0 = symbols("x0", real=True)
    y0 = symbols("y0", real=True)
    sin_yaw0 = symbols("sin_yaw0", real=True)
    cos_yaw0 = symbols("cos_yaw0", real=True)
    x1 = symbols("x1", real=True)
    y1 = symbols("y1", real=True)
    sin_yaw1 = symbols("sin_yaw1", real=True)
    cos_yaw1 = symbols("cos_yaw1", real=True)
    dp = dp1
    new_ineqs = [
        ineq.subs({
            time: dp[0], x0: dp[1], y0: dp[2], sin_yaw0: dp[3], cos_yaw0: dp[4],
            x1: dp[5], y1: dp[6], sin_yaw1: dp[7], cos_yaw1: dp[8]
        }) for ineq in inequalities
    ]
    print("New inequalities after substitution:")
    print(new_ineqs)

    check = new_ineqs[0] < 0 and new_ineqs[1] < 0 and new_ineqs[2] < 0 and new_ineqs[3] >= 0 \
        and new_ineqs[4] >= 0 and new_ineqs[5] < 0 and new_ineqs[6] < 0 and new_ineqs[7] >= 0 \
        and new_ineqs[8] < 0 and new_ineqs[9] < 0 and new_ineqs[10] < 0 and new_ineqs[11] >= 0 \
        and new_ineqs[12] < 0 and new_ineqs[13] >= 0 and new_ineqs[14] >= 0 and new_ineqs[15] >= 0
    print("Check result:", check)
    exit(0)

    for i in np.arange(45, 52, 1.0):
        inequality = inequality.subs(time, i)
        inequality1 = inequality1.subs(time, i)
        inequality2 = inequality2.subs(time, i)
        inequality3 = inequality3.subs(time, i)
        inequality4 = inequality4.subs(time, i)
        inequality5 = inequality5.subs(time, i)
        inequality6 = inequality6.subs(time, i)
        inequality7 = inequality7.subs(time, i)
        inequality8 = inequality8.subs(time, i)
        inequality9 = inequality9.subs(time, i)
        inequality10 = inequality10.subs(time, i)
        inequality11 = inequality11.subs(time, i)
        inequality12 = inequality12.subs(time, i)
        inequality13 = inequality13.subs(time, i)
        inequality14 = inequality14.subs(time, i)
        inequality15 = inequality15.subs(time, i)
        inequality16 = inequality16.subs(time, i)

        ineqs = [
            inequality, inequality1, inequality2, inequality3, inequality4, inequality5, inequality6, 
            inequality7, inequality8, inequality9, inequality10, inequality11, inequality12, inequality13,
            inequality14, inequality15, inequality16
        ]

        points = get_satisfying_points(ineqs, num_points=100, sympy_vars=[
            x0, y0, sin_yaw0, cos_yaw0, x1, y1, sin_yaw1, cos_yaw1
        ])
        print("Number of satisfying points:", len(points))
        plot_points(points)
