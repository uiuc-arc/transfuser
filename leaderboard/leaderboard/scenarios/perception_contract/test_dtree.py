import z3


bbox = [
    (-2.4508416652679443, 1.0641621351242065),
    (-2.4508416652679443, -1.0641621351242065),
    (2.4508416652679443, -1.0641621351242065),
    (2.4508416652679443, 1.0641621351242065),
]

npc_starting = [
    [292.85765075683594, 57.392520904541016],
    [292.85765075683594, 37.02000427246094],
    [31.21482849121094, 37.02000427246094],
    [31.21482849121094, 57.392520904541016],
]
npc_forward = [-0.9999862909317017, -0.0012179769109934568]
scenario_start_time = 43.4
scenario_end_time = 53.4
npc_speeds = [2.0, 15.0]  # Minimum and maximum speeds for the NPCs


x_0 = z3.Real("x0")
y_0 = z3.Real("y0")
x_1 = z3.Real("x1")
y_1 = z3.Real("y1")
sin_yaw_0 = z3.Real("sin_yaw_0")
cos_yaw_0 = z3.Real("cos_yaw_0")
sin_yaw_1 = z3.Real("sin_yaw_1")
cos_yaw_1 = z3.Real("cos_yaw_1")

x = z3.Real("x")
y = z3.Real("y")
s = z3.Real("s")  # Speed of the NPC
t = z3.Real("t")  # Time variable


expr = (
    (1 * x_0)
    + (1 * sin_yaw_0)
    + (-1 * cos_yaw_0)
    + (1 * x_1)
    + (-1 * sin_yaw_1)
    + (1 * cos_yaw_1)
)
expression = (expr - 173.5703) > 0.0

solver = z3.Solver()
solver.add(expression)


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


def point_is_in_box(xz, yz, point):
    """
    Check if a point is inside a bounding box defined by the vertices.
    """
    x, y, cos_yaw, sin_yaw = point
    box = []
    for vertex in bbox:
        translated_vertex = (vertex[0] + x, vertex[1] + y)
        box.append(translated_vertex)

    # Rotate the box vertices
    rotation_matrix = [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]]
    rotated_box = []
    for vertex in box:
        rotated_x = (
            vertex[0] * rotation_matrix[0][0] + vertex[1] * rotation_matrix[0][1]
        )
        rotated_y = (
            vertex[0] * rotation_matrix[1][0] + vertex[1] * rotation_matrix[1][1]
        )
        rotated_box.append((rotated_x, rotated_y))

    return is_in_box((xz, yz), rotated_box)


def check_point_in_npc(point):
    x, y, s, t = point
    predicates = []

    npc_x_0 = z3.Real("npc_x_0")
    npc_y_0 = z3.Real("npc_y_0")
    npc_x_1 = z3.Real("npc_x_1")
    npc_y_1 = z3.Real("npc_y_1")
    npc_x_2 = z3.Real("npc_x_2")
    npc_y_2 = z3.Real("npc_y_2")
    npc_x_3 = z3.Real("npc_x_3")
    npc_y_3 = z3.Real("npc_y_3")

    predicates.append(
        z3.Implies(
            t < scenario_start_time,
            z3.And(
                s == 0,
                npc_x_0 == npc_starting[0][0],
                npc_y_0 == npc_starting[0][1],
                npc_x_1 == npc_starting[1][0],
                npc_y_1 == npc_starting[1][1],
                npc_x_2 == npc_starting[2][0],
                npc_y_2 == npc_starting[2][1],
                npc_x_3 == npc_starting[3][0],
                npc_y_3 == npc_starting[3][1],
            ),
        )
    )
    predicates.append(
        z3.Implies(
            z3.And(t >= scenario_start_time, t <= scenario_end_time),
            z3.And(s >= npc_speeds[0], s <= npc_speeds[1]),
        )
    )
    predicates.append(z3.Implies(t > scenario_end_time, s == 0))

    predicates.append(
        z3.Implies(
            t >= scenario_start_time,
            z3.And(
                npc_x_0
                == (
                    npc_starting[0][0]
                    + npc_forward[0] * s * ((t - scenario_start_time))
                ),
                npc_y_0
                == (
                    npc_starting[0][1]
                    + npc_forward[1] * s * ((t - scenario_start_time))
                ),
                npc_x_1
                == (
                    npc_starting[1][0]
                    + npc_forward[0] * s * ((t - scenario_start_time))
                ),
                npc_y_1
                == (
                    npc_starting[1][1]
                    + npc_forward[1] * s * ((t - scenario_start_time))
                ),
                npc_x_2
                == (
                    npc_starting[2][0]
                    + npc_forward[0] * s * ((t - scenario_start_time))
                ),
                npc_y_2
                == (
                    npc_starting[2][1]
                    + npc_forward[1] * s * ((t - scenario_start_time))
                ),
                npc_x_3
                == (
                    npc_starting[3][0]
                    + npc_forward[0] * s * ((t - scenario_start_time))
                ),
                npc_y_3
                == (
                    npc_starting[3][1]
                    + npc_forward[1] * s * ((t - scenario_start_time))
                ),
            ),
        )
    )

    A = (npc_x_0, npc_y_0)
    B = (npc_x_1, npc_y_1)
    D = (npc_x_3, npc_y_3)
    M = (x, y)
    AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
    AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
    AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
    AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)
    predicates.append(z3.And(AM_AB >= 0, AM_AB <= AB_AB, AM_AD >= 0, AM_AD <= AD_AD))
    # Check if the point (x, y) is inside the bounding box of the NPC
    return z3.And(*predicates)


step1 = z3.And(
    point_is_in_box(x, y, (x_0, y_0, sin_yaw_0, cos_yaw_0)),
    check_point_in_npc((x, y, s, t)),
)
step2 = z3.And(
    point_is_in_box(x, y, (x_1, y_1, sin_yaw_1, cos_yaw_1)),
    check_point_in_npc((x, y, s, t + 0.5)),
)

f = z3.Or(step1, step2)

solver.add(f)
solver.add(cos_yaw_0 ** 2 + sin_yaw_0 ** 2 == 1)
solver.add(cos_yaw_1 ** 2 + sin_yaw_1 ** 2 == 1)
res = solver.check()
if res == z3.sat:
    model = solver.model()
    print("SAT")
    for d in model.decls():
        print(f"{d.name()} = {model[d]}")
else:
    print("UNSAT")
    unsat_core = solver.unsat_core()
    print("Unsat core:", unsat_core)
