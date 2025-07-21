import z3
from typing import List
from copy import deepcopy as copy


class DTreeChecker:
    def __init__(self, npc_speeds: List[float] = [3.0, 15.0]):
        # Taken from TransFuser codebase and checked with CARLA
        self.ego_bbox = [
            (-2.4508416652679443, 1.0641621351242065),
            (-2.4508416652679443, -1.0641621351242065),
            (2.4508416652679443, -1.0641621351242065),
            (2.4508416652679443, 1.0641621351242065),
        ]
        self.solver = z3.Solver()
        self.scenario_start_time = 43.4
        self.scenario_end_time = 73.4
        self.fps = 2
        self.npc_starting = [
            [162.85765075683594, 37.39452362060547],
            [162.8581085205078, 37.02200698852539],
            [161.21482849121094, 37.392520904541016],
            [161.2152862548828, 37.02000427246094],
        ]
        self.npc_forward = [-0.9999862909317017, -0.0012179769109934568]
        self.npc_speeds = npc_speeds

    def is_in_unsafe_region(
        self, x: z3.ArithRef, y: z3.ArithRef, s: z3.ArithRef, t: z3.ArithRef
    ) -> z3.BoolRef:
        """
        Check if the point (x, y) is inside the bounding box of the NPC at time t, assuming translation based on speed s.
        Args: (All symbolic)
            x (z3.ArithRef): x-coordinate of the point.
            y (z3.ArithRef): y-coordinate of the point.
            s (z3.ArithRef): Speed of the NPC.
            t (z3.ArithRef): Absolute time at which to check the bounding box.
        Returns:
            z3.BoolRef: A Z3 expression that is True if the point is inside the bounding box of the space
                        occupied by the NPC
        """

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
                t < self.scenario_start_time,
                z3.And(
                    s == 0,
                    npc_x_0 == self.npc_starting[0][0],
                    npc_y_0 == self.npc_starting[0][1],
                    npc_x_1 == self.npc_starting[1][0],
                    npc_y_1 == self.npc_starting[1][1],
                    npc_x_2 == self.npc_starting[2][0],
                    npc_y_2 == self.npc_starting[2][1],
                    npc_x_3 == self.npc_starting[3][0],
                    npc_y_3 == self.npc_starting[3][1],
                ),
            )
        )
        predicates.append(
            z3.Implies(
                z3.And(t >= self.scenario_start_time, t <= self.scenario_end_time),
                z3.And(s >= self.npc_speeds[0], s <= self.npc_speeds[1]),
            )
        )
        predicates.append(z3.Implies(t > self.scenario_end_time, s == 0))

        predicates.append(
            z3.Implies(
                t >= self.scenario_start_time,
                z3.And(
                    npc_x_0
                    == (
                        self.npc_starting[0][0]
                        + self.npc_forward[0] * s * ((t - self.scenario_start_time))
                    ),
                    npc_y_0
                    == (
                        self.npc_starting[0][1]
                        + self.npc_forward[1] * s * ((t - self.scenario_start_time))
                    ),
                    npc_x_1
                    == (
                        self.npc_starting[1][0]
                        + self.npc_forward[0] * s * ((t - self.scenario_start_time))
                    ),
                    npc_y_1
                    == (
                        self.npc_starting[1][1]
                        + self.npc_forward[1] * s * ((t - self.scenario_start_time))
                    ),
                    npc_x_2
                    == (
                        self.npc_starting[2][0]
                        + self.npc_forward[0] * s * ((t - self.scenario_start_time))
                    ),
                    npc_y_2
                    == (
                        self.npc_starting[2][1]
                        + self.npc_forward[1] * s * ((t - self.scenario_start_time))
                    ),
                    npc_x_3
                    == (
                        self.npc_starting[3][0]
                        + self.npc_forward[0] * s * ((t - self.scenario_start_time))
                    ),
                    npc_y_3
                    == (
                        self.npc_starting[3][1]
                        + self.npc_forward[1] * s * ((t - self.scenario_start_time))
                    ),
                ),
            )
        )

        """
        M of coordinates (x,y) is inside the rectangle iff
        (0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD) where . is the dot product,
        where A is the first vertex of the rectangle, B is the second vertex, and D is the fourth vertex.
        """
        A = (npc_x_0, npc_y_0)
        B = (npc_x_1, npc_y_1)
        D = (npc_x_3, npc_y_3)
        M = (x, y)
        AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
        AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
        AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
        AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)
        predicates.append(
            z3.And(AM_AB >= 0, AM_AB <= AB_AB, AM_AD >= 0, AM_AD <= AD_AD)
        )
        # Check if the point (x, y) is inside the bounding box of the NPC
        return z3.And(*predicates)

    def is_in_safe_region(
        self, x: z3.ArithRef, y: z3.ArithRef, ego_point: List[z3.ArithRef]
    ) -> z3.BoolRef:
        if len(ego_point) != 4:
            raise ValueError(
                "Ego point must be a list of 4 elements: [x, y, cos(yaw), sin(yaw)]"
            )
        ego_x, ego_y, cos_yaw, sin_yaw = ego_point
        predicates = []

        # get the vertices of the ego bounding box
        ego_bbox_transformed = []
        for vertex in self.ego_bbox:
            transformed_vertex = (vertex[0] + ego_x, vertex[1] + ego_y)
            ego_bbox_transformed.append(transformed_vertex)

        # Rotate the bounding box vertices by the yaw angle
        rotated_bbox = []
        for vertex in ego_bbox_transformed:
            rotated_x = vertex[0] * cos_yaw - vertex[1] * sin_yaw
            rotated_y = vertex[0] * sin_yaw + vertex[1] * cos_yaw
            rotated_bbox.append((rotated_x, rotated_y))

        # Check if the point (x, y) is inside the rotated bounding box
        A = rotated_bbox[0]
        B = rotated_bbox[1]
        D = rotated_bbox[3]  # Fourth vertex
        M = (x, y)
        AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
        AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
        AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
        AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)
        predicates.append(
            z3.And(AM_AB >= 0, AM_AB <= AB_AB, AM_AD >= 0, AM_AD <= AD_AD)
        )
        return z3.And(*predicates)

    def is_region_safe(self, conjunct: z3.BoolRef, pred_len: int = 2) -> z3.BoolRef:
        t = z3.Real("t")
        s = z3.Real("s")

        x = z3.Real("x")
        y = z3.Real("y")

        x_pred = [z3.Real(f"x_{i}") for i in range(pred_len)]
        y_pred = [z3.Real(f"y_{i}") for i in range(pred_len)]
        cos_yaw_pred = [z3.Real(f"cos_{i}") for i in range(pred_len)]
        sin_yaw_pred = [z3.Real(f"sin_{i}") for i in range(pred_len)]

        predicates = []

        ego_preds = []

        for i in range(pred_len):
            predicates.append(cos_yaw_pred[i] ** 2 + sin_yaw_pred[i] ** 2 == 1)
            ego_preds.append(
                z3.And(
                    self.is_in_safe_region(
                        x, y, [x_pred[i], y_pred[i], cos_yaw_pred[i], sin_yaw_pred[i]]
                    ),
                    self.is_in_unsafe_region(x, y, s, t + (i / self.fps)),
                )
            )

        predicates.append(z3.Or(*ego_preds))
        predicates.append(t >= 0)
        predicates.append(conjunct)

        return z3.And(*predicates)

    def get_cex(self, conjunct: z3.BoolRef, pred_len: int = 2) -> List[float]:
        """
        Get a counterexample for the given conjunct.
        Args:
            conjunct (z3.BoolRef): The conjunct to check.
            pred_len (int): Number of predictions to consider.
        Returns:
            List[float]: A list of values representing the counterexample.
        """
        self.solver.push()
        self.solver.add(self.is_region_safe(conjunct, pred_len))
        if self.solver.check() == z3.sat:
            model = self.solver.model()
            cex = {
                "time" : None,
            }
            for i in range(pred_len):
                cex[f"x_{i}"] = None
                cex[f"y_{i}"] = None
                cex[f"cos_yaw_{i}"] = None
                cex[f"sin_yaw_{i}"] = None
            model = DTreeChecker.z3_model_to_dict(model, cex)
            print("Counterexample found:", model)
            self.solver.pop()
            
            datapoint = [model["time"]]
            for i in range(pred_len):
                datapoint.append(model[f"x_{i}"])
                datapoint.append(model[f"y_{i}"])
                datapoint.append(model[f"cos_yaw_{i}"])
                datapoint.append(model[f"sin_yaw_{i}"])

            return datapoint
        else:
            self.solver.pop()
            return []

    def candidate_to_conjuncts(self, candidate: z3.BoolRef):
        init_path = [candidate]
        stack = [init_path]
        while stack:
            curr_path = stack.pop()  # remove this path
            curr_node = curr_path.pop()  # remove last node in this path
            if z3.is_false(curr_node):
                # the leaf node in this path is false. Skip
                continue
            elif z3.is_true(curr_node):
                if not curr_path:
                    yield z3.BoolVal(True)
                else:
                    yield z3.And(*curr_path)
            elif (
                z3.is_gt(curr_node)
                or z3.is_ge(curr_node)
                or z3.is_lt(curr_node)
                or z3.is_le(curr_node)
            ):
                yield z3.And(*curr_path, curr_node)
            elif z3.is_app_of(curr_node, z3.Z3_OP_ITE):
                cond, left, right = curr_node.children()
                l_path = curr_path.copy()
                l_path.extend([cond, left])

                r_path = curr_path.copy()
                assert len(cond.children()) == 2
                lhs, rhs = cond.children()
                if z3.is_le(cond):
                    not_cond = lhs > rhs
                elif z3.is_ge(cond):
                    not_cond = lhs < rhs
                else:
                    raise RuntimeError(f"Unexpected condition {cond} for ITE")
                r_path.extend([not_cond, right])

                stack.append(r_path)
                stack.append(l_path)
            else:
                raise RuntimeError(
                    f"Candidate formula {curr_node} should have been converted to DNF."
                )

    def check(self, candidate: z3.BoolRef, pred_len: int = 2) -> List[List[float]]:
        """
        Check if the candidate is safe.
        Args:
            candidate (z3.BoolRef): The candidate to check.
            pred_len (int): Number of predictions to consider.
        Returns:
            bool: True if the candidate is safe, False otherwise.
        """
        cexs = []
        conjuncts = list(self.candidate_to_conjuncts(candidate))
        for conjunct in conjuncts:
            cex = self.get_cex(conjunct, pred_len)
            if len(cex) > 0:
                cexs.append(cex)
        if len(cexs) > 0:
            print(f"Counterexamples found: {cexs}")
        else:
            print("No counterexamples found. The candidate is safe.")
        return cexs

    @staticmethod
    def z3_model_to_dict(model, placeholder_dict):
        """
        Convert a Z3 model to a dictionary with string keys and numerical values.
        """
        res = copy(placeholder_dict)
        for key in placeholder_dict:
            value = model.evaluate(z3.Real(key), model_completion=True)

            if isinstance(value, z3.ArithRef):
                if isinstance(value, z3.RatNumRef):
                    res[key] = (
                        value.numerator().as_long() / value.denominator().as_long()
                    )
                elif isinstance(value, z3.AlgebraicNumRef):
                    res[key] = z3.simplify(value).approx()
                else:
                    res[key] = z3.FPVal(value, z3.Float64())
            elif isinstance(value, z3.BoolRef):
                res[key] = value.as_long()
            else:
                res[key] = value
        return res


if __name__ == "__main__":
    checker = DTreeChecker()
    # Example usage
