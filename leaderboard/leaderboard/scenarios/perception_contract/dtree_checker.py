import z3
from typing import List
from copy import deepcopy as copy
import subprocess
import json
import tempfile
from pysmt.smtlib.parser import SmtLibParser
from pysmt.fnode import FNode

SMT2_CHECK_CODE = "\n(set-option :pp.decimal true)\n(set-option :pp.decimal_precision 10)\n(check-sat)\n(get-model)\n"

class DTreeChecker:
    def __init__(self, npc_speeds: List[float] = [2.0, 15.0]):
        # Taken from TransFuser codebase and checked with CARLA
        self.ego_bbox = [
            (-2.4508416652679443, 1.0641621351242065),
            (-2.4508416652679443, -1.0641621351242065),
            (2.4508416652679443, -1.0641621351242065),
            (2.4508416652679443, 1.0641621351242065),
        ]
        self.scenario_start_time = 43.4
        self.scenario_end_time = 53.4
        self.fps = 2
        self.npc_starting = [
            # [292.85765075683594, 57.39452362060547],
            # [292.8581085205078, 37.02200698852539],
            # [31.2152862548828, 37.02000427246094],
            # [31.21482849121094, 57.392520904541016],
            [162.85592651367188, 37.39451599121094],
            [162.85638427734375, 37.02199935913086],
            [161.2130889892578, 37.392520904541016],
            [161.2135467529297, 37.02000427246094],
        ]
        self.npc_forward = [-0.9999862909317017, -0.0012179769109934568]
        self.npc_speeds = npc_speeds
        self.cexes = []

    @staticmethod
    def _to_real(v: float) -> z3.RatNumRef:
        return z3.simplify(
            z3.fpToReal(z3.FPVal(v, z3.Float64()))
        )

    @staticmethod
    def _remove_qn_marks(s: str) -> str:
        """
        Remove question marks from the string
        """
        return s.replace("?", "")

    def _get_solver(self):
        solver = z3.Solver()
        solver.set("timeout", 1000 * 60 * 10)  # 10 minutes timeout
        solver.set("model", True)
        solver.set("unsat_core", True)
        return solver

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

    def is_in_npc(
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

        # """
        #     M of coordinates (x,y) is inside the rectangle iff
        #     (0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD) where . is the dot product,
        #     where A is the first vertex of the rectangle, B is the second vertex, and D is the fourth vertex.
        # """
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

    def is_in_ego_actor(
        self, x: z3.ArithRef, y: z3.ArithRef, ego_point: List[z3.ArithRef]
    ) -> z3.BoolRef:
        if len(ego_point) != 4:
            raise ValueError(
                "Ego point must be a list of 4 elements: [x, y, cos(yaw), sin(yaw)]"
            )
        ego_x, ego_y, sin_yaw, cos_yaw = ego_point
        predicates = []

        # get the vertices of the ego bounding box
        ego_bbox_transformed = []
        # Rotate the bounding box vertices by the yaw angle
        rotated_bbox = []
        for vertex in self.ego_bbox:
            rotated_x = vertex[0] * cos_yaw - vertex[1] * sin_yaw
            rotated_y = vertex[0] * sin_yaw + vertex[1] * cos_yaw
            rotated_bbox.append((rotated_x, rotated_y))

        for vertex in rotated_bbox:
            transformed_vertex = (vertex[0] + ego_x, vertex[1] + ego_y)
            ego_bbox_transformed.append(transformed_vertex)

        # Check if the point (x, y) is inside the rotated bounding box
        A = ego_bbox_transformed[0]
        B = ego_bbox_transformed[1]
        D = ego_bbox_transformed[3]  # Fourth vertex
        M = (x, y)
        AM_AB = ((M[0] - A[0]) * (B[0] - A[0])) + ((M[1] - A[1]) * (B[1] - A[1]))
        AM_AD = ((M[0] - A[0]) * (D[0] - A[0])) + ((M[1] - A[1]) * (D[1] - A[1]))
        AB_AB = ((B[0] - A[0]) ** 2) + ((B[1] - A[1]) ** 2)
        AD_AD = ((D[0] - A[0]) ** 2) + ((D[1] - A[1]) ** 2)
        predicates.append(
            z3.And(AM_AB >= 0, AM_AB <= AB_AB, AM_AD >= 0, AM_AD <= AD_AD)
        )
        return z3.And(*predicates)

    def is_collision_present(
        self, conjunct: z3.BoolRef, num_frames: int = 2
    ) -> z3.BoolRef:
        t = z3.Real("time")
        s = z3.Real("s")

        x = z3.Real("x")
        y = z3.Real("y")

        x_pred = [z3.Real(f"x_{i}") for i in range(num_frames)]
        y_pred = [z3.Real(f"y_{i}") for i in range(num_frames)]
        sin_yaw_pred = [z3.Real(f"sin_yaw_{i}") for i in range(num_frames)]
        cos_yaw_pred = [z3.Real(f"cos_yaw_{i}") for i in range(num_frames)]

        predicates = []

        ego_preds = []

        for i in range(num_frames):
            predicates.append(cos_yaw_pred[i] ** 2 + sin_yaw_pred[i] ** 2 == 1)
            ego_preds.append(
                z3.And(
                    self.is_in_ego_actor(
                        x, y, [x_pred[i], y_pred[i], sin_yaw_pred[i], cos_yaw_pred[i]]
                    ),
                    self.is_in_npc(x, y, s, t + ((i+1) / self.fps)),
                )
            )

        predicates.append(z3.Or(*ego_preds))
        predicates.append(t >= self.scenario_start_time)
        predicates.append(t <= self.scenario_end_time)
        predicates.append(conjunct)

        return z3.And(*predicates)

    def _to_float(self, value: FNode) -> float:
        """
        Convert a FNode to a float value.
        Args:
            value (FNode): The FNode to convert.
        Returns:
            float: The float value of the FNode.
        """
        value = str(value)
        if "/" in value:
            numerator, denominator = value.split("/")
            return float(numerator) / float(denominator)
        else:
            return float(value)

    def _is_not_prev_seen_cex(self) -> z3.BoolRef:
        """
        Add constraints to the solver to avoid previously seen counterexamples.
        Returns:
            z3.BoolRef: A Z3 expression that is True if the current counterexample is not in the seen set.
        """
        if len(self.cexes) == 0:
            return None
        constraints = []
        for cex in self.cexes:
            cex_constraints = []
            time = cex[0]
            cex_constraints.append(z3.Real("time") != time)
            for i in range((len(cex) - 1) // 4):
                cex_constraints.append(z3.Real(f"x_{i}") != cex[1 + i * 4])
                cex_constraints.append(z3.Real(f"y_{i}") != cex[2 + i * 4])
                cex_constraints.append(z3.Real(f"sin_yaw_{i}") != cex[3 + i * 4])
                cex_constraints.append(z3.Real(f"cos_yaw_{i}") != cex[4 + i * 4])
            constraints.append(z3.And(*cex_constraints))
        return z3.Not(z3.Or(*constraints))

    def get_cex(
        self, solver: z3.Solver, conjunct: z3.BoolRef, pred_len: int = 2
    ) -> List[float]:
        """
        Get a counterexample for the given conjunct.
        Args:
            conjunct (z3.BoolRef): The conjunct to check.
            pred_len (int): Number of predictions to consider.
        Returns:
            List[float]: A list of values representing the counterexample.
        """
        solver.push()
        # solver.add(self.is_region_safe(conjunct, pred_len))
        solver.add(self.is_collision_present(conjunct, pred_len))

        cex_new = self._is_not_prev_seen_cex()
        if cex_new is not None:
            solver.add(cex_new)

        current_formula = solver.sexpr()
        current_formula += SMT2_CHECK_CODE
        model = None
        with tempfile.NamedTemporaryFile(mode='w+t', suffix=".smt2") as f:
            f.write(current_formula)
            f.flush()
            f.seek(0)
            cmd = f"z3 -smt2 unsat_core=true -T:600 {f.name}"
            print(f"Running command: {cmd}")
            proc = subprocess.Popen(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            stdout, stderr = proc.communicate()
            if proc.returncode != 0:
                print("Error running Z3 command:", stderr.decode())
                return []

            parser = SmtLibParser()
            output_str = stdout.decode().strip().splitlines()
            status = output_str[0].strip()
            if status == "sat":
                model_string = output_str[1:]
                model_string = "\n".join(model_string)
                model_string = DTreeChecker._remove_qn_marks(model_string)
                model_string = model_string.strip(" ").strip("\n")
                print("Model string:", model_string)
                with tempfile.NamedTemporaryFile(mode='w+t', suffix=".smt2") as f1:
                    if model_string:
                        f1.write(model_string)
                        f1.flush()
                        f1.seek(0)

                        model = parser.parse_model(f1)
                        print("Model:", model[0])
                        new_model = {}
                        for k, v in model[0].items():
                            new_model[str(k)] = self._to_float(v)

                        datapoint = [new_model["time"]]
                        for i in range(pred_len):
                            datapoint.append(new_model[f"x_{i}"])
                            datapoint.append(new_model[f"y_{i}"])
                            datapoint.append(new_model[f"sin_yaw_{i}"])
                            datapoint.append(new_model[f"cos_yaw_{i}"])

                        return datapoint
                
            return []

    def is_cex_pred(self, cex, pred_len: int = 2) -> z3.BoolRef:
        """
        Check if the given counterexample is valid.
        Args:
            cex: The counterexample to check.
            pred_len (int): Number of predictions to consider.
        Returns:
            z3.BoolRef: A Z3 expression that is True if the counterexample is valid.
        """
        if len(cex) != 1 + pred_len * 4:
            raise ValueError(
                f"Counterexample must have {1 + pred_len * 4} elements, got {len(cex)}"
            )

        cex_map = []
        cex = list(map(DTreeChecker._to_real, cex))
        cex_map.append((z3.Real("time"), cex[0]))
        for i in range(pred_len):
            cex_map.append((z3.Real(f"x_{i}"), cex[1 + i * 4]))
            cex_map.append((z3.Real(f"y_{i}"), cex[2 + i * 4]))
            cex_map.append((z3.Real(f"sin_yaw_{i}"), cex[3 + i * 4]))
            cex_map.append((z3.Real(f"cos_yaw_{i}"), cex[4 + i * 4]))
        return cex_map

    def is_valid_cex(self, conjunct, cex, pred_len:int = 2) -> bool:
        """
        Check if the given counterexample is valid.
        Args:
            cex (List[float]): The counterexample to check.
        Returns:
            bool: True if the counterexample is valid, False otherwise.
        """
        cex_map = self.is_cex_pred(cex, pred_len)

        val = z3.simplify(
            z3.substitute(conjunct, *cex_map)
        )
        if not z3.is_bool(val):
            raise ValueError("Conjunct must be a boolean expression.")
        if z3.is_true(val):
            return True
        else:
            return False

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
        solver = self._get_solver()
        conjuncts = list(self.candidate_to_conjuncts(candidate))
        print(f"Checking {len(conjuncts)} conjuncts for safety.")
        for conjunct in conjuncts:
            print(f"Checking conjunct: {conjunct}")
            cex = self.get_cex(solver, conjunct, pred_len)
            if len(cex) > 0:
                cexs.append(cex)
                self.cexes.append(cex)
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
                    value = z3.simplify(value).approx()
                    res[key] = (
                        value.numerator().as_long() / value.denominator().as_long()
                    )
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
    pred = checker.is_collision_present(z3.BoolVal(True), num_frames=2)
    print("Predicates for collision check:", pred)
