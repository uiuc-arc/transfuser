import gurobipy as gp
from gurobipy import GRB
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import copy
from dtree_checker import DTreeChecker
import z3
import re

from dtree_learner import DTreeLearner
import math

class GurobiSeparatorDTreeChecker(DTreeChecker):
    def __init__(self, npc_speeds: List[float] = [2.0, 15.0]):
        # Same initialization as original
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
            [162.85592651367188, 37.39451599121094],
            [162.85638427734375, 37.02199935913086],
            [161.2130889892578, 37.392520904541016],
            [161.2135467529297, 37.02000427246094],
        ]
        self.npc_forward = [-0.9999862909317017, -0.0012179769109934568]
        self.npc_speeds = npc_speeds
        
        self.M = 10000  # Big-M constant for logical constraints
        self.PRECISION = 1e-12  # Precision for strict inequalities

        # Regex pattern to extract variable names and indices from Z3 variables
        self.Z3_VAR_RE = re.compile(r"(?P<var>[a-zA-Z_]\w*)(?:\[(?P<idx>\d+)\])?")
        
        # Keep track of current candidate constraints for removal
        self._prev_candidate_constr = []
        self._gp_model = None


    def _create_model(self) -> gp.Model:
        """Create and configure Gurobi model with all necessary variables"""
        model = gp.Model("collision_checker")
        model.setParam('OutputFlag', 0)  # Suppress output
        model.setParam('LogToConsole', 0)  # Suppress console output
        model.setParam('Presolve', 2)  # Aggressive presolve
        model.setParam('TimeLimit', 600)  # 10 minutes timeout
        # model.setParam('MIPGap', 1e-12)   # High precision
        model.setParam('NonConvex', 2)  # Enable non-convex optimization
        model.setParam('PoolSolutions', 5)  # Store up to 100 solutions
        model.setParam('PoolGap', 0.2)       # Only optimal solutions (gap = 0)
        model.setParam('PoolSearchMode', 2)  # Systematic search for solutions

        # Create all variables that might be referenced in Z3 expressions
        # These variable names should match what Z3 uses
        model.addVar(lb=self.scenario_start_time, ub=self.scenario_end_time, name="time")
        model.addVar(lb=self.npc_speeds[0], ub=self.npc_speeds[1], name="s")
        model.addVar(lb=-GRB.INFINITY, name="x")
        model.addVar(lb=-GRB.INFINITY, name="y")
        
        # Add NPC position variables
        for i in range(4):
            model.addVar(lb=-GRB.INFINITY, name=f"npc_x_{i}")
            model.addVar(lb=-GRB.INFINITY, name=f"npc_y_{i}")
        
        model.update()
        return model


    def _add_npc_position_constraints(self, model: gp.Model, t: float, s: gp.Var, 
                                    npc_vars: List[Tuple[gp.Var, gp.Var]]) -> List[gp.Constr]:
        """Add constraints for NPC positions based on time and speed"""
        
        # Binary variables for time intervals
        before_scenario = model.addVar(vtype=GRB.BINARY, name="before_scenario")
        during_scenario = model.addVar(vtype=GRB.BINARY, name="during_scenario")
        after_scenario = model.addVar(vtype=GRB.BINARY, name="after_scenario")
        
        # Exactly one time interval must be active
        model.addConstr(before_scenario + during_scenario + after_scenario == 1)
        
        # Time interval constraints
        model.addConstr((before_scenario == 1) >> (t <= self.scenario_start_time))
        model.addConstr((during_scenario == 1) >> (t >= self.scenario_start_time))
        model.addConstr((during_scenario == 1) >> (t <= self.scenario_end_time))
        model.addConstr((after_scenario == 1) >> (t >= self.scenario_end_time))
        
        # Speed constraints
        model.addConstr((before_scenario == 1) >> (s == 0))
        model.addConstr((after_scenario == 1) >> (s == 0))
        model.addConstr((during_scenario == 1) >> (s >= self.npc_speeds[0]))
        model.addConstr((during_scenario == 1) >> (s <= self.npc_speeds[1]))
        
        # Position constraints for each NPC corner
        for i, (npc_x, npc_y) in enumerate(npc_vars):
            # Before scenario: stay at starting position
            model.addConstr((before_scenario == 1) >> 
                          (npc_x == self.npc_starting[i][0]))
            model.addConstr((before_scenario == 1) >> 
                          (npc_y == self.npc_starting[i][1]))
            
            # During scenario: move based on speed and time
            time_diff = model.addVar(lb=0, name=f"time_diff_{i}")
            model.addConstr(time_diff == t - self.scenario_start_time)
            
            displacement_x = model.addVar(lb=-GRB.INFINITY, name=f"disp_x_{i}")
            displacement_y = model.addVar(lb=-GRB.INFINITY, name=f"disp_y_{i}")

            model.addConstr(displacement_x == s * time_diff * self.npc_forward[0])
            model.addConstr(displacement_y == s * time_diff * self.npc_forward[1])
            
            model.addConstr((during_scenario == 1) >> 
                          (npc_x == self.npc_starting[i][0] + displacement_x))
            model.addConstr((during_scenario == 1) >> 
                          (npc_y == self.npc_starting[i][1] + displacement_y))
            
            # After scenario: displacement is zero
            model.addConstr((after_scenario == 1) >> 
                          (displacement_x == 0))
            model.addConstr((after_scenario == 1) >> 
                          (displacement_y == 0))
        

    def _add_point_in_box_constraints(self, model: gp.Model, point_x: gp.Var, 
                                    point_y: gp.Var, box_corners: List[Tuple[gp.Var, gp.Var]], 
                                    is_inside: gp.Var) -> List[gp.Constr]:
        """Add constraints to check if point is inside box defined by corners"""

        # Extract box corners (assuming rectangular box)
        A_x, A_y = box_corners[0]  # First corner
        B_x, B_y = box_corners[1]  # Second corner  
        D_x, D_y = box_corners[3]  # Fourth corner
        
        # Vector calculations
        AM_x = model.addVar(lb=-GRB.INFINITY, name="AM_x")
        AM_y = model.addVar(lb=-GRB.INFINITY, name="AM_y")
        AB_x = model.addVar(lb=-GRB.INFINITY, name="AB_x")
        AB_y = model.addVar(lb=-GRB.INFINITY, name="AB_y")
        AD_x = model.addVar(lb=-GRB.INFINITY, name="AD_x")
        AD_y = model.addVar(lb=-GRB.INFINITY, name="AD_y")
        
        model.addConstr(AM_x == point_x - A_x)
        model.addConstr(AM_y == point_y - A_y)
        model.addConstr(AB_x == B_x - A_x)
        model.addConstr(AB_y == B_y - A_y)
        model.addConstr(AD_x == D_x - A_x)
        model.addConstr(AD_y == D_y - A_y)
        
        # Dot products
        AM_AB = model.addVar(lb=-GRB.INFINITY, name="AM_AB")
        AM_AD = model.addVar(lb=-GRB.INFINITY, name="AM_AD")
        AB_AB = model.addVar(name="AB_AB")
        AD_AD = model.addVar(name="AD_AD")
        
        # These are quadratic constraints - Gurobi can handle them
        model.addConstr(AM_AB == AM_x * AB_x + AM_y * AB_y)
        model.addConstr(AM_AD == AM_x * AD_x + AM_y * AD_y)
        model.addConstr(AB_AB == AB_x * AB_x + AB_y * AB_y)
        model.addConstr(AD_AD == AD_x * AD_x + AD_y * AD_y)
        
        # Point is inside if all conditions are met
        inside_AB = model.addVar(vtype=GRB.BINARY, name="inside_AB")
        inside_AD = model.addVar(vtype=GRB.BINARY, name="inside_AD")
        
        # Use indicator constraints for the box conditions
        model.addConstr((inside_AB == 1) >> (AM_AB >= 0))
        model.addConstr((inside_AB == 1) >> (AM_AB <= AB_AB))
        
        # Same logic for AD direction
        model.addConstr((inside_AD == 1) >> (AM_AD >= 0))
        model.addConstr((inside_AD == 1) >> (AM_AD <= AD_AD))
        
        # Point is inside if both AB and AD conditions are satisfied
        model.addConstr(is_inside == inside_AB * inside_AD)


    def _add_ego_collision_constraints(self, model: gp.Model, collision_point_x: gp.Var, 
                                     collision_point_y: gp.Var, ego_states: List[Dict[str, gp.Var]], 
                                     collision_indicators: List[gp.Var]) -> List[gp.Constr]:
        """Add constraints for ego vehicle collision detection"""
        
        for i, (ego_state, collision_indicator) in enumerate(zip(ego_states, collision_indicators)):
            ego_x = ego_state['x']
            ego_y = ego_state['y']
            sin_yaw = ego_state['sin_yaw']
            cos_yaw = ego_state['cos_yaw']

            # add constraint that sin and cos are that of the same angle
            yaw = model.addVar(lb=-math.pi, ub=math.pi, name=f"yaw_{i}")
            model.addGenConstrSin(yaw, sin_yaw)
            model.addGenConstrCos(yaw, cos_yaw)
            
            # Trigonometric constraint: sin²θ + cos²θ = 1
            # model.addConstr(sin_yaw * sin_yaw + cos_yaw * cos_yaw == 1)
            
            # Transform ego bounding box
            ego_corners = []
            for vertex in self.ego_bbox:
                # Rotated coordinates
                rot_x = model.addVar(lb=-GRB.INFINITY, name=f"rot_x_{i}")
                rot_y = model.addVar(lb=-GRB.INFINITY, name=f"rot_y_{i}")
                
                model.addConstr(rot_x == vertex[0] * cos_yaw - vertex[1] * sin_yaw)
                model.addConstr(rot_y == vertex[0] * sin_yaw + vertex[1] * cos_yaw)
                
                # Translated coordinates
                trans_x = model.addVar(lb=-GRB.INFINITY, name=f"trans_x_{i}")
                trans_y = model.addVar(lb=-GRB.INFINITY, name=f"trans_y_{i}")
                
                model.addConstr(trans_x == rot_x + ego_x)
                model.addConstr(trans_y == rot_y + ego_y)
                
                ego_corners.append((trans_x, trans_y))
            
            # Check if collision point is inside ego bounding box
            self._add_point_in_box_constraints(model, collision_point_x, collision_point_y, 
                                             ego_corners, collision_indicator)


    def check(self, candidate: z3.BoolRef) -> List[List[float]]:
        """
        Check a Z3 candidate formula by finding counterexamples for each conjunct
        This replaces the original check() method to work with Z3 expressions
        
        Args:
            candidate: Z3 boolean expression representing the decision tree
            pred_len: Number of prediction frames to consider
            
        Returns:
            List of counterexamples found
        """
        cexs = []

        # Convert candidate to conjuncts (from original code)
        conjuncts = list(self.candidate_to_conjuncts(candidate))
        if len(conjuncts) > 20:
            print("Warning: More than 25 conjuncts, this may take a while.")
        print(f"Checking {len(conjuncts)} conjuncts for safety using Gurobi.")

        for i, conjunct in enumerate(conjuncts):
            print(f"Checking conjunct {i+1}/{len(conjuncts)}: {conjunct}")
            cexs = self.find_counterexample_for_conjunct(conjunct, pred_len)
            valid_cexs = []
            if cexs:
                for cex in cexs:
                    is_valid = self.is_valid_cex(conjunct, cex, pred_len)
                    if is_valid:
                        valid_cexs.append(cex)
                    else:
                        print(f"SPURIOUS COUNTEREXAMPLE FOUND")
            else:
                print(f"No counterexample found for conjunct {i+1}")
        
        return valid_cexs


    def _build_affine_expr(self, z3_expr: z3.ExprRef):
        """Convert Z3 expression to Gurobi expression"""
        if z3.is_rational_value(z3_expr):
            # Handle rational constants
            return float(z3_expr.as_fraction())
        elif z3.is_int_value(z3_expr):
            # Handle integer constants
            return float(z3_expr.as_long())
        elif z3.is_var(z3_expr) or z3.is_const(z3_expr):
            # Handle variables
            var_str = str(z3_expr)
            result = self.Z3_VAR_RE.search(var_str)
            if result is None:
                raise RuntimeError(f"Could not parse variable: {var_str}")
            
            var_name = result.group("var")
            idx = result.group("idx")
            
            if idx is not None:
                gp_var = self._gp_model.getVarByName(f"{var_name}_{idx}")
            else:
                gp_var = self._gp_model.getVarByName(var_name)
            
            if gp_var is None:
                raise RuntimeError(f"Variable {var_name} not found in Gurobi model")
            return gp_var
        elif z3.is_add(z3_expr):
            # Handle addition
            return sum(self._build_affine_expr(arg) for arg in z3_expr.children())
        elif z3.is_mul(z3_expr):
            # Handle multiplication
            if len(z3_expr.children()) > 2:
                raise NotImplementedError("TODO: multiplication of three or more operands")
            lhs = self._build_affine_expr(z3_expr.arg(0))
            rhs = self._build_affine_expr(z3_expr.arg(1))
            return lhs * rhs
        elif z3.is_sub(z3_expr):
            # Handle subtraction
            args = z3_expr.children()
            result = self._build_affine_expr(args[0])
            for arg in args[1:]:
                result -= self._build_affine_expr(arg)
            return result
        else:
            raise RuntimeError(f"Only support affine expressions. Got: {z3_expr} with children: {z3_expr.children()}")


    def _set_candidate(self, conjunct: z3.BoolRef) -> None:
        """Convert Z3 conjunct to Gurobi constraints"""
        # Remove constraints from previous candidate first
        if self._prev_candidate_constr != [] and self._prev_candidate_constr in self._gp_model.getConstrs():
            self._gp_model.remove(self._prev_candidate_constr)
            self._prev_candidate_constr.clear()
            self._gp_model.update()

        conjunct = z3.simplify(conjunct, flat=True, arith_lhs=True)
        if z3.is_true(conjunct):
            return
        elif z3.is_and(conjunct):
            pred_list = list(conjunct.children())
        elif z3.is_eq(conjunct) or z3.is_le(conjunct) or z3.is_ge(conjunct) or z3.is_not(conjunct):
            pred_list = [conjunct]
        else:
            raise RuntimeError(f"{conjunct} should be a conjunction.")

        for orig_pred in pred_list:
            if z3.is_not(orig_pred):
                pred = orig_pred.arg(0)
                is_negated = True
            else:
                pred = orig_pred
                is_negated = False

            if not (z3.is_eq(pred) or z3.is_le(pred) or z3.is_ge(pred)):
                raise RuntimeError(f"Unsupported predicate: {pred}")
            
            lhs = self._build_affine_expr(pred.arg(0))
            rhs = self._build_affine_expr(pred.arg(1))

            if z3.is_eq(pred):
                if not is_negated:
                    cons = (lhs == rhs)
                else:  # !(lhs == rhs) is not easily representable in MILP
                    # We can approximate with |lhs - rhs| >= PRECISION
                    # This requires auxiliary variables and absolute value constraints
                    diff_pos = self._gp_model.addVar(name="diff_pos")
                    diff_neg = self._gp_model.addVar(name="diff_neg")
                    is_pos = self._gp_model.addVar(vtype=GRB.BINARY, name="is_pos")
                    
                    self._gp_model.addConstr(lhs - rhs == diff_pos - diff_neg)
                    self._gp_model.addConstr(diff_pos <= self.M * is_pos)
                    self._gp_model.addConstr(diff_neg <= self.M * (1 - is_pos))
                    cons = (diff_pos + diff_neg >= self.PRECISION)
            elif z3.is_ge(pred):
                if not is_negated:
                    cons = (lhs >= rhs)
                else:  # !(lhs >= rhs) <=> (lhs < rhs) => lhs <= rhs - ε
                    cons = (lhs <= rhs - self.PRECISION)
            elif z3.is_le(pred):
                if not is_negated:
                    cons = (lhs <= rhs)
                else:  # !(lhs <= rhs) <=> (lhs > rhs) => lhs >= rhs + ε
                    cons = (lhs >= rhs + self.PRECISION)

            gp_cons = self._gp_model.addConstr(cons, name=f"conjunct_{len(self._prev_candidate_constr)}")
            self._prev_candidate_constr.append(gp_cons)


    def find_counterexample_for_conjunct(self, conjunct: z3.BoolRef) -> Optional[List[float]]:
        """
        Find a counterexample for a specific Z3 conjunct representing a decision tree path
        
        Args:
            conjunct: Z3 boolean expression representing the decision tree's "safe" region
            pred_len: Number of prediction frames to consider
            
        Returns:
            List of values representing the counterexample, or None if no counterexample found
        """
        # Create model with all necessary variables
        self._gp_model = self._create_model(pred_len)
        model = self._gp_model
        
        # Get variables from model
        t = model.getVarByName("time")
        s = model.getVarByName("s")
        collision_x = model.getVarByName("x")
        collision_y = model.getVarByName("y")
        
        
        # Get NPC corner positions
        npc_corners = []
        for i in range(4):
            npc_x = model.getVarByName(f"npc_x_{i}")
            npc_y = model.getVarByName(f"npc_y_{i}")
            npc_corners.append((npc_x, npc_y))
        
        # Get ego vehicle states for each prediction frame
        ego_states = []
        collision_indicators = []
        npc_collision_indicators = []

        model.update()

        # Add collision detection constraints
        self._add_npc_position_constraints(model, t, s, npc_corners)
        
        # NPC collision constraints
        for i, npc_collision in enumerate(npc_collision_indicators):
            self._add_point_in_box_constraints(model, collision_x, collision_y, 
                                             npc_corners, npc_collision)
        
        # At least one frame must have a collision
        frame_collisions = []
        for i in range(pred_len):
            frame_collision = model.addVar(vtype=GRB.BINARY, name=f"frame_collision_{i}")
            model.addConstr(frame_collision == collision_indicators[i] * npc_collision_indicators[i])
            frame_collisions.append(frame_collision)
        
        model.addConstr(gp.quicksum(frame_collisions) >= 1)
        
        # Convert Z3 conjunct to Gurobi constraints (decision tree says this region is "safe")
        self._set_candidate(conjunct)
        
        # Exclude previously found counterexamples
        self._add_exclusion_constraints(model, ego_states, t, s)
        
        # Set objective (can be arbitrary since we just want feasibility)
        model.setObjective(0, GRB.MINIMIZE)

        # Test gurobi trig function
        # model.addGenConstrSin(ego_states[0]['sin_yaw'], ego_states[0]['cos_yaw'], "sin_cos_0")
        
        # Optimize
        model.optimize()
        
        if model.status == GRB.OPTIMAL:
            cexs = []
            for i in range(model.SolCount):
                model.setParam(GRB.Param.SolutionNumber, i)
                cex = [t.X]
                for j in range(pred_len):
                    ego_state = ego_states[j]
                    cex.extend([
                        ego_state['x'].X,
                        ego_state['y'].X,
                        ego_state['sin_yaw'].X,
                        ego_state['cos_yaw'].X
                    ])
                cexs.append(cex)

            self.cexes.extend(cexs)  # Store only the first part of the counterexample
            return cexs
        else:
            print(f"No counterexample found for conjunct. Status: {model.status}")
            return None


    def _add_exclusion_constraints(self, model: gp.Model, ego_states: List[Dict[str, gp.Var]], 
                                 t: gp.Var, s: gp.Var):
        """Add constraints to exclude previously found counterexamples"""
        for cex in self.cexes:
            exclusion_vars = []
            
            # Time exclusion
            time_diff = model.addVar(lb=0, name="time_diff")
            model.addConstr(time_diff >= t - cex[0])
            model.addConstr(time_diff >= cex[0] - t)
            time_different = model.addVar(vtype=GRB.BINARY, name="time_different")
            model.addConstr((time_different == 1) >> (time_diff >= 1e-3))
            exclusion_vars.append(time_different)
            
            # Ego state exclusions
            for i in range(len(ego_states)):
                for j, var_name in enumerate(['x', 'y', 'sin_yaw', 'cos_yaw']):
                    var_diff = model.addVar(lb=0, name=f"var_diff_{i}_{j}")
                    model.addConstr(var_diff >= ego_states[i][var_name] - cex[1 + i*4 + j])
                    model.addConstr(var_diff >= cex[1 + i*4 + j] - ego_states[i][var_name])
                    var_different = model.addVar(vtype=GRB.BINARY, name=f"var_different_{i}_{j}")
                    model.addConstr((var_different == 1) >> (var_diff >= 1e-3))
                    exclusion_vars.append(var_different)
            
            # At least one variable must be different from this counterexample
            model.addConstr(gp.quicksum(exclusion_vars) >= 1)


    def __del__(self):
        """Destructor to clean up Gurobi model"""
        if self._gp_model is not None:
            self._gp_model.write("gurobi_dtree_checker.lp")
            self._gp_model.dispose()
            self._gp_model = None
        self.cexes.clear()


if __name__ == "__main__":
    # Create checker
    checker = GurobiDTreeChecker()
    learner = DTreeLearner(base_features=["time", "x_0", "y_0", "sin_yaw_0", "cos_yaw_0", "x_1", "y_1", "sin_yaw_1", "cos_yaw_1"])
    learner.generate_features()

    tree = learner.get_pre_from_json("out/dataset.json")

    # Check for counterexamples
    counterexamples = checker.check(tree, pred_len=2)
    
    if counterexamples:
        print(f"Found {len(counterexamples)} counterexamples:")
        for i, cex in enumerate(counterexamples):
            print(f"  Counterexample {i+1}: {cex}")
    else:
        print("No counterexamples found - the candidate appears safe.")

    del checker
