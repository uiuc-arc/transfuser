import os
import sys
import numpy as np
from synthesize_dtree import CONFIG_DICT_KEYS
import z3
import gurobipy as gp
from gurobipy import GRB


def prepare_features(base_features, labels):
    feature_map = {}
    truth_map = {}
    for i, feature in enumerate(base_features):
        feature_map[i] = z3.Real(feature)

    if all(labels):
        truth_map[0.0] = True
        labels = [0.0 if label else 0.0 for label in labels]
    else:
        truth_map[1.0] = True
        truth_map[0.0] = False

    return labels, feature_map, truth_map


def parse_tree_json(tree, feature_map, truth_map):
    """
    Parse the decision tree JSON structure into a Z3 expression.
    Args:
        tree (dict): The decision tree in JSON format.
    Returns:
        z3.BoolRef: The Z3 expression representing the decision tree.
    """
    if tree["is_leaf"]:
        return z3.BoolVal(truth_map[tree["value"]])

    left = parse_tree_json(tree["left"], feature_map, truth_map)
    right = parse_tree_json(tree["right"], feature_map, truth_map)

    z3_expr = None
    if tree["is_oblique"]:
        z3_expr = z3.Sum(
            *(w * feature_map[i] for w, i in zip(tree["weights"], tree["features"]))
        )
    else:
        z3_expr = feature_map[tree["feature_idx"]]

    z3_threshold = z3.simplify(z3.fpToReal(z3.FPVal(tree["threshold"], z3.Float64())))

    if z3.is_true(left):
        if z3.is_true(right):
            return z3.BoolVal(True)
        elif z3.is_false(right):
            return z3_expr <= z3_threshold
    if z3.is_false(left):
        if z3.is_true(right):
            return z3_expr > z3_threshold
        elif z3.is_false(right):
            return z3.BoolVal(False)

    return z3.If((z3_expr <= z3_threshold), left, right)


def candidate_to_conjuncts(candidate: z3.BoolRef):
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


def z3_to_gurobi(conjunct: z3.BoolRef, model: gp.Model, var_map: dict):
    """
    Convert a Z3 conjunctive formula to Gurobi constraints.

    Args:
        conjunct (z3.BoolRef): A Z3 formula in conjunctive form.
        model (gp.Model): Gurobi model to which constraints will be added.
        var_map (dict): Mapping from Z3 variables to Gurobi variables.
    """

    def extract_linear_terms(expr: z3.ArithRef):
        """
        Extract linear terms from a Z3 arithmetic expression.

        Args:
            expr (z3.ArithRef): A Z3 arithmetic expression.
        Returns:
            dict: A mapping from Z3 variables to their coefficients.
        """
        terms = {}
        if z3.is_add(expr):
            for child in expr.children():
                child_terms = extract_linear_terms(child)
                for var, coef in child_terms.items():
                    if var in terms:
                        terms[var] += coef
                    else:
                        terms[var] = coef
        elif z3.is_mul(expr):
            coeff = 1.0
            var = None
            for child in expr.children():
                if z3.is_int(child) or z3.is_real(child):
                    coeff *= float(child.as_decimal(6))
                else:
                    var = child
            if var is not None:
                terms[var] = coeff
        elif z3.is_int(expr) or z3.is_real(expr):
            terms[None] = float(expr.as_decimal(6))
        else:
            terms[expr] = 1.0
        return terms

    if z3.is_true(conjunct):
        return  # No constraints to add for true

    if z3.is_and(conjunct):
        clauses = conjunct.children()
    else:
        clauses = [conjunct]

    for clause in clauses:
        if z3.is_le(clause):
            lhs, rhs = clause.children()
            model.addConstr(
                gp.quicksum(
                    var_map[int(str(var))] * float(coef)
                    for var, coef in extract_linear_terms(lhs).items()
                )
                <= float(rhs.as_decimal(6)),
                name=f"z3_le_{str(clause)}",
            )
        elif z3.is_ge(clause):
            lhs, rhs = clause.children()
            model.addConstr(
                gp.quicksum(
                    var_map[int(str(var))] * float(coef)
                    for var, coef in extract_linear_terms(lhs).items()
                )
                >= float(rhs.as_decimal(6)),
                name=f"z3_ge_{str(clause)}",
            )
        else:
            raise RuntimeError(f"Unsupported clause type: {clause}")


def sample_config_from_space(fixed_config=None, config_space=None):
    """
    Samples a configuration from a predefined configuration space.

    Returns:
        dict: A configuration dictionary with waypoints and weather parameters.
    """
    npc_speed = None
    if fixed_config is not None and "npc_speed" in fixed_config:
        npc_speed = fixed_config["npc_speed"]
    elif config_space is not None and "npc_speed" in config_space:
        npc_speed = round(
            np.random.uniform(
                config_space["npc_speed"][0], config_space["npc_speed"][1]
            ),
            6,
        )
    else:
        npc_speed = round(np.random.uniform(0.5, 10), 6)
    npc_min_starting_distance = None
    if fixed_config is not None and "npc_min_starting_distance" in fixed_config:
        npc_min_starting_distance = fixed_config["npc_min_starting_distance"]
    elif config_space is not None and "npc_min_starting_distance" in config_space:
        npc_min_starting_distance = round(
            np.random.randint(
                config_space["npc_min_starting_distance"][0],
                config_space["npc_min_starting_distance"][1],
            ),
            6,
        )
    else:
        npc_min_starting_distance = round(np.random.randint(12, 20), 6)
    config = {
        "waypoints": [
            {
                "x": 120.212006,
                "y": 59.523838,
                "z": 0.033585,
                "pitch": -0.019200,
                "roll": 0.000290,
                "yaw": 0.301839,
            },
            {
                "x": 158.060257,
                "y": 16.202417,
                "z": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "yaw": 270.069580,
            },
        ],
        "weather": {
            "cloudiness": np.random.randint(0, 101),
            "precipitation": np.random.randint(0, 101),
            "precipitation_deposits": np.random.randint(0, 101),
            "wind_intensity": np.random.randint(0, 101),
            "sun_azimuth_angle": np.random.randint(0, 361),
            "sun_altitude_angle": np.random.randint(-90, 91),
        },
        "other_config": {
            "npc_speed": npc_speed,
            "npc_min_starting_distance": npc_min_starting_distance,
        },
    }
    return config


def dump_config_to_xml(config, file_path):
    """
    Dumps a configuration dictionary to an XML file.

    Args:
        config (dict): Configuration dictionary.
        file_path (str): Path to the output XML file.
    """
    xml_string = "<?xml version='1.0' encoding='UTF-8'?>\n"
    routes_begin = "<routes>\n"
    routes_end = "</routes>\n"
    route_begin = '  <route id="{id}" town="{town}">\n'
    route_end = "  </route>\n"
    waypoint_template = '    <waypoint x="{x}" y="{y}" z="{z}" pitch="{pitch}" roll="{roll}" yaw="{yaw}"/>\n'
    weather_template = '  <weather cloudiness="{cloudiness}" precipitation="{precipitation}" precipitation_deposits="{precipitation_deposits}" wind_intensity="{wind_intensity}" sun_azimuth_angle="{sun_azimuth_angle}" sun_altitude_angle="{sun_altitude_angle}" />\n'

    other_config_template = '  <other_config npc_speed="{npc_speed}" npc_min_starting_distance="{npc_min_starting_distance}" />\n'

    xml_string += routes_begin

    xml_string += route_begin.format(
        id=config.get("id", 0), town=config.get("town", "Town01")
    )
    for waypoint in config["waypoints"]:
        xml_string += waypoint_template.format(
            x=waypoint["x"],
            y=waypoint["y"],
            z=waypoint["z"],
            pitch=waypoint["pitch"],
            roll=waypoint["roll"],
            yaw=waypoint["yaw"],
        )
    if "weather" in config:
        weather = config["weather"]
        xml_string += weather_template.format(
            cloudiness=weather["cloudiness"],
            precipitation=weather["precipitation"],
            precipitation_deposits=weather["precipitation_deposits"],
            wind_intensity=weather["wind_intensity"],
            sun_azimuth_angle=weather["sun_azimuth_angle"],
            sun_altitude_angle=weather["sun_altitude_angle"],
        )
    if "other_config" in config:
        other_config = config["other_config"]
        xml_string += other_config_template.format(
            npc_speed=other_config["npc_speed"],
            npc_min_starting_distance=other_config["npc_min_starting_distance"],
        )
    xml_string += route_end
    xml_string += routes_end

    with open(file_path, "w") as file:
        file.write(xml_string)


if __name__ == "__main__":
    # Example configuration
    config = sample_config_from_space(
        {"npc_min_starting_distance": 19, "npc_speed": 0.45}
    )
    output_file_path = "leaderboard/data/training/routes/Scenario4/Town01_Scenario4.xml"
    dump_config_to_xml(config, output_file_path)
    print(f"Configuration dumped to {output_file_path}")
