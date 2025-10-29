import numpy as np


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
