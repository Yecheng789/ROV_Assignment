#!/usr/bin/env python3

import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def child(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = str(value)
    return node


def plugin(parent, name):
    filename = "".join("-" + c.lower() if c.isupper() else c for c in name).lstrip("-")
    return child(parent, "plugin", filename=f"gz-sim-{filename}-system",
                 name=f"gz::sim::systems::{name}")


def build(scenario, destination):
    source = ROOT / "vendor/bluerov2_gz/models/bluerov2_ping/model.sdf"
    model = copy.deepcopy(ET.parse(source).getroot().find("model"))
    model.find("pose").text = f"0 0 -{scenario['depth']} 0 0 0"
    for p in list(model.findall("plugin")):
        if p.get("name") == "ArduPilotPlugin":
            model.remove(p)
    hydro = next(p for p in model.findall("plugin") if "Hydrodynamics" in p.get("name"))
    child(hydro, "default_current", " ".join(map(str, scenario["current"])))
    child(hydro, "namespace", "bluerov2_ping")
    ping = model.find("link[@name='ping_link']")
    ping.find("pose").text = "0.25 0 -0.095 0 0 0"
    sonar = ping.find("sensor")
    sonar.find("pose").text = "0 0 0 0 0 0"
    sonar.find("visualize").text = "false"


    vertical = sonar.find("ray/scan/vertical")
    vertical.find("samples").text = "1"
    vertical.find("min_angle").text = "0"
    vertical.find("max_angle").text = "0"
    sonar.find("ray/range/max").text = "10"
    joint = model.find("joint[@name='ping_joint']")
    joint.set("type", "fixed")
    joint.remove(joint.find("axis"))
    odom = plugin(model, "OdometryPublisher")
    for key, value in {"odom_frame": "map", "robot_base_frame": "base_link",
                       "dimensions": 3, "odom_publish_frequency": 50,
                       "odom_topic": "/rov/odometry"}.items():
        child(odom, key, value)
    sdf = ET.Element("sdf", version="1.9")
    world = child(sdf, "world", name=scenario["world_name"])
    physics = child(world, "physics", name="default", type="ignored")
    child(physics, "max_step_size", 0.002)
    child(physics, "real_time_factor", scenario.get("real_time_factor", 2.0))
    child(world, "gravity", "0 0 -9.81")
    for name in ("Physics", "UserCommands", "SceneBroadcaster", "Imu"):
        plugin(world, name)
    sensors = plugin(world, "Sensors")
    child(sensors, "render_engine", "ogre2")
    buoy = plugin(world, "Buoyancy")
    graded = child(buoy, "graded_buoyancy")
    child(graded, "default_density", 1000)
    change = child(graded, "density_change")
    child(change, "above_depth", 0)
    child(change, "density", 1)
    child(buoy, "enable", "bluerov2_ping")
    scene = child(world, "scene")
    child(scene, "ambient", "0.5 0.6 0.7 1")
    child(scene, "background", "0.04 0.15 0.23 1")
    light = child(world, "light", name="sun", type="directional")
    child(light, "pose", "0 0 10 0 0 0")
    child(light, "diffuse", "0.8 0.8 0.8 1")
    child(light, "direction", "-0.3 -0.2 -1")
    floor = child(world, "model", name="seabed")
    child(floor, "static", "true")
    child(floor, "pose", "4 4 -8 0 0 0")
    link = child(floor, "link", name="base")
    for tag in ("collision", "visual"):
        shape = child(link, tag, name=tag)
        box = child(child(shape, "geometry"), "box")
        child(box, "size", "24 24 0.2")
        if tag == "visual":
            mat = child(shape, "material")
            child(mat, "diffuse", "0.65 0.59 0.4 1")
    for i, (x, y) in enumerate(scenario["obstacles"]):
        obstacle = child(world, "model", name=f"obstacle_{i}")
        child(obstacle, "static", "true")
        child(obstacle, "pose", f"{x} {y} -5 0 0 0")
        link = child(obstacle, "link", name="cylinder")
        for tag in ("collision", "visual"):
            shape = child(link, tag, name=tag)
            cyl = child(child(shape, "geometry"), "cylinder")
            child(cyl, "radius", scenario["obstacle_radius"])
            child(cyl, "length", scenario["obstacle_height"])
            if tag == "visual":
                mat = child(shape, "material")
                child(mat, "diffuse", "0.9 0.3 0.08 1")

    for y in range(0, 9, 2):
        line = child(world, "model", name=f"survey_line_{y}")
        child(line, "static", "true")
        child(line, "pose", f"4 {y} -7.89 0 0 0")
        visual = child(child(line, "link", name="line"), "visual", name="line")
        child(child(child(visual, "geometry"), "box"), "size", "8 0.025 0.01")
        child(child(visual, "material"), "diffuse", "0.15 0.8 0.9 1")
    world.append(model)
    ET.indent(sdf, "  ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(sdf).write(destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=Path, default=ROOT / "config/scenario.json")
    parser.add_argument("--output", type=Path, default=ROOT / "worlds/survey.sdf")
    args = parser.parse_args()
    build(json.loads(args.scenario.read_text()), args.output)
    print(args.output)
