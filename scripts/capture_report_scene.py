#!/usr/bin/env python3

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/report_evidence/scene"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = "rov_report_scene_" + uuid.uuid4().hex[:8]
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "vendor/bluerov2_gz/models")
    os.environ["GZ_HOMEDIR"] = str(OUT / "gz_home")
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from PIL import Image as PILImage
    tree = ET.parse(ROOT / "results/current/world.sdf")
    world = tree.getroot().find("world")
    for name, position, target in [
        ("overview", (10, -10, 6), (4, 4, -5.2)),
        ("vehicle", (1.1, -1.1, -4.1), (0, 0, -5)),
    ]:
        direction = [b-a for a, b in zip(position, target)]
        yaw = math.atan2(direction[1], direction[0])
        pitch = -math.atan2(direction[2], math.hypot(*direction[:2]))
        model = ET.SubElement(world, "model", name="report_camera_"+name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, [*position, 0, pitch, yaw]))
        link = ET.SubElement(model, "link", name="camera")
        sensor = ET.SubElement(link, "sensor", name=name, type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "5"
        ET.SubElement(sensor, "topic").text = "/report/"+name
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = "1.1"
        image = ET.SubElement(camera, "image")
        ET.SubElement(image, "width").text = "2400"
        ET.SubElement(image, "height").text = "1600"
        ET.SubElement(image, "format").text = "R8G8B8"
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.03"
        ET.SubElement(clip, "far").text = "100"
    world.find("physics/real_time_factor").text = "0.05"
    path = OUT / "visualization.sdf"
    ET.indent(tree, "  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    node = Node()
    captured = {}
    def receive(name, msg):
        if name in captured:
            return
        image = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
        image.save(OUT / (name+".png"))
        captured[name] = {"sim_time": msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                          "width": msg.width, "height": msg.height}
    for name in ["overview", "vehicle"]:
        node.subscribe(Image, "/report/"+name, lambda msg, name=name: receive(name, msg))
    with (OUT/"gazebo.log").open("w") as stream:
        proc = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "3", str(path)],
                                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+50
            while len(captured) < 2 and time.monotonic() < deadline and proc.poll() is None:
                time.sleep(.1)
            if len(captured) != 2:
                raise RuntimeError("Camera capture incomplete; see scene/gazebo.log")
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
    (OUT/"capture.json").write_text(json.dumps({"source_world": "results/current/world.sdf",
        "purpose": "visualization only; cameras added, real-time factor reduced; no survey controller",
        "captures": captured}, indent=2)+"\n")
    print(json.dumps(captured))


if __name__ == "__main__":
    main()
