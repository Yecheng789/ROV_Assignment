#!/usr/bin/env python3

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
from prepare_world import build, ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT/"results/main")
    parser.add_argument("--scenario", type=Path, default=ROOT/"config/scenario.json")
    parser.add_argument("--mission", type=Path, default=ROOT/"config/mission.json")
    parser.add_argument("--wall-timeout", type=float, default=1200)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    if (output/"telemetry.csv").exists():
        parser.error(f"{output} already contains a run. Choose a new --output directory.")
    output.mkdir(parents=True, exist_ok=True)
    scenario = json.loads(args.scenario.read_text())
    build(scenario, output/"world.sdf")
    shutil.copyfile(args.scenario, output/"scenario.json")
    shutil.copyfile(args.mission, output/"mission.json")
    env = os.environ.copy()
    env.update(GZ_SIM_RESOURCE_PATH=str(ROOT/"vendor/bluerov2_gz/models"),
               GZ_PARTITION="rov_assignment_"+uuid.uuid4().hex[:10],
               GZ_HOMEDIR=str(output/"gz_home"), ROS_LOG_DIR=str(output/"ros_logs"),
               ROS_DOMAIN_ID="83", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               PYTHONPATH=str(ROOT)+os.pathsep+env.get("PYTHONPATH", ""),
               PYTHONUNBUFFERED="1")

    env.pop("ROS_STATIC_PEERS", None)
    manifest = {"command": sys.argv, "gazebo": subprocess.check_output(
        ["gz", "sim", "--versions"], text=True).strip(), "ros_distro": env.get("ROS_DISTRO"),
        "upstream_commit": subprocess.check_output(
            ["git", "-C", str(ROOT/"vendor/bluerov2_gz"), "rev-parse", "HEAD"], text=True).strip(),
        "gz_partition": env["GZ_PARTITION"], "ros_domain_id": env["ROS_DOMAIN_ID"],
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for folder in ("rov_survey", "scripts", "config")
                          for p in (ROOT/folder).rglob("*") if p.is_file() and "__pycache__" not in str(p)}}
    (output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    processes, streams = [], []
    def start(name, command):
        stream = (output/f"{name}.log").open("w")
        streams.append(stream)
        proc = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc
    try:
        gazebo = start("gazebo", ["gz", "sim", "-s", "--headless-rendering", "-v", "3",
                                  "--seed", str(scenario["seed"]), str(output/"world.sdf")])
        bridge = start("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                  "--ros-args", "-p", f"config_file:={ROOT/'config/bridge.yaml'}"])
        node = start("controller", [sys.executable, "-m", "rov_survey.node",
                                    "--config", str(output/"mission.json"), "--output", str(output)])
        if args.gui:
            start("gui", ["gz", "sim", "-g"])

        ready = False
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            if gazebo.poll() is not None:
                raise RuntimeError("Gazebo exited during startup; inspect gazebo.log.")
            unpause = subprocess.run(
                ["gz", "service", "-s", "/world/survey/control", "--reqtype",
                 "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "1000", "--req", "pause: false"],
                env=env, capture_output=True, text=True)
            if "data: true" in unpause.stdout:
                ready = True
                break
            time.sleep(0.5)
        if not ready:
            raise RuntimeError("Could not unpause Gazebo.")
        print(f"Run started: {output}", flush=True)
        deadline = time.monotonic()+args.wall_timeout
        last_status_update = 0.
        while node.poll() is None:
            if gazebo.poll() is not None or bridge.poll() is not None:
                raise RuntimeError("Gazebo or bridge exited; inspect logs.")
            if time.monotonic() > deadline:
                raise TimeoutError("Wall-time deadline exceeded.")
            if time.monotonic()-last_status_update > 20 and (output/"status.json").exists():
                last_status_update = time.monotonic()
                try:
                    s = json.loads((output/"status.json").read_text())
                    print(f"t={s['simulation_time']:.1f}s {s['state']} "
                          f"completed={len(s['completed_legs'])}/9 detected={len(s['detections'])}", flush=True)
                except json.JSONDecodeError:
                    pass
            time.sleep(0.2)
        return node.returncode
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for stream in streams:
            stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
