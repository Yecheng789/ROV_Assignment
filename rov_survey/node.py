import argparse
from collections import deque
import csv
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from .perception import SonarMapper
from .planning import Planner, mission_legs, path_projection, point_at
from .control import RobustController, wrap

ROOT = Path(__file__).resolve().parents[1]


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class SurveyNode(Node):
    def __init__(self, config, output):
        super().__init__("rov_survey")
        self.config, self.output = config, output
        output.mkdir(parents=True, exist_ok=True)
        self.mapper, self.planner = SonarMapper(config), Planner(config)
        self.controller = RobustController(ROOT / "vendor/bluerov2_gz/models/bluerov2_ping/model.sdf")
        self.legs = mission_legs(config)
        self.leg_index = 0
        self.completed = []
        self.history = deque(maxlen=200)
        self.path = None
        self.path_version = 0
        self.last_plan = -100.
        self.last_scan = -100.
        self.first_time = None
        self.last_control = None
        self.last_wall_odom = time.monotonic()
        self.wall_start = time.monotonic()
        self.last_print = -100.
        self.last_status = -100.
        self.state = "WAITING"
        self.finished = False
        self.reason = ""
        self.samples = 0
        self.scan_count = 0
        self.replans = 0
        self.last_yaw_target = 0.
        self.thrust_publishers = [self.create_publisher(Float64, f"/rov/thruster_{i}", 10)
                           for i in range(1, 7)]
        self.plan_pub = self.create_publisher(RosPath, "/survey/planned_path", 10)
        self.executed_pub = self.create_publisher(RosPath, "/survey/executed_path", 10)
        self.status_pub = self.create_publisher(String, "/survey/status", 10)
        self.executed = RosPath()
        self.executed.header.frame_id = "map"
        self.create_subscription(Odometry, "/rov/odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/rov/scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.2, self.watchdog)
        self.telemetry_file = (output/"telemetry.csv").open("w", newline="")
        self.writer = csv.writer(self.telemetry_file)
        self.writer.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw", "u", "v", "w",
                              "p", "q", "r", "target_x", "target_y", "cross_track",
                              "depth_error", "leg", "state", "obstacles", "plan_id",
                              "f1", "f2", "f3", "f4", "f5", "f6"])
        self.scans_file = (output/"scans.jsonl").open("w")
        self.plans_file = (output/"plans.jsonl").open("w")
        self.events_file = (output/"events.jsonl").open("w")

    def event(self, t, event, **data):
        self.events_file.write(json.dumps(dict(t=t, event=event, **data))+"\n")
        self.events_file.flush()

    def publish_thrust(self, thrust):
        for publisher, value in zip(self.thrust_publishers, thrust):
            publisher.publish(Float64(data=float(value)))

    def watchdog(self):
        age = time.monotonic() - self.last_wall_odom
        if age > 1.0:
            self.publish_thrust(np.zeros(6))
        if age > 15.0:
            self.finish("FAILED", "No ROS odometry for 15 wall-clock seconds.")

    def on_scan(self, msg):
        t = stamp_seconds(msg.header.stamp)
        self.scan_count += 1
        self.scans_file.write(json.dumps({
            "t": t, "angle_min": msg.angle_min, "angle_increment": msg.angle_increment,
            "range_min": msg.range_min, "range_max": msg.range_max,
            "ranges": [float(v) if np.isfinite(v) else None for v in msg.ranges]})+"\n")
        if not self.history:
            return
        sample = min(self.history, key=lambda s: abs(s[0]-t))
        if abs(sample[0]-t) > 0.10:
            return
        self.last_scan = t
        before = len(self.mapper.tracks)
        self.mapper.update(msg.ranges, msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max, sample[1], sample[2])
        if len(self.mapper.tracks) > before:
            self.event(t, "obstacle_detected", centers=self.mapper.centers.tolist())

    def publish_path(self, path, stamp):
        message = RosPath()
        message.header.frame_id = "map"
        message.header.stamp = stamp
        for x, y in path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.position.z = -self.config["depth"]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.plan_pub.publish(message)

    def on_odom(self, msg):
        self.last_wall_odom = time.monotonic()
        t = stamp_seconds(msg.header.stamp)
        pose, twist = msg.pose.pose, msg.twist.twist
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = pose.orientation
        rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
        velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        omega = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        self.history.append((t, pos, rot))
        if self.first_time is None:
            self.first_time = t
        if self.last_control is not None and t-self.last_control < 0.039:
            return
        dt = 0.04 if self.last_control is None else np.clip(t-self.last_control, 0.001, 0.15)
        self.last_control = t
        if self.finished:
            self.publish_thrust(np.zeros(6))
            return
        if t-self.first_time > self.config["max_duration"]:
            self.finish("FAILED", "Simulation-time mission deadline exceeded.")
            return
        roll, pitch, yaw = Rotation.from_matrix(rot).as_euler("xyz")
        if abs(roll) > 0.65 or abs(pitch) > 0.65 or abs(pos[2]+self.config["depth"]) > 1.:
            self.finish("FAILED", "Attitude or depth operating limit exceeded.")
            return
        target = pos[:2].copy()
        cross_track = 0.
        desired_velocity = np.zeros(2)
        active = (t-self.first_time > 2. and t-self.last_scan < self.config["scan_timeout"])
        self.state = "RUNNING" if active else "HOLD_SENSOR"
        if active:
            leg_start, goal, name = self.legs[self.leg_index]
            goal = np.array(goal)
            centers = self.mapper.centers
            if (np.linalg.norm(pos[:2]-goal) < self.config["goal_tolerance"] and
                    abs(pos[2]+self.config["depth"]) < 0.10 and np.linalg.norm(velocity[:2]) < 0.12):
                self.completed.append({"name": name, "time": t, "position": pos.tolist()})
                self.event(t, "leg_completed", name=name)
                if self.leg_index == len(self.legs)-1:
                    self.finish("COMPLETED", "All five lines and four connectors traversed in order.")
                    return
                self.leg_index += 1
                self.path = None
                leg_start, goal, name = self.legs[self.leg_index]
                goal = np.array(goal)
            replan = self.path is None
            if self.path is not None and self.mapper.version != self.path_version:
                _, s, arc = path_projection(pos[:2], self.path)
                remainder = np.vstack((pos[:2], self.path[1:][arc[1:] > s]))
                replan = any(not self.planner.clear(a, b, centers)
                             for a, b in zip(remainder[:-1], remainder[1:]))
                if not replan:
                    self.path_version = self.mapper.version
            if replan and t-self.last_plan > 0.25:
                self.last_plan = t
                try:
                    self.path = self.planner.plan(pos[:2], goal, centers, leg_start)
                    self.path_version = self.mapper.version
                    self.replans += 1
                    self.plans_file.write(json.dumps({
                        "t": t, "id": self.replans, "leg": self.leg_index,
                        "path": self.path.tolist(), "centers": centers.tolist()})+"\n")
                    self.plans_file.flush()
                    self.publish_path(self.path, msg.header.stamp)
                except ValueError as exc:
                    self.path = None
                    self.state = "HOLD_BLOCKED"
                    self.event(t, "planning_hold", reason=str(exc))
            if self.path is not None:
                cross_track, s, arc = path_projection(pos[:2], self.path)
                target = point_at(self.path, arc, min(s+0.32, arc[-1]))
                delta = target-pos[:2]
                distance = np.linalg.norm(delta)
                if distance > 0.02:
                    self.last_yaw_target = float(np.arctan2(delta[1], delta[0]))
                yaw_error = abs(wrap(self.last_yaw_target-yaw))
                speed = min(self.config["speed"], 0.7*np.linalg.norm(goal-pos[:2]))

                speed *= max(0., 1-yaw_error/0.45)
                desired_velocity = speed*delta/max(distance, 1e-9)
                if yaw_error > 0.45:
                    self.state = "ALIGNING"
                if self.mapper.nearest < 0.55:
                    desired_velocity *= 0
                    self.state = "HOLD_NEAR_RETURN"
        thrust = self.controller.command(pos, rot, velocity, omega, desired_velocity,
                                          self.last_yaw_target, self.config["depth"], dt)
        self.publish_thrust(thrust)
        self.samples += 1
        self.writer.writerow([t, *pos, roll, pitch, yaw, *velocity, *omega, *target,
                              cross_track, pos[2]+self.config["depth"], self.leg_index,
                              self.state, len(self.mapper.tracks), self.replans, *thrust])
        if self.samples % 5 == 0:
            stamped = PoseStamped()
            stamped.header.frame_id = "map"
            stamped.header.stamp = msg.header.stamp
            stamped.pose = pose
            self.executed.header.stamp = msg.header.stamp
            self.executed.poses.append(stamped)
        if t-self.last_status > 2.:
            self.last_status = t
            self.write_status(t)
            self.executed_pub.publish(self.executed)
            self.telemetry_file.flush()
            self.scans_file.flush()
        if t-self.last_print > 10.:
            self.last_print = t
            self.get_logger().info(f"t={t:.1f} {self.state} leg={self.leg_index+1}/9 "
                                   f"xy=({pos[0]:.2f},{pos[1]:.2f}) z={pos[2]:.2f} "
                                   f"obstacles={len(self.mapper.tracks)} plans={self.replans}")

    def write_status(self, t):
        status = {"state": self.state, "reason": self.reason, "simulation_time": t,
                  "wall_time": time.monotonic()-self.wall_start,
                  "completed_legs": self.completed, "leg_index": self.leg_index,
                  "detections": self.mapper.centers.tolist(), "replans": self.replans,
                  "odom_samples": self.samples, "scan_messages": self.scan_count}
        text = json.dumps(status, indent=2)
        (self.output/"status.json").write_text(text+"\n")
        self.status_pub.publish(String(data=text))

    def finish(self, state, reason):
        if self.finished:
            return
        self.finished, self.state, self.reason = True, state, reason
        self.publish_thrust(np.zeros(6))
        t = self.last_control or 0.
        self.event(t, "mission_finished", state=state, reason=reason)
        self.write_status(t)
        self.get_logger().info(f"{state}: {reason}")

    def close(self):
        self.publish_thrust(np.zeros(6))
        for stream in (self.telemetry_file, self.scans_file, self.plans_file, self.events_file):
            stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT/"config/mission.json")
    parser.add_argument("--output", type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = SurveyNode(json.loads(args.config.read_text()), args.output)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish("INTERRUPTED", "Run interrupted.")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.state == "COMPLETED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
