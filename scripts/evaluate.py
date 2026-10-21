#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np
os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parents[1]/"results/.mplconfig"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rov_survey.planning import mission_legs, segment_distance


def evaluate(directory):
    directory = Path(directory)
    scenario = json.loads((directory/"scenario.json").read_text())
    mission = json.loads((directory/"mission.json").read_text())
    status = json.loads((directory/"status.json").read_text())
    with (directory/"telemetry.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError("Insufficient telemetry; do not infer a successful run.")
    data = {key: np.array([float(row[key]) for row in rows])
            for key in rows[0] if key != "state"}
    t = data["t"]
    xyz = np.column_stack([data[k] for k in ("x", "y", "z")])
    xy = xyz[:, :2]
    plans = [json.loads(line) for line in (directory/"plans.jsonl").read_text().splitlines()]
    measured_cross_track = np.zeros(len(rows))
    for plan in plans:
        selected = data["plan_id"] == plan["id"]
        path = np.asarray(plan["path"])
        error = np.full(np.count_nonzero(selected), np.inf)
        for a, b in zip(path[:-1], path[1:]):
            error = np.minimum(error, segment_distance(xy[selected], a, b))
        measured_cross_track[selected] = error
    tracking_log_discrepancy = np.max(np.abs(measured_cross_track-data["cross_track"]))
    data["cross_track"] = measured_cross_track
    centers = np.asarray(scenario["obstacles"]).reshape(-1, 2)
    radius = mission["robot_radius"]
    obstacle_radius = scenario["obstacle_radius"]
    center_distance = (np.min(np.linalg.norm(xy[:, None]-centers[None, :], axis=-1), axis=1)
                       if len(centers) else np.full(len(xy), np.inf))
    clearance = center_distance-radius-obstacle_radius

    swept = min((np.min(segment_distance(centers, a, b))-radius-obstacle_radius
                 for a, b in zip(xy[:-1], xy[1:])), default=np.inf) if len(centers) else np.inf
    active = data["plan_id"] > 0
    tracking = data["cross_track"][active]
    detected = np.asarray(status["detections"]).reshape(-1, 2)
    detection_errors = (np.min(np.linalg.norm(centers[:, None]-detected[None, :], axis=-1), axis=1)
                        if len(detected) and len(centers) else np.array([]))
    per_line = []
    for i, y in enumerate(mission["line_y"]):
        segment = xy[data["leg"] == 2*i]
        nominal = np.column_stack((np.linspace(0, 8, 161), np.full(161, y)))
        if len(centers):
            distance_to_obstacle = np.min(np.linalg.norm(nominal[:, None]-centers[None, :], axis=-1), axis=1)
            feasible = distance_to_obstacle > radius+obstacle_radius+mission["safety_margin"]
        else:
            feasible = np.ones(len(nominal), dtype=bool)
        distance = cKDTree(segment).query(nominal)[0] if len(segment) else np.full(len(nominal), np.inf)
        per_line.append({"line": i+1, "y": y, "nominal_points": len(nominal),
                         "coverage_within_0_25m_percent": float(100*np.mean(distance <= .25)),
                         "unobstructed_coverage_within_0_25m_percent": float(100*np.mean(distance[feasible] <= .25)),
                         "start_reached": bool(distance[0] < .15),
                         "end_reached": bool(distance[-1] < .15)})
    complete_lines = sum(leg["name"].startswith("line_") for leg in status["completed_legs"])
    with (directory/"evaluation.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "center_distance", "hull_clearance", "cross_track", "depth_error"])
        writer.writerows(zip(t, center_distance, clearance, data["cross_track"], data["depth_error"]))
    metrics = {
        "backend": "Gazebo Harmonic + ROS 2 Jazzy; six physical thruster plugins",
        "evaluator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "completed": bool(status["state"] == "COMPLETED" and complete_lines == 5 and swept > 0),
        "mission_state": status["state"], "completed_survey_lines": complete_lines,
        "completed_legs": len(status["completed_legs"]),
        "duration_sim_s": float(status["simulation_time"]-t[0]),
        "distance_travelled_3d_m": float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))),
        "distance_travelled_horizontal_m": float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))),
        "nominal_route_length_m": 48.,
        "cross_track_rmse_m": float(np.sqrt(np.mean(tracking**2))) if len(tracking) else None,
        "cross_track_p95_m": float(np.percentile(tracking, 95)) if len(tracking) else None,
        "cross_track_max_m": float(np.max(tracking)) if len(tracking) else None,
        "max_tracking_log_discrepancy_m": float(tracking_log_discrepancy),
        "depth_rmse_m": float(np.sqrt(np.mean(data["depth_error"]**2))),
        "depth_max_abs_m": float(np.max(np.abs(data["depth_error"]))),
        "minimum_hull_clearance_m": float(swept) if np.isfinite(swept) else None,
        "minimum_center_to_obstacle_surface_m": float(np.min(center_distance)-obstacle_radius) if len(centers) else None,
        "robot_enclosing_radius_m": radius,
        "collision_by_conservative_envelope": bool(swept <= 0),
        "max_abs_roll_deg": float(np.rad2deg(np.max(np.abs(data["roll"])))),
        "max_abs_pitch_deg": float(np.rad2deg(np.max(np.abs(data["pitch"])))),
        "max_abs_thruster_N": float(max(np.max(np.abs(data[f"f{i}"])) for i in range(1, 7))),
        "replans": status["replans"],
        "detected_obstacles": len(detected),
        "true_obstacles": len(centers),
        "maximum_nearest_detection_error_m": float(np.max(detection_errors)) if len(detection_errors) else None,
        "scan_messages": status["scan_messages"], "control_samples": len(rows),
        "current_world_m_per_s": scenario["current"],
        "line_coverage": per_line,
        "metric_definitions": {
            "cross_track": "Distance to the active collision-free polyline at each control sample after the first plan.",
            "clearance": "Minimum horizontal segment-to-cylinder-center distance minus obstacle radius and 0.35 m vehicle envelope.",
            "completion": "All five alternating line endpoints and four connectors in order, final tolerance 0.10 m, positive conservative clearance.",
            "coverage": "Samples every 0.05 m on each nominal line within 0.25 m of execution during that line; detours reduce nominal coverage.",
            "planned_length": "Plans are revised online; concatenating full plans double-counts unexecuted portions, so no single total planned length is reported."
        }
    }
    (directory/"metrics.json").write_text(json.dumps(metrics, indent=2)+"\n")
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.dpi": 130})
    fig, ax = plt.subplots(figsize=(8, 8))
    for a, b, name in mission_legs(mission):
        ax.plot([a[0], b[0]], [a[1], b[1]], "--", color="#a5aab3", lw=1)
    for plan in plans:
        path = np.array(plan["path"])
        ax.plot(path[:, 0], path[:, 1], color="#e8a135", alpha=.7, lw=1.4)
    ax.plot(xy[:, 0], xy[:, 1], color="#087e8b", lw=2, label="Executed (Gazebo odometry)")
    ax.plot([], [], color="#e8a135", label="Online planned paths")
    ax.plot([], [], "--", color="#a5aab3", label="Nominal survey route")
    for j, center in enumerate(centers):
        ax.add_patch(Circle(center, obstacle_radius, color="#d1495b", label="True obstacle" if j == 0 else None))
        ax.add_patch(Circle(center, radius+obstacle_radius, fill=False, ls=":", color="#d1495b"))
    if len(detected):
        ax.scatter(*detected.T, marker="+", color="black", s=80, label="Sonar-estimated center")
    ax.scatter(*xy[0], color="green", s=60, zorder=5)
    ax.scatter(*xy[-1], color="black", marker="s", s=45, zorder=5)
    for i in range(5):
        ax.text(-.25, 2*i+.15, f"Line {i+1}", fontsize=9)
    ax.set(xlabel="World x [m]", ylabel="World y [m]", aspect="equal",
           xlim=(-.7, 8.7), ylim=(-.7, 8.7),
           title="BlueROV2 Ping: planned and executed survey")
    ax.legend(loc="center left", bbox_to_anchor=(1, .5), fontsize=8)
    ax.grid(alpha=.2)
    fig.savefig(directory/"paths.png", bbox_inches="tight")
    fig.savefig(directory/"paths.pdf", bbox_inches="tight")
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    axes[0].plot(t, data["cross_track"], color="#087e8b")
    axes[0].set(ylabel="Cross-track error [m]", title="Measured tracking and conservative obstacle clearance")
    axes[1].plot(t, data["depth_error"], color="#665191")
    axes[1].set(ylabel="z + 5 [m]")
    axes[2].plot(t, clearance, color="#d1495b")
    axes[2].axhline(0, color="black", ls="--", label="Collision envelope")
    axes[2].set(xlabel="Simulation time [s]", ylabel="Hull clearance [m]")
    axes[2].legend()
    for ax in axes:
        ax.grid(alpha=.2)
        for leg in status["completed_legs"]:
            ax.axvline(leg["time"], color="gray", alpha=.2)
    fig.savefig(directory/"errors_clearance.png")
    fig.savefig(directory/"errors_clearance.pdf")
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, layout="constrained")
    for i in range(1, 7):
        axes[0].plot(t, data[f"f{i}"], lw=.8, label=f"T{i}")
    axes[0].set(ylabel="Thrust [N]", title="Actuation and measured vehicle attitude")
    axes[0].legend(ncol=6)
    for key in ("roll", "pitch", "yaw"):
        axes[1].plot(t, np.rad2deg(np.unwrap(data[key])), label=key)
    axes[1].set(ylabel="Angle [deg]", xlabel="Simulation time [s]")
    axes[1].legend(ncol=3)
    for ax in axes:
        ax.grid(alpha=.2)
    fig.savefig(directory/"actuation.png")
    plt.close(fig)
    print(json.dumps({k: v for k, v in metrics.items() if k not in ("line_coverage", "metric_definitions")}, indent=2))
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    evaluate(args.run)
