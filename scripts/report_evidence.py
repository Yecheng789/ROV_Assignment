#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("MPLCONFIGDIR", str(ROOT/"results/.mplconfig"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch
from rov_survey.control import RobustController, allocation_matrix, wrap
from rov_survey.planning import mission_legs, segment_distance

plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "pdf.fonttype": 42, "savefig.dpi": 300})
TEAL, GOLD, RED, NAVY = "#087e8b", "#dc9b2d", "#bd4355", "#153047"


def read_run(directory):
    directory = Path(directory)
    rows = list(csv.DictReader((directory/"telemetry.csv").open()))
    data = {k: np.array([float(row[k]) for row in rows]) for k in rows[0] if k != "state"}
    return dict(path=directory, data=data, rows=rows,
                mission=json.loads((directory/"mission.json").read_text()),
                scenario=json.loads((directory/"scenario.json").read_text()),
                metrics=json.loads((directory/"metrics.json").read_text()),
                status=json.loads((directory/"status.json").read_text()),
                plans=[json.loads(s) for s in (directory/"plans.jsonl").read_text().splitlines()],
                scans=[json.loads(s) for s in (directory/"scans.jsonl").read_text().splitlines()],
                events=[json.loads(s) for s in (directory/"events.jsonl").read_text().splitlines()])


def intervals(values):
    if not len(values):
        return []
    return [[round(float(group[0]), 2), round(float(group[-1]), 2), len(group)]
            for group in np.split(values, np.flatnonzero(np.diff(values) > .051)+1)]


def control_replay(run):
    c = RobustController(ROOT/"vendor/bluerov2_gz/models/bluerov2_ping/model.sdf")
    previous, heading = None, 0.
    saturation, force_res, moment_res, discrepancies, raw_history = [], [], [], [], []
    goals = np.array([b for _, b, _ in mission_legs(run["mission"])])
    for row in run["rows"]:
        d = {k: float(v) for k, v in row.items() if k != "state"}
        t = d["t"]
        dt = .04 if previous is None else np.clip(t-previous, .001, .15)
        previous = t
        pos = np.array([d[k] for k in ("x", "y", "z")])
        velocity = np.array([d[k] for k in ("u", "v", "w")])
        omega = np.array([d[k] for k in ("p", "q", "r")])
        rot = Rotation.from_euler("xyz", [d[k] for k in ("roll", "pitch", "yaw")]).as_matrix()
        desired = np.zeros(2)
        if row["state"] in ("RUNNING", "ALIGNING", "HOLD_NEAR_RETURN"):
            delta = np.array([d["target_x"], d["target_y"]])-pos[:2]
            distance = np.linalg.norm(delta)
            if distance > .02:
                heading = float(np.arctan2(delta[1], delta[0]))
            speed = min(run["mission"]["speed"], .7*np.linalg.norm(goals[int(d["leg"])]-pos[:2]))
            speed *= max(0., 1-abs(wrap(heading-d["yaw"]))/.45)
            desired = speed*delta/max(distance, 1e-9)
            if row["state"] == "HOLD_NEAR_RETURN":
                desired *= 0
        old_integral = c.integral.copy()
        command = c.command(pos, rot, velocity, omega, desired, heading, run["mission"]["depth"], dt)
        error = c.v_filtered-rot@velocity
        candidate = np.clip(old_integral+dt*error, -2., 2.)
        world_force = np.array([24., 28., 32.])*error+4*candidate+np.array([1.8, 2., 2.])*np.tanh(error/.055)
        force = rot.T@world_force+np.array([33.732, 54.16, 73.225])*abs(velocity)*velocity
        yaw_rate = np.clip(1.5*wrap(heading-d["yaw"]), -.55, .55)
        wrench = np.r_[force, -1.2*d["roll"]-.8*omega[0],
                       1.8*(yaw_rate-omega[2])+3.992*abs(omega[2])*omega[2]]
        raw = c.inverse@wrench
        actual = np.array([d[f"f{i}"] for i in range(1, 7)])
        discrepancies.append(float(np.max(abs(command-actual))))
        saturation.append(bool(np.any(abs(raw) > 18)))
        residual = c.B@actual-wrench
        force_res.append(float(np.linalg.norm(residual[:3])))
        moment_res.append(float(np.linalg.norm(residual[3:])))
        raw_history.append(raw)
    assert max(discrepancies) < 1e-8, "Controller replay does not match original commands."
    active = run["data"]["plan_id"] > 0
    return dict(
        replay_max_command_error_N=max(discrepancies),
        saturated_cycles=sum(saturation), total_cycles=len(saturation),
        saturated_cycle_percent=100*float(np.mean(saturation)),
        saturation_times_s=run["data"]["t"][np.array(saturation)].tolist(),
        first_recorded_heave_velocity_mps=float(run["data"]["w"][0]),
        max_force_allocation_residual_N=max(force_res),
        max_roll_yaw_allocation_residual_Nm=max(moment_res),
        active_force_residual_max_N=float(np.max(np.asarray(force_res)[active])),
        active_roll_yaw_residual_max_Nm=float(np.max(np.asarray(moment_res)[active])),
        active_saturated_cycles=int(np.sum(np.asarray(saturation)[active])),
        max_raw_thrust_request_N=float(np.max(abs(np.asarray(raw_history)))))


def sonar_points(run, scan):
    d = run["data"]
    j = int(np.argmin(abs(d["t"]-scan["t"])))
    pos = np.array([d[k][j] for k in ("x", "y", "z")])
    rot = Rotation.from_euler("xyz", [d[k][j] for k in ("roll", "pitch", "yaw")]).as_matrix()
    raw = np.array([float("inf") if v is None else v for v in scan["ranges"]])
    angles = scan["angle_min"] + np.arange(len(raw))*scan["angle_increment"]
    mask = np.isfinite(raw) & (raw >= scan["range_min"]) & (raw < 4.)
    rays = np.column_stack((np.cos(angles[mask]), np.sin(angles[mask]), np.zeros(sum(mask))))
    origin = pos + rot@np.array(run["mission"]["sonar_offset"])
    points = origin+(rays*(raw[mask]-.1)[:, None])@rot.T
    return origin, points, raw, angles, mask, abs(float(d["t"][j]-scan["t"]))


def analyze(run):
    d, config = run["data"], run["mission"]
    xy = np.column_stack((d["x"], d["y"]))
    obstacles = np.array(run["scenario"]["obstacles"])
    radius = config["robot_radius"]+run["scenario"]["obstacle_radius"]+config["safety_margin"]
    coverage, sample_rows = [], []
    plans = {p["id"]: np.array(p["path"]) for p in run["plans"]}
    for i, y in enumerate(config["line_y"]):
        ids = np.flatnonzero(d["leg"] == 2*i)
        nominal = np.column_stack((np.linspace(0, 8, 161), np.full(161, y)))
        distance, nearest = cKDTree(xy[ids]).query(nominal)
        feasible = np.min(np.linalg.norm(nominal[:, None]-obstacles[None, :], axis=2), axis=1) > radius
        missed = feasible & (distance > .25)
        plan_distance = np.zeros(161)
        for j, point in enumerate(nominal):
            plan = plans.get(int(d["plan_id"][ids[nearest[j]]]))
            plan_distance[j] = (min(float(segment_distance([point], a, b)[0])
                               for a, b in zip(plan[:-1], plan[1:])) if plan is not None else np.nan)
            sample_rows.append(dict(line=i+1, x=float(point[0]), y=y, distance_m=float(distance[j]),
                                   category="covered" if distance[j] <= .25 else
                                   ("inflated_obstacle" if not feasible[j] else "residual_gap"),
                                   nominal_feasible=bool(feasible[j]),
                                   nearest_active_plan_distance_m=float(plan_distance[j])))
        coverage.append(dict(line=i+1, nominal_percent=100*float(np.mean(distance <= .25)),
                             unobstructed_percent=100*float(np.mean(distance[feasible] <= .25)),
                             residual_intervals_x=intervals(nominal[missed, 0]),
                             residual_count=int(sum(missed)),
                             residual_also_outside_active_plan=int(sum(missed & (plan_distance > .25))),
                             endpoint_distances_m=distance[[0, -1]].tolist()))
    detections = []
    previous = 0
    for event in run["events"]:
        if event["event"] != "obstacle_detected":
            continue
        scan = min(run["scans"], key=lambda s: abs(s["t"]-event["t"]))
        origin, points, raw, _, mask, offset = sonar_points(run, scan)
        for j in range(previous, len(event["centers"])):
            estimate = np.array(event["centers"][j])
            truth_index = int(np.argmin(np.linalg.norm(obstacles-estimate, axis=1)))
            hit_error = abs(np.linalg.norm(points[:, :2]-obstacles[truth_index], axis=1)-.25)
            matches = hit_error < .1
            ranges = raw[mask][matches]
            detections.append(dict(track=j+1, true_obstacle=truth_index+1, time_s=event["t"],
                                   logged_estimate=estimate.tolist(),
                                   associated_raw_range_m=float(min(ranges)) if len(ranges) else None,
                                   associated_corrected_range_m=float(min(ranges)-.1) if len(ranges) else None,
                                   pose_match_offset_s=offset))
        previous = len(event["centers"])
    max_excursion = float(max(0, -xy.min(), xy.max()-8))
    result = dict(coverage=coverage, first_detections=detections,
                  mean_nominal_coverage_percent=float(np.mean([v["nominal_percent"] for v in coverage])),
                  mean_unobstructed_coverage_percent=float(np.mean([v["unobstructed_percent"] for v in coverage])),
                  xy_min=xy.min(axis=0).tolist(), xy_max=xy.max(axis=0).tolist(),
                  maximum_center_excursion_m=max_excursion,
                  allocation=control_replay(run))
    return result, sample_rows


def axes_grid(fig, box, rows=1, cols=1, **kwargs):
    x, y, w, h = box
    spec = fig.add_gridspec(rows, cols, left=x, right=x+w, bottom=y, top=y+h, **kwargs)
    return [fig.add_subplot(spec[i, j]) for i in range(rows) for j in range(cols)]


def scene_figure(fig, box, run):
    axes = axes_grid(fig, box, 2, 1, hspace=.10)
    for ax, name, title in zip(axes, ("overview", "vehicle"),
                               ("Survey world: five cylinders and seabed lines",
                                "Standard six-thruster BlueROV2 with forward Ping surrogate")):
        ax.imshow(plt.imread(ROOT/f"results/report_evidence/scene/{name}.png"))
        ax.axis("off")
        ax.set_title(title, fontsize=10)


def architecture_figure(fig, box, run):
    ax = fig.add_axes(box)
    ax.set(xlim=(0, 10), ylim=(0, 7))
    ax.axis("off")
    def node(x, y, w, h, text, color="#e6f3f4"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=.08",
                                    facecolor=color, edgecolor=TEAL))
        ax.text(x+w/2, y+h/2, text, ha="center", va="center", fontsize=9)
    node(.1, 4.8, 2.5, 1.4, "Gazebo Harmonic\n6-DOF dynamics\nPing + odometry")
    node(3.5, 4.8, 2.1, 1.4, "ros_gz_bridge\nLaserScan / Odometry\nFloat64 thrust")
    node(6.6, 4.1, 3.2, 2.6, "rov_survey (Python / rclpy)\n\nSonarMapper → Planner\n↓\nGuidance → RobustController")
    node(.1, .7, 2.5, 1.5, "scenario.json\nworld.sdf\ntrue obstacle positions", "#fff1df")
    node(3.5, .7, 2.1, 1.5, "evaluate.py\nreport_evidence.py\noffline only", "#fff1df")
    node(6.6, .7, 3.2, 1.5, "telemetry.csv / scans.jsonl\nplans.jsonl / events.jsonl\nimmutable experiment records")
    for a, b in [((2.65, 5.7), (3.4, 5.7)), ((5.7, 5.7), (6.5, 5.7)),
                 ((6.5, 4.9), (5.7, 4.9)), ((3.4, 4.9), (2.65, 4.9)),
                 ((1.3, 2.3), (1.3, 4.7)), ((2.7, 1.4), (3.4, 1.4)),
                 ((8.2, 4.), (8.2, 2.3)), ((6.5, 1.4), (5.7, 1.4))]:
        ax.annotate("", xy=b, xytext=a, arrowprops={"arrowstyle": "->", "color": NAVY})
    ax.text(5, 3.05, "Obstacle truth has no input edge to the online node.", ha="center", fontsize=10)


def sonar_figure(fig, box, run):
    scan = min(run["scans"], key=lambda s: abs(s["t"]-.05))
    origin, points, raw, angles, mask, offset = sonar_points(run, scan)
    axes = axes_grid(fig, box, 2, 1, hspace=.55)
    ax = axes[0]
    finite = np.isfinite(raw)
    ax.plot(np.rad2deg(angles[finite]), raw[finite], "o-", color=TEAL, label="Recorded raw range")
    ax.plot(np.rad2deg(angles[mask]), raw[mask]-.1, "s--", color=GOLD, label="Bias-corrected range")
    for angle in angles[~finite]:
        ax.annotate("no return", xy=(np.rad2deg(angle), 2.5), ha="center", rotation=90, fontsize=8)
    degrees = np.rad2deg(angles)
    ax.set(xlabel="Beam bearing [deg]", ylabel="Range [m]",
           xlim=(degrees.min()-1, degrees.max()+1), xticks=degrees,
           title=f"Recorded Ping scan at t = {scan['t']:.2f} s")
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
    ax = axes[1]
    for point in points:
        ax.plot([origin[0], point[0]], [origin[1], point[1]], color=TEAL, alpha=.6)
    center = np.array(run["events"][0]["centers"][0])
    ax.scatter(points[:, 0], points[:, 1], color=GOLD, s=35, label="Corrected ray hits")
    ax.add_patch(Circle((3, 0), .25, fill=False, color=RED, label="True cylinder (offline)"))
    ax.scatter(*center, marker="+", s=100, color="black", label="Center stored in event log")
    ax.scatter(*origin[:2], marker=">", color=TEAL, label="Sensor origin")
    ax.set(xlim=(0, 3.5), ylim=(-.5, .5), aspect="equal", xlabel="World x [m]", ylabel="World y [m]")
    ax.legend(fontsize=7, loc="upper left", ncol=2)
    ax.grid(alpha=.2)


def paths_figure(fig, box, run):
    x, y, width, height = box
    ax = fig.add_axes((x, y, width, height-.07))
    d = run["data"]
    for a, b, _ in mission_legs(run["mission"]):
        ax.plot([a[0], b[0]], [a[1], b[1]], "--", color="#9aa4ad", lw=1)
    for plan in run["plans"]:
        path = np.array(plan["path"])
        ax.plot(*path.T, color=GOLD, lw=1, alpha=.7)
    ax.plot(d["x"], d["y"], color=TEAL, lw=1.8, label="Executed path")
    ax.plot([], [], color=GOLD, label="Online plan history")
    for j, center in enumerate(run["scenario"]["obstacles"]):
        ax.add_patch(Circle(center, .25, color=RED, label="True obstacle" if j == 0 else None))
        ax.add_patch(Circle(center, .6, fill=False, linestyle=":", color=RED))
    ax.add_patch(Rectangle((0, 0), 8, 8, fill=False, edgecolor=NAVY, label="8 m survey boundary"))
    ax.add_patch(Rectangle((-.4, -.4), 8.8, 8.8, fill=False, edgecolor=NAVY,
                           linestyle="--", label="Planning bounds"))
    ax.scatter(*np.array(run["status"]["detections"]).T, marker="+", color="black", s=40)
    ax.set(xlim=(-.65, 8.65), ylim=(-.65, 8.65), aspect="equal", xlabel="World x [m]", ylabel="World y [m]")
    ax.legend(fontsize=8, ncol=2, loc="upper center", bbox_to_anchor=(.5, 1.16))
    ax.grid(alpha=.15)


def replanning_figure(fig, box, run):
    axes = axes_grid(fig, box, 2, 2, hspace=.40, wspace=.25)
    d = run["data"]
    for ax, t in zip(axes, (101., 102.1, 111., 126.)):
        plans = [p for p in run["plans"] if p["t"] <= t]
        plan = plans[-1]
        path = np.array(plan["path"])
        ax.plot(*path.T, color=GOLD, label="Active plan")
        mask = (d["leg"] == 4) & (d["t"] <= t)
        ax.plot(d["x"][mask], d["y"][mask], color=TEAL, lw=2, label="Executed to this time")
        j = int(np.argmin(abs(d["t"]-t)))
        ax.scatter(d["x"][j], d["y"][j], color=TEAL, s=25)
        ax.add_patch(Circle((2.8, 4), .25, color=RED, alpha=.7))
        for center in plan["centers"]:
            if np.linalg.norm(np.array(center)-[2.8, 4]) < .7:
                ax.add_patch(Circle(center, .9, fill=False, edgecolor="black", ls=":"))
                ax.scatter(*center, marker="+", color="black")
        ax.axhline(4, color="gray", ls="--", lw=.6)
        ax.set(xlim=(-.3, 5.5), ylim=(2.5, 5.5), aspect="equal",
               title=f"t = {t:.1f} s; active plan {plan['id']}", xlabel="x [m]", ylabel="y [m]")
    axes[0].legend(fontsize=7, loc="lower right")


def coverage_figure(fig, box, run):
    _, samples = analyze_coverage_cached(run)
    ax = fig.add_axes(box)
    colors = {"covered": TEAL, "inflated_obstacle": RED, "residual_gap": GOLD}
    for category, color in colors.items():
        selected = [row for row in samples if row["category"] == category]
        ax.scatter([r["x"] for r in selected], [r["line"] for r in selected],
                   marker="s", s=22, color=color,
                   label={"covered": "Covered", "inflated_obstacle": "Missed: inflated zone",
                          "residual_gap": "Missed: outside zone"}[category])
    ax.set(yticks=range(1, 6), ylim=(.5, 5.5), xlim=(-.1, 8.1),
           xlabel="Nominal-line x coordinate [m]", ylabel="Survey line")
    ax.invert_yaxis()
    ax.legend(fontsize=7, ncol=3, loc="upper center", bbox_to_anchor=(.5, -.30))
    ax.grid(alpha=.15)


def errors_figure(fig, box, run):
    axes = axes_grid(fig, box, 3, 1, hspace=.25)
    d = run["data"]
    xy = np.column_stack((d["x"], d["y"]))
    obstacles = np.array(run["scenario"]["obstacles"])
    clearance = np.min(np.linalg.norm(xy[:, None]-obstacles[None], axis=2), axis=1)-.6
    for ax, values, ylabel, color in zip(axes, (d["cross_track"], d["depth_error"], clearance),
                                        ("Cross-track [m]", "z + 5 [m]", "Envelope clearance [m]"),
                                        (TEAL, "#735b91", RED)):
        ax.plot(d["t"], values, color=color, lw=1)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=.2)
        for leg in run["status"]["completed_legs"]:
            ax.axvline(leg["time"], color="gray", alpha=.15)
    axes[-1].axhline(0, color="black", ls="--", lw=.8)
    axes[-1].axhline(.3, color=GOLD, ls=":", lw=1, label="Nominal margin (not a guarantee)")
    axes[-1].legend(fontsize=7)
    axes[-1].set_xlabel("Simulation time [s]")


def actuation_figure(fig, box, run):
    axes = axes_grid(fig, box, 3, 1, hspace=.35)
    d = run["data"]
    for i in range(1, 7):
        axes[0].plot(d["t"], d[f"f{i}"], lw=.7, label=f"T{i}")
    for limit in (-18, 18):
        axes[0].axhline(limit, color=RED, ls="--", lw=1)
    axes[0].set(ylabel="Thrust [N]", ylim=(-20, 20))
    axes[0].legend(ncol=6, fontsize=7)
    for name in ("roll", "pitch"):
        axes[1].plot(d["t"], np.rad2deg(d[name]), label=name)
    axes[1].set_ylabel("Roll / pitch [deg]")
    axes[1].legend(ncol=2, fontsize=8)
    axes[2].plot(d["t"], np.rad2deg(np.unwrap(d["yaw"])), color=TEAL)
    axes[2].set(ylabel="Yaw [deg]", xlabel="Simulation time [s]")
    for ax in axes:
        ax.grid(alpha=.2)


def analyze_coverage_cached(run):
    if "_evidence" not in run:
        run["_evidence"] = analyze(run)
    return run["_evidence"]


FIGURES = {
    "scene": scene_figure, "architecture": architecture_figure, "sonar": sonar_figure,
    "paths": paths_figure, "replanning": replanning_figure, "coverage": coverage_figure,
    "errors": errors_figure, "actuation": actuation_figure,
}


def generate(runs):
    loaded = [read_run(p) for p in runs]
    out = ROOT/"results/report_evidence"
    out.mkdir(exist_ok=True)
    evidence = {}
    for run in loaded:
        analysis, samples = analyze_coverage_cached(run)
        evidence[run["path"].name] = analysis
        with (out/f"{run['path'].name}_coverage_samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
    evidence["source_hashes"] = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for run in loaded for path in run["path"].iterdir()
                                 if path.name in ("telemetry.csv", "scans.jsonl", "plans.jsonl",
                                                  "events.jsonl", "status.json", "metrics.json", "world.sdf")}
    evidence["allocation_matrix_5x6"] = allocation_matrix(
        ROOT/"vendor/bluerov2_gz/models/bluerov2_ping/model.sdf").tolist()
    (out/"analysis.json").write_text(json.dumps(evidence, indent=2)+"\n")
    figures = ROOT/"docs/figures"
    figures.mkdir(exist_ok=True)
    for name, draw in FIGURES.items():
        fig = plt.figure(figsize=(8, 7 if name != "coverage" else 4))
        draw(fig, (.12, .17, .82, .72), loaded[0])
        fig.savefig(figures/(name+".pdf"))
        fig.savefig(figures/(name+".png"), dpi=300)
        plt.close(fig)
    print(out/"analysis.json")
    return loaded, evidence


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", nargs="+", type=Path, default=[
        ROOT/"results/current", ROOT/"results/calm", ROOT/"results/strong_current"])
    args = parser.parse_args()
    generate(args.runs)
