#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def check(directory, data_only=False):
    directory = Path(directory)
    metrics = json.loads((directory/"metrics.json").read_text())
    manifest = json.loads((directory/"manifest.json").read_text())
    events = [json.loads(line) for line in (directory/"events.jsonl").read_text().splitlines()]
    with (directory/"telemetry.csv").open() as stream:
        samples = list(csv.DictReader(stream))
    t = np.array([float(row["t"]) for row in samples])
    scan_count = sum(1 for _ in (directory/"scans.jsonl").open())
    finished = [e["name"] for e in events if e["event"] == "leg_completed"]
    expected = ["line_1", "connector", "line_2", "connector", "line_3",
                "connector", "line_4", "connector", "line_5"]
    checks = {
        "mission_completed": metrics["completed"],
        "nine_leg_events_in_order": finished == expected,
        "timestamps_strictly_increasing": bool(np.all(np.diff(t) > 0)),
        "control_gap_under_0_2_sim_s": bool(np.max(np.diff(t)) < .2),
        "recorded_scan_count_matches": scan_count == metrics["scan_messages"],
        "all_cylinders_detected": metrics["detected_obstacles"] == metrics["true_obstacles"],
        "positive_conservative_clearance": metrics["minimum_hull_clearance_m"] > 0,
        "tracking_rmse_under_0_1m": metrics["cross_track_rmse_m"] < .1,
        "max_depth_error_under_0_1m": metrics["depth_max_abs_m"] < .1,
        "thrust_bounds_respected": metrics["max_abs_thruster_N"] <= 18.000001,
        "independent_tracking_matches_node": metrics["max_tracking_log_discrepancy_m"] < 1e-8,
    }
    source_hash_check = "skipped"
    if not data_only:
        sources = {name: value for name, value in manifest["source_sha256"].items()
                   if name.startswith("rov_survey/") and name.endswith(".py")}
        hashes_match = bool(sources) and all(
            (ROOT/name).is_file() and hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == value
            for name, value in sources.items())
        checks["delivered_controller_matches_run_hashes"] = hashes_match
        source_hash_check = "passed" if hashes_match else "failed"
    return {"run": str(directory), "passed": all(checks.values()), "checks": checks,
            "source_hash_check": source_hash_check,
            "max_control_gap_sim_s": float(np.max(np.diff(t)))}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--data-only", action="store_true",
                        help="Check archived data without comparing acquisition source hashes to this checkout.")
    args = parser.parse_args()
    audits = [check(run, data_only=args.data_only) for run in args.runs]
    result = {"passed": all(audit["passed"] for audit in audits), "runs": audits}
    (ROOT/"results/validation.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
