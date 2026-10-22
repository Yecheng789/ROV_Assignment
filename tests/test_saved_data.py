import hashlib
import json

import pytest

from scripts import check_results


@pytest.fixture
def saved_run(tmp_path, monkeypatch):
    monkeypatch.setattr(check_results, "ROOT", tmp_path)
    source = tmp_path / "rov_survey/control.py"
    source.parent.mkdir()
    source.write_text("value = 1\n")
    run = tmp_path / "run"
    run.mkdir()
    metrics = {
        "completed": True, "scan_messages": 1,
        "detected_obstacles": 1, "true_obstacles": 1,
        "minimum_hull_clearance_m": 0.3, "cross_track_rmse_m": 0.02,
        "depth_max_abs_m": 0.01, "max_abs_thruster_N": 10,
        "max_tracking_log_discrepancy_m": 0,
    }
    (run / "metrics.json").write_text(json.dumps(metrics))
    (run / "manifest.json").write_text(json.dumps({"source_sha256": {
        "rov_survey/control.py": hashlib.sha256(source.read_bytes()).hexdigest()}}))
    names = ["line_1", "connector", "line_2", "connector", "line_3",
             "connector", "line_4", "connector", "line_5"]
    (run / "events.jsonl").write_text("\n".join(
        json.dumps({"event": "leg_completed", "name": name}) for name in names))
    (run / "telemetry.csv").write_text("t\n0\n0.04\n")
    (run / "scans.jsonl").write_text("{}\n")
    return run, source


def test_default_verification_requires_original_source(saved_run):
    run, source = saved_run
    assert check_results.check(run)["passed"]
    source.write_text("value = 2\n")
    result = check_results.check(run)
    assert not result["passed"]
    assert result["source_hash_check"] == "failed"


def test_data_only_skips_source_comparison_but_keeps_data_checks(saved_run):
    run, source = saved_run
    source.unlink()
    result = check_results.check(run, data_only=True)
    assert result["passed"]
    assert result["source_hash_check"] == "skipped"
    assert len(result["checks"]) == 11
    metrics = json.loads((run / "metrics.json").read_text())
    metrics["minimum_hull_clearance_m"] = -0.1
    (run / "metrics.json").write_text(json.dumps(metrics))
    result = check_results.check(run, data_only=True)
    assert not result["passed"]
    assert not result["checks"]["positive_conservative_clearance"]


def test_default_verification_rejects_empty_source_hashes(saved_run):
    run, _ = saved_run
    (run / "manifest.json").write_text(json.dumps({"source_sha256": {}}))
    assert not check_results.check(run)["passed"]
