#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$PROJECT_ROOT"
python3 scripts/run_simulation.py "$@"
