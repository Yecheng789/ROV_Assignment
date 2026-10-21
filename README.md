# BlueROV2 Survey

## Requirements

| Component | Requirement |
| --- | --- |
| Operating system | Ubuntu 24.04 |
| Python | System Python 3.12 |
| Robotics middleware | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic; recorded experiments used Sim 8.11.0 |
| Rendering | EGL/OpenGL support for Ogre2 GPU sensors, including headless runs |
| Vehicle model | `bluerov2_gz` at commit `4451a5d1b15d204581d43ef7d93df4a0508eddbb` |
| Numerical libraries | NumPy, SciPy, Matplotlib |
| Video composition | Pillow, DejaVu fonts |
| Camera capture | FFmpeg, `gz.transport13`, `gz.msgs10` Python bindings |

Use `config/mission.json` for depth, survey lines, speed, safety margin and sensor settings. Use `config/scenario.json`, `config/scenario_calm.json` or `config/scenario_strong_current.json` for the world, obstacles and current. ROS/Gazebo topic mappings are in `config/bridge.yaml`.

## Installation

Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including its APT repository setup. Install the [supported Gazebo pairing](https://gazebosim.org/docs/harmonic/ros_installation/) and project dependencies:

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-ros-gz \
  git python3-numpy python3-scipy python3-matplotlib python3-pytest \
  python3-pil fonts-dejavu-core
source /opt/ros/jazzy/setup.bash
```

Run the remaining commands from the project root. Download the pinned model once:

```bash
mkdir -p vendor
[ -d vendor/bluerov2_gz ] || git clone https://github.com/clydemcqueen/bluerov2_gz.git vendor/bluerov2_gz
git -C vendor/bluerov2_gz checkout 4451a5d1b15d204581d43ef7d93df4a0508eddbb
python3 -m pytest -q
```

For video recording, enable the [official Gazebo APT repository](https://gazebosim.org/docs/harmonic/install_ubuntu/) and install:

```bash
sudo apt update
sudo apt install -y ffmpeg python3-gz-transport13 python3-gz-msgs10
```

An optional ROS package build requires `python3-colcon-common-extensions`:

```bash
sudo apt install -y python3-colcon-common-extensions
colcon build --symlink-install --packages-select rov_survey
source install/setup.bash
```

## Reproduction

Run experiments sequentially. Each output directory must be new. These commands assume a fresh checkout; choose different output paths for additional runs. The launcher uses ROS domain 83 and local discovery.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest -q
bash scripts/run.sh --output results/current
bash scripts/run.sh --scenario config/scenario_calm.json --output results/calm
bash scripts/run.sh --scenario config/scenario_strong_current.json --output results/strong_current
python3 scripts/evaluate.py results/current
python3 scripts/evaluate.py results/calm
python3 scripts/evaluate.py results/strong_current
python3 scripts/check_results.py results/current results/calm results/strong_current
```

To run with the Gazebo window:

```bash
bash scripts/run.sh --output results/gui_run --gui
```

To record and compose a complete mission video:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/record_mission.py --output results/video_run
python3 scripts/evaluate.py results/video_run
python3 scripts/check_results.py results/video_run
python3 scripts/compose_mission_video.py \
  --run results/video_run --output videos/ROV_Survey_Full_Run.mp4
```
