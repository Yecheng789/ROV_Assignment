Installation and reproduction
=============================

Target environment: Ubuntu 24.04 (Noble), system Python 3.12, ROS 2 Jazzy,
Gazebo Harmonic. The recorded experiments used Gazebo Sim 8.11.0. Installing
the current packages can select a newer compatible Harmonic patch version;
check the run manifest when comparing results. These instructions follow the
official sources below. A separate clean-machine installation was not executed
for the report revision; the existing workstation, package build and saved
experiment checks were tested.

1. Install ROS and simulation dependencies
------------------------------------------

Run this section only on a machine needing installation. Use a UTF-8 locale.
The following commands change system packages and require sudo::

    sudo apt update
    sudo apt install -y locales software-properties-common curl git python3
    sudo locale-gen en_US en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo add-apt-repository -y universe

Install the official ROS repository configuration package. Its release version
is resolved from the official ros-infrastructure repository::

    ROV_ROS_APT_VERSION=$(curl -fsSL \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
    curl -fL -o /tmp/rov-ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROV_ROS_APT_VERSION}/ros2-apt-source_${ROV_ROS_APT_VERSION}.noble_all.deb"
    sudo dpkg -i /tmp/rov-ros2-apt-source.deb
    sudo apt update
    sudo apt install -y ros-jazzy-ros-base ros-jazzy-ros-gz \
      python3-colcon-common-extensions python3-numpy python3-scipy \
      python3-matplotlib python3-pytest python3-pil python3-docutils \
      poppler-utils
    source /opt/ros/jazzy/setup.bash

The ROS vendor packages provide the supported Jazzy/Harmonic pairing. No
ArduSub/SITL or custom compiled Gazebo plugin is required. The application uses
rclpy plus standard ROS messages. Use the system Python packages so that rclpy
and its binary dependencies remain visible; no pip installation into the system
interpreter is needed. ``requirements.txt`` lists the core Python requirements.

Scene recapture is optional and is separate from mission execution and report
rebuilding. ``scripts/capture_report_scene.py`` additionally uses Gazebo Python
bindings ``gz.transport13`` and ``gz.msgs10``. They were available on the tested
machine as ``python3-gz-transport13`` and ``python3-gz-msgs10``. If absent, install
them from the official Gazebo Harmonic repository following its installation
documentation, or keep the supplied scene images. The report builder does not
need these bindings. Ogre2 GPU sensors require a functioning EGL/OpenGL renderer;
headless mode removes the window, not the rendering requirement.

2. Source checkout and optional ROS package build
-------------------------------------------------

Run commands from the project root. The launcher resolves paths relative to
its own source location. The upstream checkout is excluded from Git and must
be downloaded once. Retain its Git metadata because the launcher records its
commit. The expected revision is
``4451a5d1b15d204581d43ef7d93df4a0508eddbb``::

    mkdir -p vendor
    git clone https://github.com/clydemcqueen/bluerov2_gz.git vendor/bluerov2_gz
    git -C vendor/bluerov2_gz checkout 4451a5d1b15d204581d43ef7d93df4a0508eddbb

Do not clone over the already supplied vendor directory. Verify and test::

    source /opt/ros/jazzy/setup.bash
    gz sim --versions
    git -C vendor/bluerov2_gz rev-parse HEAD
    python3 -m pytest -v

The recommended entry point is the source-tree launcher ``scripts/run.sh``.
The package also supports an optional symlink build::

    colcon build --symlink-install --packages-select rov_survey
    source install/setup.bash
    ros2 pkg executables rov_survey

The console entry point is ``survey_node = rov_survey.node:main``. For resource
resolution, retain the checkout and use the symlink build; the package does not
install all worlds and vendor assets as standalone resources. Starting only
``survey_node`` does not start Gazebo or the bridge. The launcher starts all
three processes and invokes ``python3 -m rov_survey.node`` itself.

3. Generate the world and run experiments
-----------------------------------------

The original primary world is ``results/current/world.sdf``. A world-only build
is useful for inspecting model changes::

    python3 scripts/prepare_world.py --scenario config/scenario.json \
      --output worlds/survey.sdf

Run each experiment sequentially, using a new output directory::

    bash scripts/run.sh --output results/repeat_current
    bash scripts/run.sh --scenario config/scenario_calm.json \
      --output results/repeat_calm
    bash scripts/run.sh --scenario config/scenario_strong_current.json \
      --output results/repeat_strong

Add ``--gui`` to display the Gazebo client. The default is server-side headless
rendering. The runner refuses to overwrite an existing telemetry log and stops
only its own child processes on completion or interruption.

The runner supplies these environment settings automatically:

* ``GZ_SIM_RESOURCE_PATH``: project ``vendor/bluerov2_gz/models``.
* ``GZ_PARTITION``: unique per run; Gazebo process isolation.
* ``ROS_DOMAIN_ID=83`` and ``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``.
  Run only one project experiment at a time in this ROS domain.
* ``GZ_HOMEDIR`` and ``ROS_LOG_DIR``: directories under the run output.
* ``PYTHONPATH``: includes the source checkout.

The bridge configuration is ``config/bridge.yaml``. Online settings are copied
from ``config/mission.json`` and contain no obstacle positions. Scenario truth
is read by world construction and offline evaluation. Each output directory
contains the generated SDF, configuration snapshots, manifest, raw scans,
telemetry, plans, mission events and process logs.

4. Evaluate and rebuild
-----------------------

For newly recorded runs::

    python3 scripts/evaluate.py results/repeat_current
    python3 scripts/evaluate.py results/repeat_calm
    python3 scripts/evaluate.py results/repeat_strong
    python3 scripts/check_results.py \
      results/repeat_current results/repeat_calm results/repeat_strong

For the original assignment archive, the reports can be rebuilt from its saved
experiments::

    python3 scripts/build_report.py \
      --runs results/current results/calm results/strong_current

The Git checkout excludes saved runs. Use the fresh-run commands in
``README.md`` to recreate them, and run ``scripts/capture_report_scene.py``
before building the report to generate its scene images.

``check_results.py`` checks exact source-file hashes. Comment and docstring
cleanup changes those hashes even when executable code is unchanged. Use this
check on runs produced by the current checkout. Historical manifests in the
assignment archive retain the hashes of their original source version.

The report includes explanatory snapshots at times selected for the supplied
primary run. Replacing the experiment set requires reviewing those narrative
examples and the scene provenance as well as rebuilding computed tables. Do not
treat a regenerated PDF for an arbitrary new layout as a fully reviewed report.

``scripts/evaluate.py`` recomputes metrics and original diagnostic plots from
raw logs. ``scripts/report_evidence.py`` derives extra coverage classifications,
first-detection diagnostics and controller replay, writes source hashes, and
exports vector/300 dpi figures. ``build_report.py`` calls that analysis, renders
the final PDF directly with vector graphics and writes the editable RST source.
The saved source hashes provide a basis for checking original log integrity.

The existing revision verification outputs are:

* ``results/report_evidence/tests.txt``: seven named pytest checks.
* ``results/report_evidence/validation.txt`` and ``results/validation.json``:
  12 consistency/acceptance checks for each of three runs.
* ``results/report_evidence/analysis.json``: numerical derived evidence and hashes.

Official sources
----------------

* `ROS 2 Jazzy Ubuntu installation <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>`_
* `Official ROS installation source <https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst>`_
* `Gazebo Harmonic with ROS <https://gazebosim.org/docs/harmonic/ros_installation/>`_
* `Gazebo Harmonic Ubuntu installation <https://gazebosim.org/docs/harmonic/install_ubuntu/>`_

5. Editable Word report and video recording
-------------------------------------------

Optional tools for Word generation, preview and MP4 capture::

    sudo apt install -y pandoc python3-docx libreoffice-writer libreoffice-math ffmpeg

The current workstation uses workspace-local unpacked document tools in
``tools/``; these are excluded from the submission archive. A standard package
installation supplies the same interfaces. Run the shared content generator
before exporting Word, particularly after moving the project to another path::

    python3 scripts/build_report.py
    python3 scripts/build_report_word.py

``report_content.json`` is the shared text/table/equation source. Word contains
native Office Math equations and editable tables. ``report.pdf`` retains the
vector renderer; ``docs/word_preview/report.pdf`` is an actual Word rendering
used to check pagination and formulas. Standard LibreOffice preview command::

    libreoffice --headless --convert-to pdf --outdir docs/word_preview docs/report.docx

Record a new full mission with two Gazebo rendering sensors (requires the
Gazebo Python bindings discussed in Section 1)::

    source /opt/ros/jazzy/setup.bash
    python3 scripts/record_mission.py --output results/video_repeat
    python3 scripts/evaluate.py results/video_repeat
    python3 scripts/compose_mission_video.py --run results/video_repeat \
      --output videos/repeat.mp4

The recorder changes the requested real-time factor to 0.6 to allow camera
capture, while all control and video timing use simulation timestamps. It adds
an overhead camera and a render-only sensor on base_link; no vehicle dynamics
or controller gains are changed. Raw camera streams and timestamps remain in
the output directory. The composed video plays at one simulation second per
video second, followed by a three-second final-frame summary.

The supplied recording is ``results/video_run_20261021`` and is separate from
the three original quantitative experiments. Rebuild the submission archive
with ``python3 scripts/package_submission.py`` after updating deliverables.
