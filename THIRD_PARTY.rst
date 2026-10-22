Third-party model provenance
============================

Installation places the unmodified upstream repository in ``vendor/bluerov2_gz``.
This directory is excluded from Git; the installation commands in ``README.md``
download the pinned revision.

* Project: BlueROV2 in Gazebo Harmonic, by Clyde McQueen and contributors.
* Source: https://github.com/clydemcqueen/bluerov2_gz
* Pinned revision: 4451a5d1b15d204581d43ef7d93df4a0508eddbb
* Upstream package version: 0.7.0.
* Upstream ``package.xml`` declares the MIT license. This checkout does not
  contain a separate root LICENSE file; original asset files and notices
  are retained, and no new ownership of those assets is claimed.

``scripts/prepare_world.py`` derives a new SDF without changing the upstream
files. Changes: forward/fixed sonar mounting, one vertical ray layer, 10 m
maximum range, removal of the ArduPilot interface, odometry publisher, current,
survey world and obstacles. The original six thrusters, inertia, buoyancy
collision volume, hydrodynamic coefficients and five horizontal Ping rays
are preserved.

ROS 2, Gazebo, NumPy, SciPy and Matplotlib are external installed dependencies.
They are not copied into the project and retain their respective licenses.
