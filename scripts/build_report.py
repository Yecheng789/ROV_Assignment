#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import numpy as np
from report_evidence import ROOT, FIGURES, generate
from report_layout import Report


def build(paths):
    paths = [Path(p).resolve() for p in paths]
    runs, evidence = generate(paths)
    run = runs[0]
    m = run['metrics']
    a = evidence[paths[0].name]
    output = ROOT/'docs'
    report = Report(output)
    def figure(name, height, caption):
        report.figure(name, FIGURES[name], run, height, caption)

    report.page('Sonar-guided underwater survey')
    report.p('LTU-RAI PhD Recruitment Assignment\nName: Yecheng Wang\nDate: 21 October 2026')
    report.sub('Abstract')
    report.p('This project implements a sonar-guided survey of an 8 m by 8 m area at 5 m depth, using a standard six-thruster BlueROV2 in ROS 2 Jazzy and Gazebo Harmonic. A sparse forward sonar map informs online A* planning, while velocity PI feedback, a smooth nonlinear term and drag feedforward drive the six simulated thrusters.')
    report.p(f'The primary run completed all nine ordered mission legs in {m["duration_sim_s"]:.2f} s over {m["distance_travelled_3d_m"]:.3f} m. Active-path cross-track RMSE was {m["cross_track_rmse_m"]:.4f} m and minimum clearance under an assumed 0.35 m circular vehicle footprint was {m["minimum_hull_clearance_m"]:.4f} m. Nominal-line proximity coverage averaged {a["mean_nominal_coverage_percent"]:.2f}%, or {a["mean_unobstructed_coverage_percent"]:.2f}% after excluding the defined inflated obstacle regions; ordered completion does not imply full line or area coverage.')
    report.p('Three single runs with constant currents provide empirical disturbance-rejection evidence. The report distinguishes this evidence from a full stability or collision guarantee, documents the simulator sonar approximation, and supplies code, original logs and reproducible metric calculations.')
    report.sub('1. Task and operating assumptions')
    report.p('Five 8 m lines at y = 0, 2, 4, 6 and 8 m are traversed in alternating directions, with four connectors. Static cylinders have known radius 0.25 m but unknown locations online. World z points upward, so the depth setpoint is z = -5 m. Simulated pose and velocity feedback are permitted by the task.')
    report.p('The survey square is interpreted as a footprint, without enclosing walls. The planner permits the vehicle center in [-0.4, 8.4] m on both horizontal axes. This 0.4 m allowance is an implementation assumption. Obstacles span the survey depth, so avoidance is horizontal. No acoustic swath width is specified; the proximity metric is not an area-coverage estimate.')
    report.p('The following sections describe the model, simulation, perception, planning, controller and measured results. Appendices locate the implementation and reproduce its checks; Figures 1–8 show the scene, data flow and recorded behavior.')

    report.page('2. Motion model and control inputs')
    report.p('Let the body frame have x forward, y left and z upward. R maps body vectors to world coordinates. Position and Euler angles form eta; nu contains body linear and angular velocities. The equations use the coordinate conventions defined above [1].')
    report.eq(r'\eta=[x,y,z,\phi,\theta,\psi]^T,\quad \nu=[u,v,w,p,q,r]^T,\quad \dot\eta=J(\eta)\nu', .058, 11)
    report.eq(r'M_{RB}\dot\nu+C_{RB}(\nu)\nu+M_A\dot\nu_r+C_A(\nu_r)\nu_r+D(\nu_r)\nu_r+g(\eta)=B_6 f+d', .058, 10)
    report.eq(r'\nu_r=\nu-[(R^T v_c)^T,0,0,0]^T,\quad f\in\mathbb{R}^{6},\quad B_6\in\mathbb{R}^{6\times6}', .055, 11)
    report.p('M, C, D and g denote inertia, Coriolis/centripetal terms, damping and restoring forces. The world current is v_c; d collects unmodeled wrench. Each signed f_i is thrust in N. Equation (2) is the general marine model; the actual Gazebo configuration retains the simplified upstream coefficients [2,3].')
    report.table('Retained physical parameters from the pinned model.', ['Quantity', 'Value and units'], [
        ['Mass', 'Base 10.0 kg; 6 thruster links at 0.002 kg each; Ping 0.133 kg'],
        ['Base principal inertia', '(0.099, 0.178, 0.269) kg m²'],
        ['Added mass / linear drag', 'All configured coefficients zero'],
        ['Quadratic translation drag', '(33.732, 54.16, 73.225) N s²/m²; positive magnitudes'],
        ['Quadratic angular drag', '(3.992, 3.992, 3.992) N m s²/rad²; positive magnitudes'],
        ['Buoyancy box / water', '0.457 × 0.338 × 0.066 m; center z = +0.06 m; density 1000 kg/m³']
    ], [.32,.68])
    report.p('Retaining the upstream hydrodynamic model makes the simulation traceable to the supplied vehicle. It is not an identified real-vehicle model; zero added mass limits the physical interpretation of transient performance.')
    report.eq(r'B_{6,i}=[a_i^T,(\ell_i\times a_i)^T]^T,\quad B_5=S B_6\in\mathbb{R}^{5\times6}', .055, 11)
    report.p('a_i is the thrust axis and ell_i its lever arm from the SDF. S selects surge, sway, heave, roll and yaw (rows 1, 2, 3, 4 and 6). Pitch remains in the six-DOF simulation but is passively stabilized; Table 7 gives B_5. Translational rows are dimensionless and moment rows have units m.')

    report.page('3. Simulation setup and model changes')
    report.p('The platform is Ubuntu 24.04, Python 3.12, ROS 2 Jazzy and Gazebo Harmonic 8.11.0. The model is the standard six-thruster Ping variant, not BlueROV2 Heavy. The local upstream commit is 4451a5d1b15d204581d43ef7d93df4a0508eddbb [2]. Table 2 compares that SDF with scripts/prepare_world.py.')
    report.table('Complete model-level change inventory.', ['Property', 'Pinned upstream', 'Survey model'], [
        ['Vehicle spawn pose', '(0,0,0), level', '(0,0,-5), level'],
        ['Ping link offset [m]', '(-0.18,0,-0.095)', '(0.25,0,-0.095)'],
        ['Sensor orientation', 'Pitch 1.571 rad; downward', 'Zero rotation; forward'],
        ['Ping joint', 'Revolute, with axis', 'Fixed; axis removed'],
        ['Horizontal samples / angles', '5; ±0.14835 rad (~17° full)', 'Unchanged'],
        ['Vertical samples / angles', '3; 0 to 0.14835 rad', '1; 0 rad'],
        ['Range [m] / resolution', '0.2 to 50 / 0.02 m', '0.2 to 10 / unchanged'],
        ['Rate / Gaussian noise', '20 Hz / mean +0.1 m, std 0.005 m', 'Unchanged'],
        ['Sensor visualization', 'Enabled', 'Disabled'],
        ['ArduPilotPlugin', 'Present', 'Removed; direct ROS thrust'],
        ['Hydrodynamics additions', 'No explicit scenario current', 'default_current; model namespace'],
        ['OdometryPublisher', 'Absent', '3D, 50 Hz; map / base_link'],
    ], [.28,.36,.36], 8)
    report.p('The thruster plugins, geometry, inertia and buoyancy box are retained. The generated world adds physics, IMU and Ogre2 sensor systems, graded buoyancy, seabed, lighting, static cylinders and visual-only seabed survey lines. The step is 0.002 s and requested real-time factor is 2; control uses simulation time.', 9)
    report.p('Five horizontal rays were already present upstream. Keeping them supplies independent bearing/range information useful for fitting a known-radius circle. This is a GPU-lidar surrogate: real Ping hardware is a single-beam echosounder, whose current product page specifies a 25° acoustic beam [7]. That hardware value is distinct from the retained 17° simulator aperture.', 9)

    report.page('3.1 Gazebo scene and geometry')
    figure('scene', .54, 'Gazebo renders of the saved primary world: overview and vehicle detail. Cameras were added in a separate visualization run on 21 October 2026; both frames are at t = 0.004 s. These illustrate geometry, not an original mission snapshot or an additional performance trial.')
    report.p('Cylinder centers are (3,0), (5,2), (2.8,4), (5.4,6) and (3.8,8) m, with radius 0.25 m, height 4 m and center z = -5 m. The seabed is centered at z = -8 m. Only world construction and offline evaluation read these locations.', 9)
    report.p('Figure 1 was captured by scripts/capture_report_scene.py from results/current/world.sdf. The derived visualization SDF, camera provenance and original 2400 × 1600 images are in results/report_evidence/scene/. Source experiment logs were not replaced.', 9)

    report.page('3.2 Software and information flow')
    figure('architecture', .29, 'ROS/Gazebo architecture and information separation. Sonar processing, planning and control run as modules of one Python/rclpy node. Obstacle truth enters world construction and offline evaluation only.')
    report.p('scripts/run.sh launches Gazebo, ros_gz_bridge and python3 -m rov_survey.node. SurveyNode integrates SonarMapper, Planner and RobustController. config/mission.json has no true obstacle coordinates; the online node does not read scenario.json. Thrust commands reach the original Gazebo Thruster systems through the bridge [5,6].', 9)
    report.table('ROS interfaces (full mappings: config/bridge.yaml).', ['Topic', 'Type / direction / purpose'], [
        ['/rov/odometry', 'nav_msgs/Odometry; Gazebo → node; map pose, body twist'],
        ['/rov/scan', 'sensor_msgs/LaserScan; Ping → node; five bearings'],
        ['/rov/thruster_1 … _6', 'std_msgs/Float64; node → Gazebo; signed force [N]'],
        ['/survey/planned_path', 'nav_msgs/Path; node → visualization; current polyline'],
        ['/survey/executed_path', 'nav_msgs/Path; node → visualization; sampled odometry'],
        ['/survey/status; /clock', 'std_msgs/String status JSON; rosgraph_msgs/Clock simulation time'],
    ], [.40,.60], 8)
    report.p('OdometryPublisher [4] supplies states at 50 Hz; a control step is accepted approximately every 0.04 s (25 Hz). Scans arrive at 20 Hz. No pose-setting command is used to move the vehicle. Appendix A gives the exact modules and recorded artifacts.', 9)

    report.page('4. Sonar perception and map updates')
    report.p('Invalid, nonfinite and range-limit returns are rejected. Returns below 4 m enter detection after subtracting the modeled +0.1 m bias. The nearest buffered odometry pose must be within 0.10 s. Sensor hits are transformed using the mount offset and full body attitude:')
    report.eq(r'p_i=p+R\{p_m+(\rho_i-b)[\cos\alpha_i,\sin\alpha_i,0]^T\},\quad b=0.1\ {\rm m}', .06, 11)
    report.p('Here p is world position, p_m = (0.25,0,-0.095) m, rho_i is raw range and alpha_i is beam bearing. Adjacent returns are clustered unless indices are discontinuous or separation exceeds 0.6 m; points outside the survey-depth slice are discarded.')
    report.eq(r'\hat c=\arg\min_c\ \sum_i \mathcal{L}(\Vert p_{i,xy}-c\Vert-0.25)', .057, 12)
    report.p('Equation (6) uses robust least squares for sufficiently separated multi-ray hits. A single ray initializes a center one radius behind its hit; lateral position is then ambiguous. Estimates within 0.65 m associate to an existing track. Multi-ray estimates refine the center with gain 0.25; single rays do not overwrite an established center. Tracks persist through missing returns.')
    report.table('Primary run: first map insertions and associated scan returns.', ['Cylinder', 'Time [s]', 'Raw [m]', 'Corrected [m]'], [
        [str(v['true_obstacle']), f"{v['time_s']:.2f}", f"{v['associated_raw_range_m']:.3f}", f"{v['associated_corrected_range_m']:.3f}"]
        for v in a['first_detections']], [.22,.25,.25,.28])
    report.p('Times and center estimates come from events.jsonl. Range association in Table 4 is an offline diagnostic against known cylinder surfaces, not a new input to the node. Reconstructed ray hits use the nearest logged control pose, so they are illustrations rather than bit-exact replay of the online pose buffer (at most 0.02 s offset in the primary data).', 9)
    initial = max(np.linalg.norm(np.array(v['logged_estimate'])-run['scenario']['obstacles'][v['true_obstacle']-1]) for v in a['first_detections'])
    report.p(f'The largest first-insertion center error is {initial:.3f} m; the largest final nearest-center error is {m["maximum_nearest_detection_error_m"]:.4f} m. Final map accuracy must not be substituted for uncertainty at first detection. Figure 3 shows a concrete scan-to-map example.', 9)

    report.page('4.1 A recorded scan-to-map example')
    figure('sonar', .51, 'Primary scan at t = 0.05 s. The upper panel shows recorded and bias-corrected ranges at their own bearings. The lower panel transforms valid hits to the world and compares the stored map estimate with the true cylinder, used here only for offline illustration.')
    v = a['first_detections'][0]
    report.p(f'The nearest associated raw return is {v["associated_raw_range_m"]:.3f} m. The logged first center is ({v["logged_estimate"][0]:.4f}, {v["logged_estimate"][1]:.4f}) m. The pose used to draw this scan is {v["pose_match_offset_s"]:.2f} s from its timestamp; the stored estimate itself is taken directly from the event log.', 9)
    report.p('The forward mount allows obstacle returns before reaching a cylinder on a line. A narrow forward aperture still misses some side geometry while turning. Equal-radius cylinders and isolated tracks simplify fitting; nearby objects can merge. Real single-beam Ping deployment would need additional observations, such as controlled yaw sweeps, to resolve the bearing ambiguity.', 9)

    report.page('5. Online path planning and mission order')
    report.p('I selected eight-connected A* for the small planar workspace because it supports explicit obstacle-clearance checks and a simple cost for returning to the survey line. The line-deviation penalty encourages rejoining the active segment after each bypass.')
    report.eq(r'r_{\rm plan}=r_o+r_v+m_s=0.25+0.35+0.30=0.90\ {\rm m}', .052, 12)
    report.eq(r'c(a,b)=\Vert b-a\Vert\,[1+\lambda d(b,L)],\quad \lambda=4\ {\rm m}^{-1}', .052, 12)
    report.p('The grid resolution is 0.10 m with an extra 0.02 m occupancy allowance. Diagonals cannot cut occupied corners. Local line-of-sight simplification and start/goal links are checked analytically against inflated disks. A blocked endpoint or failed search commands zero horizontal velocity. New legs and map changes that invalidate the remaining path trigger replanning.')
    report.p('Plans are geometric polylines, not timed trajectories. A 0.32 m lookahead gives the target direction; speed is limited to 0.28 m/s and decreases near the endpoint. Heading error scales horizontal speed to zero at 0.45 rad. Each velocity-reference component changes by at most 0.18 m/s²; consequently a zero request does not stop the vehicle instantaneously.')
    goals = [((0,0),(8,0)),((8,0),(8,2)),((8,2),(0,2)),((0,2),(0,4)),((0,4),(8,4)),((8,4),(8,6)),((8,6),(0,6)),((0,6),(0,8)),((0,8),(8,8))]
    report.table('Ordered nominal endpoints and recorded completion times.', ['Leg', 'Start → goal [m]', 'Time [s]'], [
        [f"{i+1}: {v['name']}", f'{start} → {goal}', f"{v['time']:.2f}"] for i,(v,(start,goal)) in enumerate(zip(run['status']['completed_legs'],goals))
    ], [.29,.46,.25], 8)
    report.p('A leg completes when horizontal goal error < 0.10 m, depth error < 0.10 m and horizontal body speed < 0.12 m/s. The nine endpoints total 48 m nominal travel. Table 5 establishes order and timing; Section 7.4 separately measures proximity coverage.', 9)

    report.page('6. Feedback law and parameter choices')
    report.p('Guidance provides a world-frame horizontal velocity request. Depth control requests clip[1.1(-5-z), -0.20, 0.20] m/s. The acceleration-limited reference is v_d; world velocity is R times body linear velocity. For translational error e and integral I, the implemented feedback is:')
    report.eq(r'e=v_d-R[u,v,w]^T,\quad \dot I=e,\quad F_w=K_v e+K_I I+K_s\tanh(e/\epsilon)', .06, 11)
    report.eq(r'F_b=R^T F_w+Q_t|\nu_t|\nu_t,\quad \nu_t=[u,v,w]^T', .05, 12)
    report.eq(r'r_d={\rm clip}(1.5\,{\rm wrap}(\psi_d-\psi),-0.55,0.55)', .05, 12)
    report.eq(r'\tau_\phi=-1.2\phi-0.8p,\quad \tau_\psi=1.8(r_d-r)+3.992|r|r', .052, 12)
    report.table('Controller gains, bounds and physical units.', ['Parameter', 'Value', 'Unit / role'], [
        ['K_v (diagonal)', '24, 28, 32', 'N s/m; velocity feedback'],
        ['K_I (diagonal)', '4, 4, 4', 'N/m; constant residual rejection'],
        ['K_s / epsilon', '(1.8,2,2) / 0.055', 'N / m/s; smooth nonlinear feedback'],
        ['Integral bound', '±2 per axis', 'm; elementwise clipping'],
        ['Depth gain / speed cap', '1.1 / 0.20', '1/s / m/s'],
        ['Yaw gain / rate cap', '1.5 / 0.55', '1/s / rad/s'],
        ['Yaw rate / roll damping', '1.8 / 0.8', 'N m s/rad'],
        ['Roll restoring gain', '1.2', 'N m/rad'],
        ['Reference acceleration', '0.18 per axis', 'm/s²'],
        ['Per-thruster force limit', '±18', 'N'],
    ], [.35,.25,.40], 8)
    report.p('I used PI velocity feedback to reject persistent residuals and a smooth tanh term to add bounded feedback without a discontinuous sign function. Q_t uses the translational drag magnitudes in Table 1 to compensate still-water drag, while the imposed current remains unknown to the controller. The 0.055 m/s scale sets the tanh transition width.', 9)
    report.p('The fixed engineering gains are evaluated in the three recorded current conditions. Gain optimization and an ablation without tanh were outside this evaluation, so the individual contribution of the nonlinear term remains unresolved.', 9)

    report.page('6.1 Allocation and local control rationale')
    report.eq(r'\tau_5=[F_x,F_y,F_z,\tau_\phi,\tau_\psi]^T,\quad f={\rm clip}(B_5^\dagger\tau_5,-18,18)', .057, 11)
    matrix = evidence['allocation_matrix_5x6']
    report.table('Geometry-derived B_5, rounded for display (rank 5).', ['Axis', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6'], [
        [axis]+[f'{x:.4f}' if abs(x)>1e-6 else '0' for x in row] for axis,row in zip(['Fx','Fy','Fz','Roll','Yaw'],matrix)
    ], [.16]+[.14]*6, 8)
    report.p('The code computes the full-precision columns from each thruster axis and lever arm. The six-DOF wrench B_6 f includes pitch; the allocator uses its five selected rows B_5. If any unconstrained thrust exceeds ±18 N, all channels are clipped individually and the integral update is rolled back for that cycle (conditional anti-windup).')
    report.sub('6.1.1 Local, unsaturated interpretation')
    report.p('Consider a reduced constant-inertia translational error model after nominal compensation, with positive diagonal gains and lumped residual d_e. It abstracts away attitude coupling, reference switches, saturation and the integral limiter:')
    report.eq(r'M_e\dot e=-K_v e-K_I I-K_s\tanh(e/\epsilon)+d_e,\quad \dot I=e', .057, 11)
    report.eq(r'V=\frac{1}{2} e^T M_e e+\frac{1}{2} I^T K_I I', .055, 13)
    report.eq(r'\dot V=-e^T K_v e-e^T K_s\tanh(e/\epsilon)+e^T d_e', .057, 12)
    report.p('The cross terms cancel in this reduced model. For zero residual, the velocity-dependent terms dissipate energy. For nonzero residual, eᵀd_e can be positive, and the expression has no strictly negative term in I. These equations alone do not prove input-to-state stability or full-state boundedness under arbitrary bounded time-varying disturbances.')
    report.p('This local analysis provides a rationale for the feedback terms. It is not a global stability proof for the implemented hybrid, saturated six-DOF system. Disturbance rejection is assessed empirically in the three constant-current conditions of Section 7.')

    report.page('6.2 Detection margins and safety meaning')
    report.sub('6.2.1 Assumed footprint and inflation')
    report.p('The planner uses a 0.35 m circular horizontal vehicle radius. It exceeds the 0.284 m circumscribed radius of the base rectangle, but the full frame, thrusters, forward Ping and attitude-dependent projection have not been certified to fit this disk. Therefore the metric is explicitly named clearance under an assumed circular footprint.')
    report.p('The extra 0.30 m inflation is a heuristic allowance for mapping and tracking error. The largest initial map-center error is about 0.332 m in the primary run, so the allowance does not bound every initial estimate. The 0.02 m grid allowance accompanies continuous segment checking; it is not a replacement for that check.')
    report.eq(r'c_{\min}=\min_{k,j}d([p_{k,xy},p_{k+1,xy}],c_j)-r_o-r_v', .06, 12)
    report.p('Equation (17) uses consecutive executed segments and true cylinder centers offline. Positive values mean those horizontal segments do not intersect the assumed obstacle-plus-vehicle disks. This is conditional geometric evidence; Gazebo contact records were not collected, and the full vehicle geometry has not been independently checked. The raw JSON field minimum_hull_clearance_m is retained for data compatibility with this narrower interpretation.')
    report.sub('6.2.2 Zero requests, sensing delay and emergency threshold')
    report.p('Scan staleness above 0.5 simulated seconds or a nearest valid raw return below 0.55 m commands zero horizontal velocity while retaining depth/heading feedback if odometry is available. Reference filtering, inertia and current still affect actual translation. The 0.55 m threshold is applied before subtracting the +0.1 m bias.')
    report.p('For an illustrative level center ray, raw range 0.55 m means corrected sensor-to-surface range 0.45 m. Adding the 0.25 m forward mount gives 0.70 m from the base to the surface, or 0.35 m remaining under the assumed radius. At 0.28 m/s and an assumed 0.10 m/s² deceleration, braking distance alone is 0.392 m; 0.5 s latency adds up to 0.14 m. That deceleration is an illustration, not an established lower bound.')
    report.p('Thus the near-return rule is not a certified stopping barrier; earlier detection within 4 m and replanning provide the principal avoidance mechanism in these scenes. One wall second without odometry commands zero thrust and 15 s ends the run. Zero thrust cannot guarantee depth hold. Attitude/depth limits and the mission deadline also terminate the run.')

    report.page('7. Experiment design and measured results')
    report.p('Three fresh Gazebo processes used seed 23, the same cylinder layout and identical controller/planner settings. Only the spatially uniform constant current changed. Each started with an empty sonar map. Metrics below are computed from the original ROS logs, with source hashes and environment details in each run manifest.')
    report.table('One realization per current; all completed five lines and four connectors.', ['Run / current x,y [m/s]', 'Legs', 'Time [s]', 'Travel [m]', 'RMSE [m]', 'Clearance [m]'], [
        [f"{r['path'].name}\n({r['metrics']['current_world_m_per_s'][0]:.2f}, {r['metrics']['current_world_m_per_s'][1]:.2f})", f"{r['metrics']['completed_legs']}/9", f"{r['metrics']['duration_sim_s']:.2f}", f"{r['metrics']['distance_travelled_3d_m']:.3f}", f"{r['metrics']['cross_track_rmse_m']:.4f}", f"{r['metrics']['minimum_hull_clearance_m']:.4f}"] for r in runs
    ], [.29,.10,.14,.15,.15,.17], 8)
    report.p('Clearance is the minimum segment clearance under the assumed 0.35 m circular footprint, as defined in Section 6.2. All three values are positive but below the 0.30 m nominal planning margin; that margin was not maintained throughout execution.', 9)
    report.sub('7.1 Evaluation definitions')
    report.eq(r'L_{\rm exec}=\sum_k\Vert p_{k+1}-p_k\Vert,\quad e_{z,k}=z_k+5', .053, 12)
    report.eq(r'e_{\perp,k}=\min_{s\in P_k}\Vert p_{k,xy}-s\Vert,\quad {\rm RMSE}=\sqrt{N^{-1}\sum_k e_{\perp,k}^2}', .058, 11)
    report.p('P_k is the active planned polyline at sample k. Cross-track statistics exclude samples before the first plan, and do not measure timed-reference lag or distance from the original nominal line. Travel uses 3D increments; summing entire replanned paths would double-count unexecuted portions. Clearance uses segments to avoid hiding crossings between samples.')
    report.p(f'Primary depth RMSE is {m["depth_rmse_m"]:.5f} m and maximum absolute depth error is {m["depth_max_abs_m"]:.5f} m. Cross-track 95th percentile is {m["cross_track_p95_m"]:.4f} m and maximum is {m["cross_track_max_m"]:.4f} m. The record has {m["control_samples"]} control samples, {m["scan_messages"]} scans and {m["replans"]} plans. Duration is final event time minus the first odometry timestamp, not the absolute clock reading.')
    report.p(f'The primary route is {m["distance_travelled_3d_m"]-48:.3f} m ({100*(m["distance_travelled_3d_m"]/48-1):.2f}%) longer than the nominal 48 m, with mean speed {m["distance_travelled_3d_m"]/m["duration_sim_s"]:.3f} m/s. Figures 4–8 and Table 9 separate path following, bypasses, coverage, attitude and actuation.')
    report.p('Single realizations with one layout do not establish success probability, a maximum safe current, or robustness to time-varying current and arbitrary unseen obstacles. No additional controller comparison is inferred from this table.')

    report.page('7.2 Planned and executed paths')
    figure('paths', .52, 'Primary run: nominal lines/connectors (gray), online plan history (gold), execution (teal), true obstacles (red) and final estimates (black crosses). Dotted red circles have radius 0.60 m (obstacle plus assumed vehicle); planning uses 0.90 m. Solid and dashed squares show the survey and allowed center-planning bounds.')
    report.p(f'The executed center ranges from x = {a["xy_min"][0]:.4f} to {a["xy_max"][0]:.4f} m and y = {a["xy_min"][1]:.4f} to {a["xy_max"][1]:.4f} m. The maximum center excursion beyond the survey square is {a["maximum_center_excursion_m"]:.4f} m. This quantifies the boundary assumption; the survey square was not enforced as a physical wall.', 9)
    report.p('Older plans may cross a cylinder that had not yet been observed. The overlay therefore must be read with timestamps: a historical planned segment is not an executed collision. Figure 5 resolves one bypass into its successive active plans.', 9)

    report.page('7.3 Detection and replanning over time')
    figure('replanning', .51, 'Primary run, third survey line. At 101.0 s the new cylinder is absent from the map; its red true outline is drawn offline for reference. After detection at 102.05 s, the active route changes. Later panels show the executed bypass and return toward the nominal y = 4 m line. Dotted disks are the active estimated 0.90 m exclusions.')
    report.p('Each panel uses the last plan published before its displayed time and only execution up to that time. The source records are plans.jsonl and telemetry.csv. This distinguishes the initially incomplete map from the path followed after a new return invalidates the remaining route.', 9)
    report.p('There are 14 primary plan records, including initial planning and leg transitions; the count is not 14 emergency avoidance events. The snapshot sequence shows why a single overlay of all plans can contain apparently conflicting routes.', 9)

    report.page('7.4 Ordered completion and line coverage')
    report.p('For each nominal line, 161 points at 0.05 m intervals are covered when the nearest executed sample during that line is within 0.25 m. The second denominator retains only points strictly outside the true 0.90 m inflated disks (the original evaluator threshold is preserved). This is sampled line proximity, not acoustic swath coverage.', 9)
    figure('coverage', .17, 'Primary coverage categories. Gold points are the remaining misses outside the inflated obstacles; red marks missed points within inflation. The full sampled classification is supplied as current_coverage_samples.csv.')
    report.table('Per-line coverage and residual missed sample intervals.', ['Line', 'All [%]', 'Outside inflation [%]', 'Residual x intervals [m]'], [
        [str(v['line']), f"{v['nominal_percent']:.2f}", f"{v['unobstructed_percent']:.2f}", '; '.join(f'[{lo:.2f}, {hi:.2f}]' for lo,hi,_ in v['residual_intervals_x'])] for v in a['coverage']
    ], [.10,.17,.27,.46], 8)
    misses=sum(v['residual_count'] for v in a['coverage'])
    plan_misses=sum(v['residual_also_outside_active_plan'] for v in a['coverage'])
    report.p(f'Means are {a["mean_nominal_coverage_percent"]:.2f}% nominal and {a["mean_unobstructed_coverage_percent"]:.2f}% outside inflation. The {misses} residual missed samples lie beside bypass entries/exits. For {plan_misses}, even the active plan at the nearest executed sample is more than 0.25 m away; for the other {misses-plan_misses}, that plan lies within 0.25 m but execution does not. This supports bypass geometry plus path-following deviation as sources of the residual loss.', 9)
    report.p('Every nominal endpoint has an executed sample within 0.104 m, so endpoint tolerance does not explain these residual gaps at a 0.25 m threshold. The log comparison does not isolate lookahead, current or discretization causally. Interval bounds are discrete sample locations, not exact continuous gap lengths. The assignment specifies no swath width or coverage pass threshold; nine completed legs remain distinct from 100% line coverage.', 9)

    report.page('7.5 Tracking, depth and assumed clearance')
    figure('errors', .53, 'Primary cross-track error relative to the active plan, depth error relative to -5 m, and sampled clearance under the assumed circular footprint. Vertical markers show completed legs. The dotted 0.30 m line is the nominal margin, not a guaranteed bound. Table 8 uses the stricter segment minimum.')
    report.p('The positive footprint metric is consistent with the recorded detours and their bounded geometric tracking errors in this run. Mapping and following errors reduce the nominal planning margin. No decomposition into independent worst-case error bounds is available, so these curves cannot certify future unseen scenes.', 9)
    report.p('The depth and clearance metrics include the startup transient and use the sample selection defined in Section 7.1. The accompanying per-sample records allow the reported statistics to be recomputed.', 9)

    report.page('7.6 Actuation and startup transient')
    figure('actuation', .43, 'Primary commanded thruster forces with ±18 N rails; roll/pitch shown separately from unwrapped yaw. The first thrust sample saturates. Later active navigation remains below the limits; angles are measured physical attitudes, without a planar pose constraint.')
    alloc=a['allocation']
    report.p(f'Controller replay reproduces logged thrust to {alloc["replay_max_command_error_N"]:.2e} N. Exactly {alloc["saturated_cycles"]} of {alloc["total_cycles"]} cycles ({alloc["saturated_cycle_percent"]:.4f}%) saturates, at t = 0.02 s before the first plan. The first recorded heave velocity is {alloc["first_recorded_heave_velocity_mps"]:.3f} m/s, inconsistent with the tiny position change from the initial pose. This is an odometry startup anomaly; its precise simulator cause has not been established.', 9)
    report.p(f'The anomalous velocity feeds quadratic drag compensation, producing an unconstrained per-thruster request up to {alloc["max_raw_thrust_request_N"]:.3e} N and force-allocation residual {alloc["max_force_allocation_residual_N"]:.3e} N. These are rejected numerical requests, not applied forces: the two vertical channels clip to +18 N. This initial sample is reported separately because it dominates the allocation residual.', 9)
    report.p('All three runs show one such initial saturated cycle and zero saturated cycles after the first plan. A velocity-validity gate before enabling feedback is a priority for future implementation.', 9)

    report.page('8. Limitations and conclusion')
    report.sub('8.1 Evidence boundaries')
    report.p(f'During primary active navigation, the maximum force-allocation residual is {alloc["active_force_residual_max_N"]:.2e} N and the selected roll/yaw residual is {alloc["active_roll_yaw_residual_max_Nm"]:.2e} N m. These near-roundoff values follow from the unsaturated five-axis allocation; they do not include the uncontrolled pitch component. The large startup residual is reported separately in Section 7.6.')
    report.p('The simulator retains zero added mass and simplified drag. The GPU-lidar surrogate omits acoustic multipath, reflection strength and single-beam ambiguity. State estimation uses allowed simulator feedback. A static, equal-radius map can merge close objects; the forward aperture is not a complete visibility model. A 0.35 m footprint and 0.30 m margin are assumptions, not certified full-vehicle safety bounds.')
    report.p('The quantitative evaluation covers three constant currents, one layout and one realization per condition. Controller ablation, randomized layouts, time-varying currents, contact logging and fault injection were outside the scope of the present evaluation. These experiments would help assess individual controller contributions and generalization. The supplied checks verify the implementation and consistency of the stored metrics.')
    report.sub('8.2 Conclusion')
    report.p(f'The implemented ROS/Gazebo system completes all nine ordered mission legs, including the obstacle bypasses, in each of the three recorded current conditions. In the primary run, active-path RMSE is {m["cross_track_rmse_m"]:.4f} m and minimum clearance is {m["minimum_hull_clearance_m"]:.4f} m under the assumed circular footprint. Completion coexists with {a["mean_nominal_coverage_percent"]:.2f}% nominal proximity coverage because obstacle bypasses replace portions of the original lines. The evidence supports this bounded simulation task, while the sonar approximation, startup velocity anomaly and limited disturbance campaign remain explicit limitations.')
    report.sub('8.3 Data and implementation availability')
    report.p('The accompanying project archive contains the implementation, configurations and original experiment logs. The primary, calm and stronger-current datasets are in results/current, results/calm and results/strong_current. The analysis scripts and source hashes support recomputation of the reported metrics. A separate recorded demonstration is provided as videos/ROV_Survey_Full_Run.mp4 with its own run logs.')

    report.page('References')
    references = [
        ('[1] Fossen, T. I. Marine Craft Model.', 'https://www.fossen.biz/html/marineCraftModel.html', 'General six-DOF marine model; coordinate signs are defined in Section 2.'),
        ('[2] McQueen, C., et al. bluerov2_gz, pinned source.', 'https://github.com/clydemcqueen/bluerov2_gz/tree/4451a5d1b15d204581d43ef7d93df4a0508eddbb', 'Model parameters and sensor baseline are taken from the local pinned SDF.'),
        ('[3] Gazebo Sim 8. Hydrodynamics system.', 'https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html', 'Fluid-relative drag and current configuration.'),
        ('[4] Gazebo Sim 8. OdometryPublisher implementation.', 'https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc', 'Odometry interface and body-frame twist; see the installed version for exact behavior.'),
        ('[5] Gazebo / ROS. ros_gz_bridge.', 'https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge', 'ROS/Gazebo message conversion; local mappings are in config/bridge.yaml.'),
        ('[6] Gazebo Sim 8. Thruster system.', 'https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Thruster.html', 'Force commands to the model thruster joints.'),
        ('[7] Blue Robotics. Ping Sonar Altimeter and Echosounder.', 'https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/', 'Single acoustic beam; product page lists 25° beamwidth (checked 21 October 2026).'),
        ('[8] ROS 2 Jazzy. Ubuntu Debian installation.', 'https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html', 'Official Ubuntu 24.04 setup; full project instructions are in docs/INSTALL.rst.'),
        ('[9] Gazebo Harmonic. Installing with ROS.', 'https://gazebosim.org/docs/harmonic/ros_installation/', 'Supported ROS/Gazebo pairing and ros-jazzy-ros-gz packages.'),
    ]
    for title,url,note in references:
        report.link(title,url)
        report.p(note,9)
    report.p('The local assignment PDF defines task scope. External documentation explains model and interface semantics; implementation and measured results are supported by the attached code and logs.',9)

    report.page('Appendix A. Implementation and evidence')
    report.p('The accompanying project archive, ROV_Assignment_Project.zip, contains the implementation, configurations and original experiment logs. Table 10 locates the main files within the extracted project directory.',9)
    report.table('Source and data attachment index.', ['File or directory', 'Responsibility / inspection target'], [
        ['rov_survey/node.py', 'SurveyNode; ROS callbacks, mission state, thrust publication, logging'],
        ['rov_survey/perception.py', 'SonarMapper; calibration, frame transform, fitting and persistent map'],
        ['rov_survey/planning.py', 'Planner; mission legs, A*, inflation, continuous segment checks'],
        ['rov_survey/control.py', 'RobustController; feedback, B_5 extraction, saturation and anti-windup'],
        ['config/mission.json', 'Online settings; contains no true obstacle locations'],
        ['config/scenario*.json; config/bridge.yaml', 'World truth for 3 cases; exact Gazebo/ROS topic mappings'],
        ['scripts/prepare_world.py', 'Model changes and generated worlds; results/current/world.sdf is the primary world'],
        ['scripts/run.sh; scripts/run_simulation.py', 'ROS setup; process isolation; bridge/node startup; environment and hashes'],
        ['scripts/evaluate.py; scripts/check_results.py', 'Offline metrics and independent consistency/acceptance checks'],
        ['tests/test_algorithms.py', 'Seven named algorithm tests, detailed on the next page'],
        ['results/{current,calm,strong_current}/', 'telemetry.csv; scans/plans/events.jsonl; status, metrics and manifest JSON; evaluation.csv; SDF; process logs'],
        ['scripts/report_evidence.py; docs/figures/', 'Derived coverage/allocation evidence; vector PDF and 300 dpi PNG figures'],
        ['results/report_evidence/', 'analysis.json; per-line sample CSVs; scene capture; verification outputs'],
        ['package.xml; setup.py; requirements.txt', 'ROS package metadata, console entry point and Python requirements'],
        ['vendor/bluerov2_gz/; THIRD_PARTY.rst', 'Pinned upstream model/assets, retained Git revision and attribution'],
        ['docs/INSTALL.rst; README.md', 'Clean-machine setup, optional colcon build, execution and evaluation'],
    ], [.47,.53],7.7)
    report.p('results/development_* are development records and are excluded from the reported experiment set. Legacy metric names containing conservative or hull retain their original spelling for auditability; the physical interpretation in Section 6.2 applies.',9)

    report.page('Appendix A.1 Execution logic and tests')
    report.sub('A.1.1 Condensed implementation logic')
    report.code('on_scan(scan):\n    save raw ranges; find nearest buffered pose\n    if time offset <= 0.10 s:\n        filter and bias-correct rays; transform to world\n        fit/associate static cylinders; record new tracks\n\non_odometry(state):\n    save pose in history; run control at about 25 Hz\n    check sensor age, depth/attitude limits and deadline\n    advance ordered leg only when endpoint tests pass\n    if map invalidates route or new leg: plan and log A*\n    compute lookahead velocity and heading request\n    apply near-return hold; compute six clipped thrusts\n    publish Float64 forces; record telemetry and status',7.9)
    report.p('This pseudocode summarizes the actual callbacks; the full file includes watchdog handling and cleanup. A scan timeout keeps a zero horizontal request, while an odometry watchdog commands zero thrust. Neither is described as instantaneous stopping.',9)
    names=[
        ('test_sparse_scan_detects_cylinder_with_bias_and_mount_offset','Bias, mount and known-radius fit'),
        ('test_invalid_scan_and_pitch_rejection','Invalid returns and off-depth points'),
        ('test_map_rotation_and_persistence_without_returns','Coordinate rotation and track persistence'),
        ('test_online_planner_detours_and_rejoins_line_continuously','Detour, rejoin and continuous clearance'),
        ('test_blocked_endpoint_does_not_return_an_unsafe_route','Explicit blocked-goal behavior'),
        ('test_empty_map_and_exact_boustrophedon_order','Empty-map route and endpoint order'),
        ('test_allocation_reconstructs_wrench_and_bounds_saturation','Allocation rank, reconstruction, limits and anti-windup'),
    ]
    report.table('Algorithm checks in tests/test_algorithms.py.', ['Pytest name', 'Behavior checked'],names,[.65,.35],7.5)
    report.p('On 21 October 2026, all seven tests and all 36 saved-run checks passed. The manifests link the delivered algorithm files to the original experiments. Appendix B locates the commands and outputs; these checks do not validate untested physical assumptions.',9)

    report.page('Appendix B. Reproduction and verification')
    report.p('docs/INSTALL.rst gives clean Ubuntu 24.04 package/repository setup using the official ROS and Gazebo instructions [8,9], dependencies and optional colcon build. The current workstation is the tested environment; installation on a separate clean machine has not been independently validated.',9)
    report.sub('B.1 Existing workstation: execute and evaluate a new run')
    report.code('cd /home/yecheng/ROV_Assignment\nsource /opt/ros/jazzy/setup.bash\npython3 -m pytest -v\nbash scripts/run.sh --output results/repeat_current\npython3 scripts/evaluate.py results/repeat_current\nbash scripts/run.sh --scenario config/scenario_calm.json \\\n  --output results/repeat_calm\nbash scripts/run.sh --scenario config/scenario_strong_current.json \\\n  --output results/repeat_strong',7.8)
    report.p('The launcher sets GZ_SIM_RESOURCE_PATH to vendor/bluerov2_gz/models, a unique GZ_PARTITION, ROS_DOMAIN_ID=83 and local discovery. GZ_HOMEDIR and ROS_LOG_DIR stay inside the run directory. Run one experiment at a time. The model needs no ArduSub/SITL or custom C++ plugin; --gui adds the graphical client.',9)
    report.sub('B.2 Audit the supplied experiments and rebuild this report')
    report.code('python3 scripts/check_results.py \\\n  results/current results/calm results/strong_current\npython3 scripts/build_report.py \\\n  --runs results/current results/calm results/strong_current\ncolcon build --symlink-install --packages-select rov_survey',7.8)
    report.p('Original metrics can be recomputed with evaluate.py; the report generator independently derives the extra coverage and allocation evidence. It verifies that replayed controller forces match every recorded command. Tests and audit output are saved under results/report_evidence/; results/validation.json gives the machine-readable acceptance checks.',9)
    report.p('Figure 1 can be recaptured with scripts/capture_report_scene.py after sourcing ROS. Other figures are rendered directly from logs as vector PDF plus 300 dpi PNG. Rebuilding the report changes docs/report.pdf, docs/report.rst, docs/figures/, results/summary.json and derived report_evidence files; it does not rerun the mission or overwrite its logs.',9)
    report.finish()
    (ROOT/'results/summary.json').write_text(json.dumps({str(p.relative_to(ROOT)):r['metrics'] for p,r in zip(paths,runs)},indent=2)+'\n')
    print(f'{output / "report.pdf"} ({report.page_number} pages)')


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--runs',nargs='+',type=Path,default=[ROOT/'results/current',ROOT/'results/calm',ROOT/'results/strong_current'])
    build(parser.parse_args().runs)
