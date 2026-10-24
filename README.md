# Master and three followers: moving turns

Double-click **Start Simulation.cmd**, or open **worlds/four_robots_multi_angle.wbt**
in Webots and press Play. The camera follows the group. Reset the simulation to
replay the route. Each robot leaves a deliberately faint, muted trail showing its actual
travelled path; resetting clears the trails. Webots R2025a and Python 3.9 or later are required; no pip
packages, MCP server, or internet assets are needed.

At the end of a run, **runtime/paths.svg** contains a snapshot of the fading
trails, and **runtime/paths.json** retains the complete measured route and
sample timestamps. After 12 seconds of stillness the visible trails disappear.
Verification copies save these files under **runtime/verification/nominal/**.
The trails are single-pixel display geometry with no collision objects. Recent
sections use a restrained 24% peak opacity and a smooth cubic fade to complete
transparency over 12 seconds. Old sections reach zero opacity before their
slots are reused, avoiding an abrupt cutoff. The floor grid stays at 94%
transparency.

The indoor lab includes low perimeter walls, an electronics workbench, an assembly bench, metal shelving with parts boxes, and a packing counter. Furniture is static and stays outside the measured driving route. The floor and props use only local geometry; no downloads are required.

## How this simulation works

The green R1 robot is the master. Only its controller imports the manoeuvre plan.
After a stationary calibration period, it drives continuously through turns of
+30, +45, -60, +90, -120, +135, +180 and -90 degrees. Positive means left /
counterclockwise. The master commands 0.13 m/s during both straight segments and
curves; angular speed ramps smoothly. It stops at the end of the route.

The master estimates its own motion from wheel encoders and a calibrated noisy
Z-axis gyroscope, then broadcasts its measured position change, heading, linear
speed and angular rate. It does **not** broadcast future heading targets,
turn angles, or waypoints. Followers do not import the master's route.

Each follower measures its own motion with its own gyro and wheel encoders.
It tracks a translated copy of the master's measured trajectory using the
received packets. Scalar radio-distance estimates provide a weak spacing
correction along the direction estimated from known starting geometry and
odometry. Neither GPS, absolute yaw, radio bearing, nor supervisor robot poses
are available to the robot controllers.

The supervisor receives telemetry, displays labels, and independently measures
the physical bodies. It has no transmitter and never moves the robots.

## Communication triangles

Two thin, muted outlines follow the measured robot positions, matching the
reference layout:

- Red triangle: green master R1, orange R3, purple R4.
- Green triangle: green master R1, blue R2, purple R4.
- Both outlines share the master-to-purple diagonal; it is drawn with a tiny
  separation so both colours remain distinguishable.

Every follower has a master link and at least one follower link. Purple bridges
both triangles and therefore exchanges peer messages with both orange and blue.
The master receives follower status and acknowledges the latest timestamps.
Followers exchange actual simulated telemetry on radio channel 44, with delay
and packet loss. The peer telemetry does not change the motor tracking law.

Link visibility follows message freshness at both endpoints, fading gently
when traffic becomes stale. The observer draws the lines but transmits no
control messages and never moves a robot.

## What formation is maintained

The robots start aligned, in a square with 0.80 m neighbouring centre spacing.
The starting relative positions and heading alignment are known to the
controllers. The square keeps its orientation relative to the arena while
the four robots drive corresponding curves and change their headings.

This demonstrates formation translation and master-motion tracking. It does
not demonstrate rotating all follower positions around the master, or finding
an unknown initial formation from scalar distances alone.

## Radio and sensor assumptions

- Master messages: 20 Hz.
- Added delivery delay: 80 ms.
- Independently dropped packets: 8%, using repeatable random seeds.
- A follower brakes if the most recent accepted master measurement is older
  than 300 ms, then resumes tracking when fresh packets return.
- Encoder model: 720 counts per wheel revolution.
- Gyro model: a small per-robot constant bias plus 0.0008 rad/s standard
  deviation white noise; bias is estimated during the first stationary second.
- Startup requires the robots to face the same direction.
- Range is an inverse-signal-strength Webots radio approximation with noise.
  It is not an implementation of Bluetooth Channel Sounding or UWB timing.

These are explicit simulation assumptions, not measured specifications of the
chosen hardware. There is no ongoing absolute heading correction; real gyro
drift, wheel slip and longer runs need additional validation.

## Stability

The physical chassis uses a 35 mm centre of mass, a 240 mm wheel track,
front/rear rolling supports, limited torque, and gradual linear/angular
acceleration. Gravity and contact physics remain active. No robot position,
orientation, or body velocity is imposed by the supervisor.

## Repeat the checks

Double-click **Check Simulation.cmd**, or run:

```text
python run_simulation.py --check
```

This checks three scenarios in separate project copies under
**runtime/check_projects/**, keeping verification separate from a running demo:

1. **nominal**: all four robots keep moving through all eight curves.
2. **radio_loss**: the blue follower loses master messages for 1.2 seconds during
   the first curve, brakes on stale data, and recovers.
3. **stationary**: the master turns 90 degrees in place and followers learn
   that turn through radio despite unchanged inter-robot distances.

Results, full measurements and logs are under **runtime/verification/**.
A nonzero exit code means a failed check. The checks also require all five
unique links in the two reference triangles to be fresh at completion.
Each scenario has a four-minute
wall-clock timeout.

To run one scenario, or capture rendered turn images:

```text
python run_simulation.py --check --scenario nominal --screenshots
python run_simulation.py --scenario radio_loss
python run_simulation.py --scenario stationary
```

Normal launching uses the moving-turn scenario. The full demonstration lasts about 98 simulated seconds including startup
calibration and the final observation period.
Rendered screenshot checks observe the final stop for 15 seconds so they can
capture the trails before and after fading. Ordinary checks observe it for
seven seconds. The final stop and deliberate radio-failure braking are the only planned stops
after calibration.

## Files

- **controllers/master_robot/master_robot.py**: master-only motion plan execution and broadcasts.
- **controllers/stable_robot/stable_robot.py**: follower radio processing and local tracking.
- **robot_io.py**: sensor model, gyro calibration, encoder odometry and motors.
- **environment.py**: indoor lab walls, furniture and storage geometry.
- **path_trails.py**: smoothly fading measured trails and SVG/JSON exports.
- **communication_visuals.py**: live reference triangles and link freshness display.
- **robot_config.py**: geometry, start offsets and communication parameters.
- **mission.py**: master manoeuvre sequence, also read by the observer for validation.
- **controllers/demo_supervisor/demo_supervisor.py**: measurement and display only.
- **protos/StableRobot.proto** and **worlds/four_robots_multi_angle.wbt**: local physical model and world.
- **build_world.py**: regenerates the prototype and world after geometry changes.
- **run_simulation.py**: launcher and isolated verification.

After changing initial geometry, run `python build_world.py` to regenerate the
world. For a different Webots installation, use `--webots PATH` or set
WEBOTS_HOME / WEBOTS_EXECUTABLE.

All sources and outputs stay inside **New simulation**. The existing
**Simulations** directory is not used at runtime and remains unchanged.

