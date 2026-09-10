# GAMAC

A PX4 + ROS2 + Gz harmonic + RViz2 simulation pipeline: a drone with a dual camera setup (front and down) + lidar (to be used in future) combo autonomously detects the field boundary, calculates the centroid, and then after that goes up to the centroid to hover over it from distance of 30m

## Stack
- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.18.0-beta1
- Micro-XRCE-DDS Agent used for PX4 <-> ROS 2 bridge
- QGroundC

## Structure
- `drone_sim/` - ROS 2 package (nodes, world, launch files)
- `px4_customizations/` - custom PX4 Gazebo models (dual camera +
  downward lidar) and the airframe registration needed to spawn them


## Pipeline nodes (drone_sim/scripts/)
- **generate_world.py** - generates the Gazebo world file
  (a colored checkerboard field on a plain ground plane). Grid size,
  tile size, and color palette are parameters at the top of the file.
- **teleop_offboard.py** - keyboard-controlled manual flight
  (WASD move, I/K up/down, J/L yaw, Q quit). Includes an automatic
  ~3s climb after arming so PX4 registers takeoff before handing
  control to the keyboard. Press **C** to switch from manual flight
  into autonomous position-hold over the computed field centroid.
- **offboard_control.py** - simple fixed-position offboard hold, used
  for early pipeline testing.
- **boundary_detector.py** - subscribes to the downward camera feed,
  uses OpenCV HSV saturation thresholding to separate the colored
  field tiles from the plain floor, and publishes detected boundary
  points.
- **coord_transformer.py** - subscribes to PX4 odometry
  (`/fmu/out/vehicle_odometry`, BEST_EFFORT QoS) and projects the
  detected 2D pixel boundary points into 3D world coordinates.
- **field_map_builder.py** - accumulates the 3D boundary points over
  a flight, streams live RViz markers, and on shutdown (Ctrl+C)
  computes the field centroid and saves the full map to
  `field_map.json`.

## Custom PX4 models (px4_customizations/)
Two Gazebo model variants, both extending PX4's stock `x500`:
- **x500_dual_cam** — adds a forward-facing and a downward-facing
  camera (camera-only, no lidar).
- **x500_dual_cam_lidar** — same as above, plus a downward-scanning
  `gpu_lidar` (64x48 sample grid, ~45 deg horizontal x 30 deg
  vertical FOV, 0.1-30m range) for a real downward point cloud.

Both are registered under one custom PX4 airframe number (22000,
inside PX4's reserved custom-model range); which model actually
spawns is chosen at launch time via the `PX4_SIM_MODEL` environment
variable — no PX4 rebuild needed to switch between them.

## Setup from a clean machine

### 1. PX4-Autopilot
Set up a dedicated Python venv for PX4's build dependencies:


### 2. Add this repo's custom PX4 assets
cp -r px4_customizations/models/x500_dual_cam <PX4-Autopilot>/Tools/simulation/gz/models/
cp -r px4_customizations/models/x500_dual_cam_lidar <PX4-Autopilot>/Tools/simulation/gz/models/
cp px4_customizations/airframes/22000_gz_x500_dual_cam <PX4-Autopilot>/ROMFS/px4fmu_common/init.d-posix/airframes/

### 3. Micro-XRCE-DDS Agent
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git


### 4. QGroundControl
Required — PX4 blocks arming without a connected GCS. Download the
current AppImage from https://docs.qgroundcontrol.com, install the
Ubuntu prerequisites listed there (dialout group, libfuse2, etc.),
and make it executable.

### 5. ROS 2 workspace
(Match `px4_msgs`'s branch to your PX4 firmware version.)

## Running it

**Terminal 1 — PX4 + Gazebo (direct binary launch, not `make`):**
(`make px4_sitl gz_x500` hardcodes its own model selection and will
ignore custom models — this direct launch is required.)

**Terminal 2 — DDS Agent:**

**Terminal 3 — QGroundControl:**

**Terminal 4 — RViz2** (to watch the field map build live):
Set the Fixed Frame to match your odometry frame, then add a
**Marker** display subscribed to `/map/field_markers`.

**Terminal 5 — fly it:**
Fly manually with WASD/I/K/J/L around the field boundary, then press
**C** to switch to autonomous mode: the drone computes and hovers
over the field centroid, and `field_map.json` is saved.

## Status
End-to-end baseline verified: manual boundary survey -> boundary
detection -> 3D coordinate projection -> centroid computation ->
autonomous hover at commanded altitude over the field center.
