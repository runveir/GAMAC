# GAMAC

A PX4 + ROS2 + Gz harmonic + RViz2 simulation pipeline: a drone with a dual camera setup (front and down) + lidar (to be used in future) combo autonomously detects the field boundary, computer calculates the centroid, and then after that goes up to the centroid to hover over it.

## Stack
- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.18.0-beta1
- Micro-XRCE-DDS Agent used for PX4 <-> ROS 2 bridge

## Structure
- `drone_sim/` - ROS 2 package (nodes, world, launch files)
- `px4_customizations/` - custom PX4 Gazebo models (dual camera +
  downward lidar) and the airframe registration needed to spawn them

## Pipeline nodes (drone_sim/scripts/)
- `boundary_detector.py` - OpenCV HSV filtering on the downward
  camera feed to detect the field boundary
- `coord_transformer.py` - projects detected 2D pixel boundary points
  into 3D world coordinates using PX4 odometry
- `field_map_builder.py` - accumulates boundary points, publishes
  RViz markers, computes and saves the field centroid to
  `field_map.json`
- `teleop_offboard.py` - keyboard-controlled manual flight (WASD),
  switches to autonomous position-hold over the field centroid on
  `C`
- `offboard_control.py` - simple fixed-position offboard hold
- `generate_world.py` - generates the Gazebo world (checkerboard
  field) programmatically

## Setup
1. Clone PX4-Autopilot (v1.18.0-beta1) and follow standard PX4 +
   Gazebo Harmonic SITL setup.
2. Copy `px4_customizations/models/*` into
   `PX4-Autopilot/Tools/simulation/gz/models/`.
3. Copy `px4_customizations/airframes/22000_gz_x500_dual_cam` into
   `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`, and
   add `22000_gz_x500_dual_cam` to the `px4_add_romfs_files(...)`
   list in that folder's `CMakeLists.txt`.
4. Rebuild PX4 (`make px4_sitl`) so the new airframe is picked up.
5. Copy `drone_sim/` into a ROS 2 workspace's `src/` and
   `colcon build`.
6. Launch PX4 directly (not via the default `make` target) with
   `PX4_SIM_MODEL=x500_dual_cam_lidar` and
   `PX4_SYS_AUTOSTART=22000` set-see project notes for the full
   launch sequence.
7. Launch RViz2 to visualize the live field map as it builds: 
   Set the Fixed Frame (top-left panel) to match your odometry frame
   (e.g. `map` or `odom`), then add a **Marker** display subscribed to
   `/map/field_markers` to see boundary points appear live as
   `field_map_builder.py` accumulates them during flight.

## Status
End-to-end baseline verified: manual boundary survey -> boundary
detection -> 3D projection -> centroid computation -> autonomous
hover at commanded altitude over field center.
