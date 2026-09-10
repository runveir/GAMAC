#!/usr/bin/env python3
"""
Generates worlds/drone_world.sdf with a colored checkerboard ground.
Re-run this script any time to regenerate the world (e.g. after
changing GRID_SIZE, TILE_SIZE, or COLORS below).
"""

import os

# ---- Configurable parameters ----
TILE_SIZE = 1.5          # meters, each tile is TILE_SIZE x TILE_SIZE
GRID_SIZE = 6            # tiles per side (6 * 1.5 = 9m x 9m)
COLORS = [
    (0.20, 0.55, 0.20, 1),  # medium green (grass)
    (0.35, 0.65, 0.15, 1),  # yellow-green (crop)
    (0.10, 0.35, 0.10, 1),  # dark green (dense foliage)
    (0.55, 0.70, 0.30, 1),  # pale olive-green (dry field)
]
OUTPUT_PATH = os.path.expanduser(
    "~/drone_ws/src/drone_sim/worlds/drone_world.sdf")


def tile_block(index, x, y, color):
    r, g, b, a = color
    return f"""
    <model name="tile_{index}">
      <static>true</static>
      <pose>{x} {y} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{TILE_SIZE} {TILE_SIZE} 0.002</size>
            </box>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def build_world():
    half = (GRID_SIZE * TILE_SIZE) / 2.0
    tiles = []
    idx = 0
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            x = -half + TILE_SIZE * i + TILE_SIZE / 2.0
            y = -half + TILE_SIZE * j + TILE_SIZE / 2.0
            color = COLORS[(i + j) % len(COLORS)]
            tiles.append(tile_block(idx, x, y, color))
            idx += 1
    tiles_str = "".join(tiles)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="drone_world">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Base ground plane: physics/collision only, tiles sit visually on top -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{tiles_str}

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""
    with open(OUTPUT_PATH, "w") as f:
        f.write(world)
    print(f"Wrote {OUTPUT_PATH} with {idx} tiles ({GRID_SIZE}x{GRID_SIZE}).")


if __name__ == "__main__":
    build_world()