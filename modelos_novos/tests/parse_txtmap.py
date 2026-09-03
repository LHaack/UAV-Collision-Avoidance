#!/usr/bin/env python3
"""
Parses a .txtmap file and generates:
  - mission.json (friendly drones)
  - enemies.json (enemy drones)
  - world SDF (walls)

Map format (20x20 chars = 20x20m area from -10 to +10):
  a-z : friendly drone spawn
  A-Z : friendly drone landing (A matches a, B matches b, etc.)
  0-9 : enemy drone spawn
  ^ ! @ $ % : enemy drone targets (for enemies 0-4)
  - | : walls
  . : empty space

Coordinate system:
  Column → Gazebo X: x = col - 9.5
  Row → Gazebo Y: y = 9.5 - row
"""
import json
import os
import sys

ENEMY_TARGET_SYMBOLS = {
    0: '^',
    1: '!',
    2: '@',
    3: '$',
    4: '%',
}

FLIGHT_ALTITUDE = 4.0
WALL_HEIGHT = 8.0
MAP_SIZE = 20


def parse_txtmap(filepath):
    with open(filepath) as f:
        lines = [line.rstrip('\n') for line in f.readlines()]

    while len(lines) < MAP_SIZE:
        lines.append('.' * MAP_SIZE)
    for i in range(len(lines)):
        lines[i] = lines[i].ljust(MAP_SIZE, '.')

    friendly_spawns = {}
    friendly_targets = {}
    enemy_spawns = {}
    enemy_targets = {}
    wall_cells = []

    for row in range(MAP_SIZE):
        for col in range(MAP_SIZE):
            ch = lines[row][col] if col < len(lines[row]) else '.'
            x = col - 9.5
            y = 9.5 - row

            if ch.islower():
                friendly_spawns[ch] = (x, y)
            elif ch.isupper():
                friendly_targets[ch.lower()] = (x, y)
            elif ch.isdigit():
                enemy_spawns[int(ch)] = (x, y)
            elif ch in ENEMY_TARGET_SYMBOLS.values():
                for eid, sym in ENEMY_TARGET_SYMBOLS.items():
                    if sym == ch:
                        enemy_targets[eid] = (x, y)
            elif ch in ('-', '|'):
                wall_cells.append((x, y))

    return friendly_spawns, friendly_targets, enemy_spawns, enemy_targets, wall_cells


def generate_mission_json(friendly_spawns, friendly_targets):
    """
    PX4 SITL coordinate mapping (from GZBridge.cpp):
      NED North (position[0]) = Gazebo Y
      NED East  (position[1]) = Gazebo X
    Mission format: go:N,E,D (NED relative to home)
    """
    drones = []
    for key in sorted(friendly_spawns.keys()):
        spawn = friendly_spawns[key]
        target = friendly_targets.get(key)
        if target is None:
            print(f"WARNING: no target for drone '{key}', skipping")
            continue

        delta_gz_x = target[0] - spawn[0]
        delta_gz_y = target[1] - spawn[1]

        delta_ned_n = delta_gz_y
        delta_ned_e = delta_gz_x

        import math as _math
        spawn_yaw = _math.atan2(delta_gz_y, delta_gz_x)

        drones.append({
            "spawn": [spawn[0], spawn[1], 0.0],
            "spawn_yaw": spawn_yaw,
            "mission": [f"go:{delta_ned_n},{delta_ned_e},-{FLIGHT_ALTITUDE}"]
        })
    return drones


def generate_enemies_json(enemy_spawns, enemy_targets):
    """
    Enemy target_pose is in NED relative to home.
    NED N = Gazebo Y delta, NED E = Gazebo X delta.
    """
    enemies = []
    for eid in sorted(enemy_spawns.keys()):
        spawn = enemy_spawns[eid]
        target = enemy_targets.get(eid)
        if target is None:
            print(f"WARNING: no target for enemy {eid}, skipping")
            continue

        delta_gz_x = target[0] - spawn[0]
        delta_gz_y = target[1] - spawn[1]
        dist = (delta_gz_x**2 + delta_gz_y**2) ** 0.5

        delta_ned_n = delta_gz_y
        delta_ned_e = delta_gz_x

        if dist > 0:
            far_ned_n = (delta_ned_n / dist) * 100
            far_ned_e = (delta_ned_e / dist) * 100
        else:
            far_ned_n, far_ned_e = 100, 0

        import math as _math
        spawn_yaw = _math.atan2(delta_gz_y, delta_gz_x)

        enemies.append({
            "spawn_pose": [spawn[0], spawn[1], 0.0],
            "spawn_yaw": spawn_yaw,
            "target_pose": [far_ned_n, far_ned_e, 0.0]
        })
    return enemies


def merge_walls(wall_cells):
    if not wall_cells:
        return []

    cell_set = set((c[0], c[1]) for c in wall_cells)
    visited = set()
    groups = []

    for cell in cell_set:
        if cell in visited:
            continue
        group = []
        stack = [cell]
        while stack:
            c = stack.pop()
            if c in visited or c not in cell_set:
                continue
            visited.add(c)
            group.append(c)
            x, y = c
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                stack.append((x+dx, y+dy))
        groups.append(group)

    walls = []
    for group in groups:
        xs = [c[0] for c in group]
        ys = [c[1] for c in group]
        min_x = min(xs) - 0.5
        max_x = max(xs) + 0.5
        min_y = min(ys) - 0.5
        max_y = max(ys) + 0.5
        walls.append({
            "center": ((min_x+max_x)/2, (min_y+max_y)/2, WALL_HEIGHT/2),
            "size": (max_x-min_x, max_y-min_y, WALL_HEIGHT)
        })

    return walls


def generate_world_sdf(walls, world_name):
    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::AirPressure" filename="gz-sim-air-pressure-system"/>
    <plugin name="gz::sim::systems::ApplyLinkWrench" filename="gz-sim-apply-link-wrench-system"/>
    <plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gui fullscreen="false">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>0 -15 4 0 0 1.5708</camera_pose>
        <camera_clip>
          <near>0.25</near>
          <far>25000</far>
        </camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin name="World control" filename="WorldControl">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">0</property>
          <property type="bool" key="resizable">0</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>1</play_pause>
        <step>1</step>
        <start_paused>1</start_paused>
      </plugin>
      <plugin name="World stats" filename="WorldStats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">0</property>
          <property type="bool" key="resizable">0</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>1</sim_time>
        <real_time>1</real_time>
        <real_time_factor>1</real_time_factor>
        <iterations>1</iterations>
      </plugin>
      <plugin name="Entity tree" filename="EntityTree"/>
    </gui>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <sky><clouds><speed>12</speed></clouds></sky>
      <grid>true</grid>
      <ambient>0.5 0.5 0.5 0.3</ambient>
      <background>0.3 0.3 0.3 0.3</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <material><diffuse>0.6 1.0 0.25 0.5</diffuse></material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
"""

    if walls:
        sdf += '    <model name="walls"><static>true</static><link name="link">\n'
        for i, wall in enumerate(walls):
            cx, cy, cz = wall["center"]
            sx, sy, sz = wall["size"]
            sdf += f"""      <collision name="c_{i}">
        <pose>{cx} {cy} {cz} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="v_{i}">
        <pose>{cx} {cy} {cz} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <diffuse>0.75 0.4 0.3 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://brick_wall/materials/textures/brick_antique_1.jpg</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
"""
        sdf += '    </link></model>\n'

    sdf += """  </world>
</sdf>
"""
    return sdf


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 parse_txtmap.py <file.txtmap> [output_dir]")
        sys.exit(1)

    txtmap_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(txtmap_path) or "."

    map_name = os.path.splitext(os.path.basename(txtmap_path))[0]

    friendly_spawns, friendly_targets, enemy_spawns, enemy_targets, wall_cells = parse_txtmap(txtmap_path)

    print(f"Parsed {txtmap_path}:")
    print(f"  Friendly drones: {len(friendly_spawns)} ({', '.join(sorted(friendly_spawns.keys()))})")
    print(f"  Enemy drones: {len(enemy_spawns)} ({', '.join(str(k) for k in sorted(enemy_spawns.keys()))})")
    print(f"  Wall cells: {len(wall_cells)}")

    mission = generate_mission_json(friendly_spawns, friendly_targets)
    mission_path = os.path.join(output_dir, f"{map_name}_mission.json")
    with open(mission_path, 'w') as f:
        json.dump(mission, f, indent=4)
    print(f"  Written: {mission_path}")

    enemies = generate_enemies_json(enemy_spawns, enemy_targets)
    enemies_path = os.path.join(output_dir, f"{map_name}_enemies.json")
    with open(enemies_path, 'w') as f:
        json.dump(enemies, f, indent=4)
    print(f"  Written: {enemies_path}")

    walls = merge_walls(wall_cells)
    world_sdf = generate_world_sdf(walls, map_name)
    world_path = os.path.join(output_dir, f"{map_name}.sdf")
    with open(world_path, 'w') as f:
        f.write(world_sdf)
    print(f"  Written: {world_path}")

    print(f"\nFriendly drones:")
    for key in sorted(friendly_spawns.keys()):
        s = friendly_spawns[key]
        t = friendly_targets.get(key, ('?','?'))
        print(f"  {key}: spawn ({s[0]}, {s[1]}) → target ({t[0]}, {t[1]})")

    print(f"\nEnemy drones:")
    for eid in sorted(enemy_spawns.keys()):
        s = enemy_spawns[eid]
        t = enemy_targets.get(eid, ('?','?'))
        print(f"  {eid}: spawn ({s[0]}, {s[1]}) → direction ({t[0]}, {t[1]})")

    print(f"\nWalls:")
    for w in walls:
        print(f"  center=({w['center'][0]}, {w['center'][1]}, {w['center'][2]}) size=({w['size'][0]}, {w['size'][1]}, {w['size'][2]})")


if __name__ == "__main__":
    main()
