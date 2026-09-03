import os
import numpy as np
from PIL import Image

# Configuration
MAPS_DIR = "maps"
OUTPUT_DIR = "../PX4-Autopilot-ColAvoid/Tools/simulation/gz/worlds"
WALL_HEIGHT = 8.0
MAP_SIZE_METERS = 20.0 # -10 to +10
MAP_SIZE_PIXELS = 600
PIXEL_SCALE = MAP_SIZE_METERS / MAP_SIZE_PIXELS # ~0.033m per pixel
OFFSET_X = -10.0
OFFSET_Y = 10.0

def generate_sdf(map_path, world_name):
    img = Image.open(map_path).convert('L')
    pixels = np.array(img)
    rows, cols = pixels.shape
    
    sdf_content = f"""<?xml version="1.0" encoding="UTF-8"?>
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
      <!-- 3D scene -->
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
      <!-- Plugins that add functionality to the scene -->
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <gz-gui>
          <property key="state" type="string">floating</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
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
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <gz-gui>
          <anchors target="Select entities">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="Spawn" name="Spawn Entities">
        <gz-gui>
          <anchors target="Select entities">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
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
      <sky>
        <clouds>
          <speed>12</speed>
        </clouds>
      </sky>
      <grid>true</grid>
      <ambient>0.5 0.5 0.5 0.3</ambient>
      <background>0.3 0.3 0.3 0.3</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
            </friction>
            <bounce/>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode>
                <min_depth>0.005</min_depth>
                <kp>1e8</kp>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>400 400</size>
            </plane>
          </geometry>
          <material>
            <diffuse>0.6 1.0 0.25 0.5</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0 0 0 0</emissive>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
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
    <light type="directional" name="sun_2">
      <cast_shadows>true</cast_shadows>
      <pose>200 200 200 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 0.3</diffuse>
      <specular>0.2 0.2 0.2 0.3</specular>
      <attenuation>
        <range>2000</range>
        <constant>1</constant>
        <linear>0</linear>
        <quadratic>0</quadratic>
      </attenuation>
      <direction>0.01 0.01 -0.9</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
"""

    sdf_content += """
    <model name="maze_walls">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
"""

    wall_count = 0
    work_grid = pixels.copy()
    
    for r in range(rows):
        for c in range(cols):
            if work_grid[r, c] < 50:
                width = 0
                while (c + width < cols) and (work_grid[r, c + width] < 50):
                    width += 1
                
                height = 0
                while (r + height < rows):
                    is_wall_row = True
                    for k in range(width):
                        if work_grid[r + height, c + k] >= 50:
                            is_wall_row = False
                            break
                    if not is_wall_row:
                        break
                    height += 1
                
                for i in range(height):
                    for j in range(width):
                        work_grid[r + i, c + j] = 255

                center_r_idx = r + (height / 2.0)
                center_c_idx = c + (width / 2.0)

                pos_x = 10.0 - (center_r_idx * PIXEL_SCALE)
                pos_y = -10.0 + (center_c_idx * PIXEL_SCALE)
                pos_z = WALL_HEIGHT / 2.0
                
                size_x = height * PIXEL_SCALE
                size_y = width * PIXEL_SCALE
                
                sdf_content += f"""
        <collision name="c_{wall_count}">
          <pose>{pos_x} {pos_y} {pos_z} 0 0 0</pose>
          <geometry>
            <box><size>{size_x} {size_y} {WALL_HEIGHT}</size></box> 
          </geometry> 
        </collision>
        <visual name="v_{wall_count}">
          <pose>{pos_x} {pos_y} {pos_z} 0 0 0</pose>
          <geometry>
            <box><size>{size_x} {size_y} {WALL_HEIGHT}</size></box>
          </geometry>
          <material>
            <diffuse>1.0 1.0 1.0 1.0</diffuse>
            <specular>0.2 0.2 0.2 1.0</specular>
            <pbr>
              <metal>
                <albedo_map>model://brick_wall_3/materials/textures/brick_antique_1.jpg</albedo_map>
                <roughness>0.8</roughness>
                <metalness>0.01</metalness>
              </metal>
            </pbr>
          </material>
        </visual>"""
                
                wall_count += 1

    sdf_content += """
      </link>
    </model>
"""
    
    sdf_content += "\n  </world>\n</sdf>"
    
    with open(f"{OUTPUT_DIR}/{world_name}.sdf", "w") as f:
        f.write(sdf_content)
    print(f"Generated {world_name}.sdf with {wall_count} walls.")

def main():
    if not os.path.exists(OUTPUT_DIR):
        print(f"Error: Output dir {OUTPUT_DIR} does not exist.")
        return

    maps = [f for f in os.listdir(MAPS_DIR) if f.endswith('.png')]
    print(f"Found {len(maps)} maps.")
    
    for m in maps:
        name = m.replace(".png", "").replace("_600x600px", "").replace("obstable", "obstacle")
        generate_sdf(os.path.join(MAPS_DIR, m), name)

if __name__ == "__main__":
    main()
