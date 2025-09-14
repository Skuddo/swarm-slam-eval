import os
import argparse

def generate_rviz_config(num_robots: int, output_path: str, nav_mode: str):
    rviz_content = f"""
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
Visualization Manager:
  Class: ""
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 50.0
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Name: Orbit (RViz)
      Pitch: 0.785398185
      Target Frame: world
      Yaw: 0.0
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Offset:
        X: 0.0
        Y: 0.0
        Z: -0.01
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
"""

    for i in range(num_robots):
        namespace = f"r{i}"
        rviz_content += f"""
    - Class: rviz_default_plugins/Marker
      Name: Robot {i} Ground Truth
      Topic:
        Value: /{namespace}/gt_vis_marker
      Enabled: true
      Queue Size: 100
"""
        rviz_content += f"""
    - Class: rviz_default_plugins/Marker
      Name: Robot {i} {nav_mode} Pose
      Topic:
        Value: /{namespace}/{nav_mode}_vis_marker
      Enabled: true
      Queue Size: 100
"""

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(rviz_content)
    print(f"Successfully generated RViz config at {output_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate an RViz config file for multi-robot visualization.')
    parser.add_argument(
      '--num-robots', type=int, required=True,
      help="How many robots"     
      )
    parser.add_argument(
      '--nav-mode', type=str, required=True,
      help="Which secondary pose source to use"     
      )
    parser.add_argument(
      '--output-path', type=str, required=True,
      help="Where the config to be used will be located"     
      )
    args = parser.parse_args()

    generate_rviz_config(args.num_robots, args.output_path, args.nav_mode)