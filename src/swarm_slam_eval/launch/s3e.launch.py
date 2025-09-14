import os
import sys
import subprocess
import yaml
import launch.logging
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, Shutdown, ExecuteProcess,
                            GroupAction, TimerAction, IncludeLaunchDescription, PushLaunchConfigurations,
                            PopLaunchConfigurations)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

ROBOT_NAME_MAP = ["Alpha", "Bob", "Carol"]

def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("s3e_launch")

    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    visualize = LaunchConfiguration('visualize').perform(context) == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    nav_mode = LaunchConfiguration('nav_mode').perform(context)
    update_time = float(LaunchConfiguration('update_time').perform(context))
    namespace = LaunchConfiguration('namespace').perform(context)
    eval_start_delay = float(LaunchConfiguration('eval_start_delay').perform(context))

    try:
        pkg_share = get_package_share_directory('swarm_slam_eval')
        project_root = os.path.abspath(os.path.join(pkg_share, '../../../../../'))
        dataset_path = os.path.join(project_root, 'datasets', dataset)
        config_path = os.path.join(dataset_path, "config.yaml")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        logger.error(f"Could not load dataset config at '{config_path}': {e}")
        return [Shutdown()]

    bag_path = os.path.join(dataset_path, f"S3E_{sequence}")
    robot_prefixes = config.get('robot_prefixes', ROBOT_NAME_MAP)

    if len(robot_prefixes) < num_robots:
        logger.error("Not enough robot_prefixes in config for requested num_robots.")
        return [Shutdown()]

    initial_coords = []
    for i in range(num_robots):
        try:
            # Prescan using deterministic S3E topic mapping
            ground_truth_topic_name = config[sequence]['topics']['ground_truth']['src']
            prescan_script_path = os.path.join(project_root, 'utils', 'get_first_pose.py')
            command = [sys.executable, prescan_script_path, '--bag-path', bag_path, '--topic', ground_truth_topic_name]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            print(result)
            x_str, y_str = result.stdout.strip().split(',')
            initial_coords.append((float(x_str), float(y_str)))
            logger.info(f"Pre-scan successful for {robot_prefixes[i]}: x={x_str}, y={y_str}")
        except Exception as e:
            logger.error(f"Failed to get initial coordinates for {robot_prefixes[i]}: {e}. Defaulting to (0,0)")
            initial_coords.append((0.0, 0.0))

    base_x, base_y = initial_coords[0]
    offsets = [(x - base_x, y - base_y) for (x, y) in initial_coords]

    # --- RViz config ---
    try:
        config_script_path = os.path.join(project_root, 'utils', 'config_generator.py')
        rviz_config_file = os.path.join(pkg_share, 'config', f"generated_{num_robots}_robots.rviz")
        subprocess.run([
            sys.executable, config_script_path,
            '--num-robots', str(num_robots),
            '--nav-mode', nav_mode,
            '--output-path', rviz_config_file
        ], check=True)
        logger.info(f"RViz config generated at {rviz_config_file}")
    except Exception as e:
        logger.error(f"Failed to generate RViz config: {e}")
        return [Shutdown()]

    actions_to_launch = []

    # --- Sync node ---
    actions_to_launch.append(Node(
        package='swarm_slam_eval',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{'num_robots': num_robots, 'use_sim_time': use_sim_time}]
    ))

    # --- CSLAM launch + global graph saver ---
    if nav_mode == 'cslam':
        cslam_launch_file = os.path.join(pkg_share, 'launch', 'cslam.launch.py')
        cslam_config_path = os.path.join(pkg_share, 'config', 'cslam_s3e_lidar_stereo.yaml')
        actions_to_launch.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cslam_launch_file),
            launch_arguments={
                'num_robots': str(num_robots),
                'use_sim_time': str(use_sim_time).lower(),
                'cslam_config_file': cslam_config_path
            }.items()
        ))

        run_folder_name = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{dataset}_{num_robots}robots_{nav_mode}"
        results_path = os.path.join(project_root, 'results', run_folder_name)

        actions_to_launch.append(Node(
            package='swarm_slam_eval',
            executable='pose_graph_node',
            name='cslam_global_graph_saver',
            output='screen',
            parameters=[{
                'results_base_path': results_path,
                'graph_name': 'cslam_global',
                'use_sim_time': use_sim_time,
                'update_time': update_time
            }]
        ))

    # --- Per-robot nodes ---
    for i in range(num_robots):
        robot_ns = f"{namespace}{i}"
        robot_prefix = robot_prefixes[i]

        per_robot_nodes = [
            # Bag reader
            Node(
                package='swarm_slam_eval',
                executable='bag_reader_node',
                name='bag_reader',
                parameters=[{
                    'bag_path': bag_path,
                    'robot_prefix': robot_prefix,
                    'use_sim_time': use_sim_time,
                    'update_time': update_time,
                    'is_clock_publisher': (i == 0)
                }]
            ),
            # Ground truth graph saver
            Node(
                package='swarm_slam_eval',
                executable='pose_graph_node',
                name='ground_truth_graph_saver',
                parameters=[{
                    'results_base_path': results_path,
                    'graph_name': 'ground_truth',
                    'use_sim_time': use_sim_time,
                    'update_time': update_time
                }]
            ),
            # Ground truth node
            Node(
                package='swarm_slam_eval',
                executable='ground_truth_node',
                name='ground_truth',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'offset': True,
                    'initial_x': initial_coords[i][0],
                    'initial_y': initial_coords[i][1],
                    'offset_x': offsets[i][0],
                    'offset_y': offsets[i][1]
                }]
            )
        ]

        if nav_mode == 'cslam':
            per_robot_nodes.append(Node(
                package='swarm_slam_eval',
                executable='pose_graph_node',
                name='cslam_graph_saver',
                parameters=[{
                    'results_base_path': results_path,
                    'graph_name': 'cslam',
                    'use_sim_time': use_sim_time,
                    'update_time': update_time
                }]
            ))
            per_robot_nodes.append(Node(
                package='swarm_slam_eval',
                executable='cslam_node',
                name='cslam',
                parameters=[{'use_sim_time': use_sim_time}]
            ))

        # Wrap each robot in its own GroupAction + Timer
        robot_group = GroupAction([
            PushRosNamespace(robot_ns),
            *per_robot_nodes
        ])
        actions_to_launch.append(TimerAction(
            period=eval_start_delay,
            actions=[robot_group]
        ))

    # --- Visualization (RViz) ---
    if visualize:
        actions_to_launch.append(ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file, '--ros-args', '-p', f'use_sim_time:={str(use_sim_time).lower()}'],
            output='screen'
        ))

    return actions_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='S3E'),
        DeclareLaunchArgument('dataset_sequence', default_value='Playground_1'),
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='Whether to launch visualization nodes'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (bag) time'),
        DeclareLaunchArgument('nav_mode', default_value='cslam',
                              description='What to use for pose calculation'),
        DeclareLaunchArgument('update_time', default_value='5.0',
                              description='How often evaluator saves checkpoints'),
        DeclareLaunchArgument('namespace', default_value='/r', description='Robot namespace prefix'),
        DeclareLaunchArgument('eval_start_delay', default_value='5.0'),
        OpaqueFunction(function=launch_setup)
    ])