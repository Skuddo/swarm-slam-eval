#!/usr/bin/env python3
import os
import sys
import subprocess
import yaml
import launch.logging
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, Shutdown, ExecuteProcess, GroupAction, TimerAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("graco_launch_setup")

    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    nav_mode = LaunchConfiguration('nav_mode').perform(context)
    update_time = float(LaunchConfiguration('update_time').perform(context))

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

    sequence_cfg = config[sequence]
    
    conf_max_robots = int(sequence_cfg.get("sequences", num_robots))
    if conf_max_robots < num_robots:
        logger.error(f"Num robots ({num_robots}) > max allowed ({conf_max_robots}). Shutting down.")
        return [Shutdown()]

    initial_coords = []
    for i in range(num_robots):
        try:
            robot_bag_name = f"{config[sequence]['names']}{i+1}"
            robot_bag_path = os.path.join(dataset_path, robot_bag_name)
            ground_truth_topic_name = config[sequence]['topics']['ground_truth']['src']
            prescan_script_path = os.path.join(project_root, 'utils', 'get_first_pose.py')

            command = [ sys.executable, prescan_script_path,
                    '--bag-path', robot_bag_path,
                    '--topic', ground_truth_topic_name 
                    ]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            initial_x_str, initial_y_str = result.stdout.strip().split(',')
            initial_x = float(initial_x_str)
            initial_y = float(initial_y_str)
            initial_coords.append((initial_x, initial_y))
            logger.info(f"Pre-scan successful. Got initial coords: x={initial_x}, y={initial_y}")
        except (subprocess.CalledProcessError, ValueError, FileNotFoundError, KeyError) as e:
            logger.error(f"Failed to get initial coordinates from bag: {e}. Defaulting to (0,0).")
            initial_coords.append((0.0, 0.0))

    base_x, base_y = initial_coords[0]
    offsets = [ (x - base_x, y - base_y) for (x, y) in initial_coords ]
    logger.info(f"Computed offsets (relative to robot 1): {offsets}")

    try:
        config_script_path = os.path.join(project_root, 'utils', 'config_generator.py')
        rviz_config_file = os.path.join(pkg_share, 'config', f"generated_{num_robots}_robots.rviz")
        command = [
            sys.executable, config_script_path,
            '--num-robots', str(num_robots),
            '--nav-mode', str(nav_mode),
            '--output-path', rviz_config_file
        ]
        subprocess.run(command, check=True)
        logger.info(f"Successfully created RViz config at {rviz_config_file}")
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        logger.error(f"Failed to generate RViz config file via script: {e}")
        return [Shutdown()]

    results_base_path = os.path.join(project_root, 'results')
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_folder_name = f"{timestamp_str}_{dataset}_{num_robots}robots_{nav_mode}"
    run_specific_path = os.path.join(results_base_path, run_folder_name)

    actions_to_launch = []
    actions_to_launch.append(Node(
        package='swarm_slam_eval',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{
            'num_robots': num_robots,
            'use_sim_time': use_sim_time
            }]
    ))

    if nav_mode == 'cslam':
        cslam_launch_file = os.path.join(pkg_share, 'launch', 'cslam.launch.py')
        cslam_config_path = os.path.join(get_package_share_directory("swarm_slam_eval"), "config", "cslam_graco_lidar.yaml")

        # Launch all external cslam nodes
        cslam_system = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cslam_launch_file),
            launch_arguments={
                'num_robots': str(num_robots),
                'use_sim_time': str(use_sim_time).lower(),
                'cslam_config_file': cslam_config_path,
                'dataset': dataset
            }.items()
        )
        
        # Global cslam pose graph saver node
        global_graph_saver = Node(
            package='swarm_slam_eval',
            executable='pose_graph_node',
            name='cslam_global_graph_saver',
            output='screen',
            parameters=[{
                'results_base_path': run_specific_path,
                'graph_name': 'cslam_global',
                'use_sim_time': use_sim_time,
                'update_time': update_time,
            }]
        )

        actions_to_launch.append(cslam_system)
        actions_to_launch.append(global_graph_saver)

    for i in range(num_robots):
        bag_path = os.path.join(dataset_path, f'{sequence_cfg["names"]}{i+1}')

        per_robot_nodes = [
            # data source (bag) node
            Node(
                package='swarm_slam_eval',
                executable='bag_reader_node',
                name='bag_reader',
                parameters=[{
                    'bag_path': bag_path,
                    'use_sim_time': use_sim_time,
                    'update_time':  update_time,
                    'is_clock_publisher': (i == 0)
                }],
            ),
            # ground truth pose graph saver node
            Node(
                package='swarm_slam_eval',
                executable='pose_graph_node',
                name='ground_truth_graph_saver',
                parameters=[{
                    'results_base_path': run_specific_path,
                    'graph_name': 'ground_truth',
                    'use_sim_time': use_sim_time,
                    'update_time' : update_time,
                }]
            ),
            # ground truth pose publisher node
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
                    }],
            )
        ]

        if nav_mode == 'cslam':
            # cslam pose graph local saver node
            per_robot_nodes.append(
                Node(
                    package='swarm_slam_eval',
                    executable='pose_graph_node',
                    name='cslam_graph_saver',
                    parameters=[{
                        'results_base_path': run_specific_path,
                        'graph_name': 'cslam',
                        'use_sim_time': use_sim_time,
                        'update_time' : update_time,
                    }]
                )
            )
            # cslam adapter node
            per_robot_nodes.append(
                Node(
                    package='swarm_slam_eval',
                    executable='cslam_node',
                    name='cslam',
                    parameters=[{'use_sim_time': use_sim_time}],
                )
            )

        robot_group = GroupAction([
            PushRosNamespace(f'r{i}'),
            *per_robot_nodes
        ])

        robot_group = TimerAction(
            period=5.0,
            actions = [
                robot_group
            ]
        )
        actions_to_launch.append(robot_group)

    # Visualizer node
    actions_to_launch.append(Node(
        package='swarm_slam_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        parameters=[{
            'num_robots': num_robots,
            'use_sim_time': use_sim_time,
            'nav_mode': nav_mode,
            }],
        condition=IfCondition(LaunchConfiguration('visualize'))
    ))

    # RViz node
    actions_to_launch.append(ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file, '--ros-args', '-p', f'use_sim_time:={str(use_sim_time).lower()}'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize'))
    ))

    return actions_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='GrAco'),
        DeclareLaunchArgument('dataset_sequence', default_value='ground'),
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='Whether to launch visualization nodes'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (bag) time'),
        DeclareLaunchArgument('nav_mode', default_value='cslam',
                              description='What to use for pose calculation'),
        DeclareLaunchArgument('update_time', default_value='5.0',
                              description='How often (in seconds) the evaluator saves checkpoints'),
        OpaqueFunction(function=launch_setup)
    ])
