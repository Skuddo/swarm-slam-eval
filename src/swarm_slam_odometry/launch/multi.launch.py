import os
import sys
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, Shutdown, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch.logging

# This is the main orchestrator function. It runs at launch time and sets everything up.
def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("launch_setup")
    actions_to_launch = []
    
    # --- 1. Resolve all launch arguments and paths ---
    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    sim_rate = float(LaunchConfiguration('sim_rate').perform(context))
    use_sim_time = bool(LaunchConfiguration('use_sim_time').perform(context))

    odometry_pkg_share = get_package_share_directory('swarm_slam_odometry')
    visualizer_pkg_share = get_package_share_directory('swarm_slam_visualizer')
    
    try:
        # This logic finds your project root based on the package location in the install space
        project_root = os.path.abspath(os.path.join(odometry_pkg_share, '../../../../../'))
        dataset_path = os.path.join(project_root, 'datasets', dataset)
        config_path = os.path.join(dataset_path, "config.yaml")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        logger.error(f"Could not load dataset config at '{config_path}': {e}")
        return [Shutdown()]

    # --- 2. Pre-scan the bag file to get initial coordinates ---
    try:
        robot1_bag_name = f"{config[sequence]['names']}1"
        robot1_bag_path = os.path.join(dataset_path, robot1_bag_name)
        ground_truth_topic_name = config[sequence]['topics']['ground_truth']['src']
        
        # Path to the script in your project's utils directory
        prescan_script_path = os.path.join(project_root, 'utils', 'get_first_pose.py')

        command = [ sys.executable, prescan_script_path, '--bag-path', robot1_bag_path, '--topic', ground_truth_topic_name ]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        initial_x_str, initial_y_str = result.stdout.strip().split(',')
        initial_x = float(initial_x_str)
        initial_y = float(initial_y_str)
        logger.info(f"Pre-scan successful. Got initial coords: x={initial_x}, y={initial_y}")
    except (subprocess.CalledProcessError, ValueError, FileNotFoundError, KeyError) as e:
        logger.error(f"Failed to get initial coordinates from bag: {e}. Defaulting to (0,0).")
        initial_x, initial_y = 0.0, 0.0

    # 2.5 Generate the rviz config
    try:
        config_script_path = os.path.join(project_root, 'utils', 'config_generator.py')
        rviz_config_file = os.path.join(odometry_pkg_share, 'config', f"generated_{num_robots}_robots.rviz")

        # Build the command to execute the script
        command = [
            sys.executable, config_script_path,
            '--num-robots', str(num_robots),
            '--initial-x', str(initial_x),
            '--initial-y', str(initial_y),
            '--output-path', rviz_config_file
        ]
        # Run the command
        subprocess.run(command, check=True)
        logger.info(f"Successfully ran RViz config generator script.")
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        logger.error(f"Failed to generate RViz config file via script: {e}")
        return [Shutdown()]

    # --- 3. Prepare all nodes and actions to be launched ---
    # A. Robot nodes (bag reader + odometry)
    conf_max_robots = config["ground"]["sequences"]
    if conf_max_robots < num_robots:
        logger.error(f"Num robots ({num_robots}) > max allowed ({conf_max_robots}). Shutting down.")
        return [Shutdown()]

    conf_dataset_names = config[sequence]["names"]
    for i in range(1, num_robots + 1):
        robot_n = f'r{i}'
        bag_path = os.path.join(dataset_path, f'{conf_dataset_names}{i}')
        is_master_clock = (i == 1)

        robot_group = GroupAction([
            PushRosNamespace(robot_n),
            Node(
                package='swarm_slam_odometry',
                executable='bag_reader_node',
                name='bag_reader',
                output='screen',
                parameters=[{
                    'bag_path': bag_path,
                    'sim_rate': sim_rate,
                    'use_sim_time': use_sim_time,
                    'is_clock_publisher': is_master_clock
                }],
            ),
            Node(
                package='swarm_slam_odometry',
                executable='odometry_node',
                name='odometry',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time
                }],
            )
        ])
        actions_to_launch.append(robot_group)

    # B. Sync node
    actions_to_launch.append(Node(
        package='swarm_slam_odometry',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{'num_robots': LaunchConfiguration('num_robots'), 'use_sim_time': use_sim_time}]
    ))

    # C. Visualizer node
    actions_to_launch.append(Node(
        package='swarm_slam_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        parameters=[{'num_robots': LaunchConfiguration('num_robots'), 'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('visualize')),
    ))

    # D. RViz node
    actions_to_launch.append(ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file, '--ros-args', '-p', f'use_sim_time:={use_sim_time}'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize'))
    ))

    return actions_to_launch


def generate_launch_description():
    return LaunchDescription([
        # Declare all launch arguments first
        DeclareLaunchArgument('dataset', default_value='GrAco'),
        DeclareLaunchArgument('dataset_sequence', default_value='ground'),
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('sim_rate', default_value='1.0'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='Whether to launch visualization nodes'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (bag) time'),
        
        # This single OpaqueFunction now orchestrates the entire launch process
        OpaqueFunction(function=launch_setup)
    ])