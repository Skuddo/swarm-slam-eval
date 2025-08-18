import os
import sys
import subprocess
import yaml
import launch.logging
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, Shutdown, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

nav_modes = {
    "imu": {
        
    },
    # "vins": {
        
    # }
}

# This is the main orchestrator function. It runs at launch time and sets everything up.
def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("launch_setup")
    
    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    sim_rate = float(LaunchConfiguration('sim_rate').perform(context))
    use_sim_time = bool(LaunchConfiguration('use_sim_time').perform(context))
    nav_mode = str(LaunchConfiguration('nav_mode').perform(context))
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

    # Check num robots param
    conf_max_robots = config["ground"]["sequences"]
    if conf_max_robots < num_robots:
        logger.error(f"Num robots ({num_robots}) > max allowed ({conf_max_robots}). Shutting down.")
        return [Shutdown()]
    
    # prepare list od odometry nodes based on launch config
    # ground truth should always be there, default: "imu"
    nav_modes = ['ground_truth']
    nav_modes.append(nav_mode)
    
    # Pre-scan the bag file to get initial coordinates ---
    try:
        robot1_bag_name = f"{config[sequence]['names']}1"
        robot1_bag_path = os.path.join(dataset_path, robot1_bag_name)
        ground_truth_topic_name = config[sequence]['topics']['ground_truth']['src']
        prescan_script_path = os.path.join(project_root, 'utils', 'get_first_pose.py')

        command = [ sys.executable, prescan_script_path,
                   '--bag-path', robot1_bag_path,
                   '--topic', ground_truth_topic_name 
                   ]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        initial_x_str, initial_y_str = result.stdout.strip().split(',')
        initial_x = float(initial_x_str)
        initial_y = float(initial_y_str)
        logger.info(f"Pre-scan successful. Got initial coords: x={initial_x}, y={initial_y}")
    except (subprocess.CalledProcessError, ValueError, FileNotFoundError, KeyError) as e:
        logger.error(f"Failed to get initial coordinates from bag: {e}. Defaulting to (0,0).")
        initial_x, initial_y = 0.0, 0.0

    # Generate the rviz config
    try:
        config_script_path = os.path.join(project_root, 'utils', 'config_generator.py')
        rviz_config_file = os.path.join(pkg_share, 'config', f"generated_{num_robots}_robots.rviz")

        command = [
            sys.executable, config_script_path,
            '--num-robots', str(num_robots),
            '--initial-x', str(initial_x),
            '--initial-y', str(initial_y),
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

    # Pack nodes
    actions_to_launch = []
    for i in range(1, num_robots + 1):
        bag_path = os.path.join(dataset_path, f'{config[sequence]["names"]}{i}')

        nav_nodes = []
        graph_nodes = []
        for mode in nav_modes:
            nav_node = Node(
                package='swarm_slam_eval',
                executable=f"{mode}_node",
                name=mode,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time
                }],
            )
            nav_nodes.append(nav_node)

            graph_node = Node(
                package='swarm_slam_eval',
                executable='pose_graph_node',
                name=f'{mode}_graph_saver',
                parameters=[{
                    'results_base_path': run_specific_path,
                    'graph_name': mode,
                    'dataset': dataset,
                    'dataset_sequence': sequence,
                    'use_sim_time': use_sim_time,
                    'update_time' : update_time
                }]
            )
            graph_nodes.append(graph_node)

        robot_group = GroupAction([
            PushRosNamespace(f'r{i}'),
            Node(
                package='swarm_slam_eval',
                executable='bag_reader_node',
                name='bag_reader',
                output='screen',
                parameters=[{
                    'bag_path': bag_path,
                    'sim_rate': sim_rate,
                    'use_sim_time': use_sim_time,
                    'is_clock_publisher': (i == 1)
                }],
            ),
            *nav_nodes,
            *graph_nodes 
        ])
        actions_to_launch.append(robot_group)

    # Sync node
    actions_to_launch.append(Node(
        package='swarm_slam_eval',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{
            'num_robots': LaunchConfiguration('num_robots'),
            'use_sim_time': use_sim_time
            }]
    ))

    # Visualizer node
    actions_to_launch.append(Node(
        package='swarm_slam_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        parameters=[{
            'num_robots': LaunchConfiguration('num_robots'),
            'use_sim_time': use_sim_time
            }],
        condition=IfCondition(LaunchConfiguration('visualize')),
    ))

    # RViz node
    actions_to_launch.append(ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file, '--ros-args', '-p', f'use_sim_time:={use_sim_time}'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize'))
    ))

    return actions_to_launch


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='GrAco'),
        DeclareLaunchArgument('dataset_sequence', default_value='ground'),
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('sim_rate', default_value='1.0'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='Whether to launch visualization nodes'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (bag) time'),
        DeclareLaunchArgument('nav_mode', default_value='imu',
                              description='What to use for pose calculation'),
        DeclareLaunchArgument('update_time', default_value='5.0',
                              description='How often (in seconds) the evaluator saves checkpoints'),
        
        OpaqueFunction(function=launch_setup)
    ])