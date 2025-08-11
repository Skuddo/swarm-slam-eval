from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import launch.logging
import yaml
import os

def launch_multi_robots(context, *args, **kwargs):
    # Perform substitutions to get concrete values
    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    sim_rate = float(LaunchConfiguration('sim_rate').perform(context))

    pkg_share = FindPackageShare('swarm_slam_odometry').perform(context)

    # Build actual dataset path as string
    dataset_path = os.path.join(pkg_share, '../../../../../datasets', dataset)
    config_path = os.path.join(dataset_path, "config.yaml")

    # Load YAML config
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    conf_max_robots = config["ground"]["sequences"]

    logger = launch.logging.get_logger("multi-launch")
    if conf_max_robots < num_robots:
        logger.error(
            f"Specified number of robots for simulation higher than allowed for dataset\n"
            f"current: {num_robots}, allowed: {conf_max_robots}"
        )
        return [Shutdown()]
    else:
        logger.info(
            f"Running simulation for {num_robots} robots out of {conf_max_robots} allowed"
        )
    
    conf_dataset_names = config[sequence]["names"]
    
    robot_groups = []
    for i in range(1, num_robots + 1):
        robot_n = f'r{i}'
        
        # Define the path to the specific bag file for this robot
        bag_path = os.path.join(dataset_path, f'{conf_dataset_names}{i}')

        # **CORRECTED**: Parameters are now for the bag_reader_node
        bag_node = Node(
            package='swarm_slam_odometry',
            executable='bag_reader_node',
            name='bag_reader',
            output='screen',
            parameters=[{
                'bag_path': bag_path,
                'sim_rate': sim_rate,
            }]
        )

        # The odometry node no longer needs these parameters directly
        odometry_node = Node(
            package='swarm_slam_odometry',
            executable='odometry_node',
            name='odometry',
            output='screen',
        )

        # Group the nodes under a common namespace for this robot
        group = GroupAction([
            PushRosNamespace(robot_n),
            odometry_node,
            bag_node,
        ])
        robot_groups.append(group)

    return robot_groups

def generate_launch_description():
    # --- Launch Arguments (remain the same) ---
    declare_dataset = DeclareLaunchArgument('dataset', default_value='GrAco')
    declare_dataset_sequence = DeclareLaunchArgument('dataset_sequence', default_value='ground')
    declare_num_robots = DeclareLaunchArgument('num_robots', default_value='1')
    declare_sim_rate = DeclareLaunchArgument('sim_rate', default_value='1.0')
    declare_sim_update_rate = DeclareLaunchArgument('sim_update_rate', default_value='5')

    # **NEW**: Add the central synchronization node
    sync_node = Node(
        package='swarm_slam_odometry', # Assuming it's in the same package
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{
            'num_robots': LaunchConfiguration('num_robots')
        }]
    )

    return LaunchDescription([
        declare_dataset,
        declare_dataset_sequence,
        declare_num_robots,
        declare_sim_rate,
        declare_sim_update_rate,
        
        # Launch the single sync node
        sync_node,
        
        # Launch the robot groups
        OpaqueFunction(function=launch_multi_robots)
    ])