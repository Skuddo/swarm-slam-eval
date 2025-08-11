from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import launch.logging
import yaml
import os

def launch_multi_robots(context, *args, **kwargs):
    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    sim_rate = float(LaunchConfiguration('sim_rate').perform(context))

    pkg_share = FindPackageShare('swarm_slam_odometry').perform(context)
    dataset_path = os.path.join(pkg_share, '../../../../../datasets', dataset)
    config_path = os.path.join(dataset_path, "config.yaml")

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
        bag_path = os.path.join(dataset_path, f'{conf_dataset_names}{i}')

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

        odometry_node = Node(
            package='swarm_slam_odometry',
            executable='odometry_node',
            name='odometry',
            output='screen',
        )

        group = GroupAction([
            PushRosNamespace(robot_n),
            odometry_node,
            bag_node,
        ])
        robot_groups.append(group)

    return robot_groups


def generate_launch_description():
    declare_dataset = DeclareLaunchArgument(
        'dataset',
        default_value='GrAco',
        description="Name of the dataset"
        )
    declare_dataset_sequence = DeclareLaunchArgument(
        'dataset_sequence', 
        default_value='ground',
        description="Name of the dataset sequences")
    declare_num_robots = DeclareLaunchArgument(
        'num_robots', 
        default_value='1',
        description='How many robots to launch'
        )
    declare_sim_rate = DeclareLaunchArgument(
        'sim_rate',
        default_value='1.0',
        description='Playback speed of the bags'
        )
    declare_sim_update_rate = DeclareLaunchArgument(
        'sim_update_rate',
        default_value='5',
        description='In seconds how often to get global update'
        )
    declare_visualize = DeclareLaunchArgument(
        'visualize', 
        default_value='true',
        description='Whether to launch visualization nodes'
        )

    sync_node = Node(
        package='swarm_slam_odometry',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{
            'num_robots': LaunchConfiguration('num_robots')
        }]
    )

    # Visualization nodes wrapped with condition
    visualize_condition = IfCondition(LaunchConfiguration('visualize'))

    visualizer_node = Node(
        package='swarm_slam_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
        condition=visualize_condition,
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', PathJoinSubstitution([
            FindPackageShare('swarm_slam_visualizer'),
            'config',
            'default.rviz'
        ])],
        output='screen',
        condition=visualize_condition,
    )

    return LaunchDescription([
        declare_dataset,
        declare_dataset_sequence,
        declare_num_robots,
        declare_sim_rate,
        declare_sim_update_rate,
        declare_visualize,

        sync_node,
        OpaqueFunction(function=launch_multi_robots),

        visualizer_node,
        rviz_node,
    ])
