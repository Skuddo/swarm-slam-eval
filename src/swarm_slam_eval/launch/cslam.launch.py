import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time') 
    cslam_config_file = LaunchConfiguration('cslam_config_file').perform(context)
    dataset = LaunchConfiguration('dataset').perform(context)

    actions_to_launch = []

    for i in range(num_robots):
        robot_namespace = f'r{i}'

        # Pick the correct transform container based on dataset
        if dataset == 'GrAco':
            transform_container = ComposableNodeContainer(
                name='static_tf_container',
                namespace='',  # Namespace is applied by GroupAction
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='tf2_ros',
                        plugin='tf2_ros::StaticTransformBroadcasterNode',
                        name='base_to_velodyne_tf',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'frame_id': 'base_link',
                            'child_frame_id': 'velodyne',
                        }]
                    ),
                    ComposableNode(
                        package='tf2_ros',
                        plugin='tf2_ros::StaticTransformBroadcasterNode',
                        name='base_to_gnss_tf',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'frame_id': 'base_link',
                            'child_frame_id': 'gnss',
                            'translation.x': -0.01192,
                            'translation.y': -0.0197,
                            'translation.z': 0.1226,
                        }]
                    ),
                ],
                output='screen'
            )
        elif dataset == 'S3E':
            transform_container = ComposableNodeContainer(
                name='s3e_static_tf_container',
                namespace='',  # Namespace is applied by GroupAction
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='tf2_ros',
                        plugin='tf2_ros::StaticTransformBroadcasterNode',
                        name='left_camera_to_velodyne_tf',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'frame_id': 'left_camera',
                            'child_frame_id': 'velodyne',
                            'translation.x': 0.13,
                            'translation.y': 0.20,
                            'translation.z': -0.13,
                            'rotation.r': 0.5,
                            'rotation.p': -0.5,
                            'rotation.y': 0.5,
                        }]
                    ),
                    ComposableNode(
                        package='tf2_ros',
                        plugin='tf2_ros::StaticTransformBroadcasterNode',
                        name='velodyne_to_imu_tf',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'frame_id': 'velodyne',
                            'child_frame_id': 'imu_link',
                            'translation.x': -0.051,
                            'translation.y': 0.0195,
                            'translation.z': -0.094,
                        }]
                    ),
                    ComposableNode(
                        package='tf2_ros',
                        plugin='tf2_ros::StaticTransformBroadcasterNode',
                        name='base_link_to_left_camera_tf',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'frame_id': 'base_link',
                            'child_frame_id': 'left_camera',
                            'translation.x': 0.0,
                            'translation.y': 0.0,
                            'translation.z': 0.0,
                            'rotation.r': 0.0,
                            'rotation.p': 0.0,
                            'rotation.y': 0.0,
                        }]
                    ),
                ],
                output='screen'
            )
        else:
            raise RuntimeError(f"Unsupported dataset: {dataset}")

        robot_group = GroupAction(
            actions=[
                PushRosNamespace(robot_namespace),
                transform_container,

                Node(
                    package='rtabmap_odom', 
                    executable='icp_odometry', 
                    name='icp_odometry',
                    output="screen",
                    parameters=[{
                        "frame_id": "base_link", 
                        "odom_frame_id": "odom",
                        "publish_tf": True,
                        "wait_for_transform": 0.2,
                        "wait_imu_to_init": True,
                        "approx_sync": True,
                        "queue_size": 100,
                        "use_sim_time": use_sim_time,
                        "Icp/MaxTranslation": "5",
                        "Icp/VoxelSize": "0.4",
                        "Icp/MaxCorrespondenceDistance": "4.0",
                        "Icp/PointToPlaneK": "20",
                        "Odom/Strategy": "0",
                        "OdomF2M/ScanSubtractRadius": "0.4"
                    }],
                    remappings=[
                        ("scan_cloud", "pointcloud"),
                        ("scan", "scan_disabled"),
                        ("odom", "odom") 
                    ],
                ),

                Node(
                    package='cslam',
                    executable='loop_closure_detection_node.py',
                    name='cslam_loop_closure_detection',
                    parameters=[
                        cslam_config_file, {
                            "robot_id": i,
                            "max_nb_robots": num_robots,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),

                Node(
                    package='cslam',
                    executable='lidar_handler_node.py',
                    name='cslam_map_manager',
                    parameters=[
                        cslam_config_file, {
                            "robot_id": i,
                            "max_nb_robots": num_robots,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),

                Node(
                    package='cslam',
                    executable='pose_graph_manager',
                    name='cslam_pose_graph_manager',
                    parameters=[
                        cslam_config_file, {
                            "robot_id": i,
                            "max_nb_robots": num_robots,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                )
            ]
        )
        actions_to_launch.append(robot_group)

    return actions_to_launch

def generate_launch_description():
    default_cslam_config = os.path.join(
        get_package_share_directory("swarm_slam_eval"), 
        "config", 
        "cslam.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3',
                              description='Number of robots in the swarm.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (bag) time.'),
        DeclareLaunchArgument('cslam_config_file', default_value=default_cslam_config,
                              description='Path to the C-SLAM parameters YAML file.'),
        DeclareLaunchArgument('dataset', default_value='GrAco',
                              description='Which dataset to use (GrAco or S3E).'),
        
        OpaqueFunction(function=launch_setup)
    ])
