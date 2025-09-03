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
    use_sim_time = LaunchConfiguration('use_sim_time') # Keep as LaunchConfiguration object
    cslam_config_file = LaunchConfiguration('cslam_config_file').perform(context)
    
    actions_to_launch = []

    # --- Per-Robot Node Launching ---
    for i in range(num_robots):
        robot_namespace = f'r{i}'

        static_tf_components = [
            # Component for base_link -> velodyne
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='static_tf_pub_base_to_velodyne',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': 'base_link',
                    'child_frame_id': 'velodyne',
                    'translation.x': 0.0,
                    'translation.y': 0.0,
                    'translation.z': 0.0,
                    'rotation.x': 0.0,
                    'rotation.y': 0.0,
                    'rotation.z': 0.0,
                    'rotation.w': 1.0
                }]
            ),
            # Component for base_link -> gnss
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='static_tf_pub_base_to_gnss',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': 'base_link',
                    'child_frame_id': 'gnss',
                    'translation.x': -0.01192,
                    'translation.y': -0.0197,
                    'translation.z': 0.1226,
                    'rotation.x': 0.0,
                    'rotation.y': 0.0,
                    'rotation.z': 0.0,
                    'rotation.w': 1.0
                }]
            ),
            # Add any other static transforms here in the same way
        ]

        tf_container = ComposableNodeContainer(
            name='static_tf_container',
            namespace=robot_namespace, # Apply namespace directly to the container
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=static_tf_components,
            output='screen'
        )

        actions_to_launch.append(tf_container)

        # Group all nodes for a single robot under a namespace
        robot_group = GroupAction(
            actions=[
                # Use PushRosNamespace to apply the namespace to all nodes within this group
                PushRosNamespace(robot_namespace),

                # 1. CORRECTED: Static transform from the robot's base to its sensor.
                # This is now correctly namespaced. It will publish /rX/base_link -> /rX/velodyne.
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    # Arguments: x y z yaw pitch roll parent_frame child_frame
                    arguments="0 0 0 0 0 0 base_link velodyne".split(" "),
                    parameters=[{'use_sim_time': use_sim_time}] 
                ),

                # Note: The static transform for "gnss" is also included here.
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments="-0.01192 -0.0197 0.1226 0 0 0 base_link gnss".split(" "),
                    parameters=[{'use_sim_time': use_sim_time}] 
                ),

                # REMOVED: The static map -> odom publisher has been deleted as it conflicts with SLAM.

                # 2. RTAB-Map ICP Odometry Node (Frontend)
                # Corrected to publish the standard odom -> base_link transform.
                Node(
                    package='rtabmap_odom', 
                    executable='icp_odometry', 
                    name='icp_odometry',
                    output="screen",
                    parameters=[{
                        # CHANGED: frame_id should be the robot's base frame.
                        "frame_id": "base_link", 
                        "odom_frame_id": "odom",
                        "publish_tf": True, # This node is now responsible for odom -> base_link
                        "wait_for_transform": 0.2,
                        "wait_imu_to_init": True,
                        "approx_sync": True, # Set to True for more robust bag playback
                        "queue_size": 100,
                        "use_sim_time": use_sim_time,
                        # Other parameters remain the same...
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
                    # REMOVED: namespace attribute is now handled by the GroupAction
                ),

                # 3. C-SLAM: Loop Closure Detection Node
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

                # 4. C-SLAM: LiDAR Handler (Map Manager) Node
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

                # 5. C-SLAM: Pose Graph Manager Node
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
    # Path to the default C-SLAM config file, now located within our own package.
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
        
        OpaqueFunction(function=launch_setup)
    ])