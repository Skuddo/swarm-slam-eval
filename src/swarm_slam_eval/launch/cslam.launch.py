import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    """
    This function directly launches all necessary nodes for a C-SLAM system,
    including RTAB-Map for odometry and the core C-SLAM backend components.
    It is designed to be self-contained to avoid dependency issues at runtime.
    """
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time') # Keep as LaunchConfiguration object
    cslam_config_file = LaunchConfiguration('cslam_config_file').perform(context)
    
    actions_to_launch = []

    # --- Per-Robot Node Launching ---
    for i in range(num_robots):
        robot_namespace = f'r{i}'

        tf_process = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments="0 0 0 0 0 0 velodyne base_link".split(" "),
            parameters=[{'use_sim_time': use_sim_time}] 
        )
        actions_to_launch.append(tf_process)

        tf_process_imu = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments="-0.01192 -0.0197 0.1226 0 0 0 velodyne gnss".split(" "),
            parameters=[{'use_sim_time': use_sim_time}] 
        )
        actions_to_launch.append(tf_process_imu)

        tf_process_map = Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments="0 0 0 0 0 0 map odom".split(" "),
                    parameters=[{'use_sim_time': use_sim_time}] 
        )
        actions_to_launch.append(tf_process_map)


        # 1. RTAB-Map ICP Odometry Node (Frontend)
        # This node computes odometry from the point cloud data.
        odom_frontend_node = Node(
            package='rtabmap_odom', 
            executable='icp_odometry', 
            name='icp_odometry',
            output="screen",
            parameters=[{
                "frame_id": "velodyne",
                "odom_frame_id": "odom", # This will be published as /rX/odom
                "publish_tf": True, # We handle TF separately if needed
                "wait_for_transform": 0.2,
                "wait_imu_to_init": True,
                # Use Approximate Time synchronization for robust bag playback.
                "approx_sync": False,
                "queue_size": 100,
                "queue_size_odom": "100",
                "use_sim_time": use_sim_time,
                "odom_tf_angular_variance": 0.01,
                "odom_tf_linear_variance": 0.001,
                "odom_guess_min_translation": '0.0',
                "odom_guess_min_rotation": '0.0',
                # Parameters from graco_lidar.yaml and rtabmap launch
                "Icp/MaxTranslation": "5",
                "Icp/VoxelSize": "0.4",
                "Icp/MaxCorrespondenceDistance": "4.0",
                "Icp/PointToPlaneK": "20",
                "Odom/Strategy": "0", # 0=Frame-to-Map
                "OdomF2M/ScanSubtractRadius": "0.4",
                "OdomLOAM/Sensor": "0", # 16 rings
                "OdomLOAM/Resolution": "0.4" 
            }],
            remappings=[
                # Remap the input topic to what our bag_reader publishes
                ("scan_cloud", "pointcloud"),
                # Disable the unused 2D scan topic to prevent timeout
                ("scan", "scan_disabled"),
                # Output odometry topic will be automatically namespaced to /rX/odom
                ("odom", "odom") 
            ],
            namespace=robot_namespace
        )
        actions_to_launch.append(odom_frontend_node)

        # 2. C-SLAM: Loop Closure Detection Node
        loop_detection_node = Node(
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
            namespace=robot_namespace
        )
        actions_to_launch.append(loop_detection_node)

        # 3. C-SLAM: LiDAR Handler (Map Manager) Node
        map_manager_node = Node(
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
            namespace=robot_namespace
        )
        actions_to_launch.append(map_manager_node)

        # 4. C-SLAM: Pose Graph Manager Node
        pose_graph_manager_node = Node(
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
            namespace=robot_namespace
        )
        actions_to_launch.append(pose_graph_manager_node)

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