# swarm-slam-visualization/launch/visualize_swarm.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch the visualization node
        Node(
            package='swarm_slam_visualizer',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen'
        ),

        # Launch RViz2 with the default config file
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([
                FindPackageShare('swarm_slam_visualizer'),
                'config',
                'default.rviz'
            ])],
            output='screen'
        )
    ])

