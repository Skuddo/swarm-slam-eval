#!/usr/bin/env python3
import os
import yaml
import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("multi_launch_setup")

    dataset = LaunchConfiguration('dataset').perform(context)
    sequence = LaunchConfiguration('dataset_sequence').perform(context)
    visualize = LaunchConfiguration('visualize').perform(context)


    # Prepare parameters forwarded to dataset-specific launch
    launch_args = {
        'dataset': dataset,
        'dataset_sequence': sequence,
        'visualize': visualize,
        'num_robots': '3',
        'use_sim_time': 'true',
        'nav_mode': 'cslam',
        'update_time': '5.0',
    }

    launch_dir = os.path.join(get_package_share_directory('swarm_slam_eval'), 'launch')

    if dataset == "S3E":
        s3e_launch_file = os.path.join(launch_dir, 's3e.launch.py')
        logger.info(f"Including s3e.launch.py for dataset {dataset}, sequence {sequence}")
        return [ IncludeLaunchDescription(
            PythonLaunchDescriptionSource(s3e_launch_file),
            launch_arguments=launch_args.items()
        ) ]
    elif dataset == "GrAco":
        graco_launch_file = os.path.join(launch_dir, 'graco.launch.py')
        logger.info(f"Including graco.launch.py for dataset {dataset}, sequence {sequence}")
        return [ IncludeLaunchDescription(
            PythonLaunchDescriptionSource(graco_launch_file),
            launch_arguments=launch_args.items()
        ) ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='GrAco',
                              description='Dataset folder name under <project_root>/datasets/'),
        DeclareLaunchArgument('dataset_sequence', default_value='ground',
                              description='Sequence key inside dataset config (e.g. ground or playground)'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='Dataset folder name under <project_root>/datasets/'),
        OpaqueFunction(function=launch_setup)
    ])
