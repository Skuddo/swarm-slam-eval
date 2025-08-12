from setuptools import find_packages, setup

package_name = 'swarm_slam_eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Skuddo',
    maintainer_email='56997587+Skuddo@users.noreply.github.com',
    description='swarm-slam-evaluator odometry node',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_reader_node = swarm_slam_eval.bag_reader:main',
            'sync_node = swarm_slam_eval.sync:main',
            'odometry_node = swarm_slam_eval.odometry.ground_truth:main',
            'imu_node = swarm_slam_eval.odometry.imu:main',
        ],
    },
)
