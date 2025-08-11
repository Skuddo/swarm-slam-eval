from setuptools import find_packages, setup

package_name = 'swarm_slam_odometry'

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
            'odometry_node = swarm_slam_odometry.odometry:main',
            'bag_reader_node = swarm_slam_odometry.bag_reader:main',
            'sync_node = swarm_slam_odometry.sync:main',
        ],
    },
)
