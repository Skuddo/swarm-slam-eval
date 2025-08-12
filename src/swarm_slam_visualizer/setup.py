from setuptools import find_packages, setup

package_name = 'swarm_slam_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Skuddo',
    maintainer_email='56997587+Skuddo@users.noreply.github.com',
    description='swarm-slam-evaluator visualization node',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualizer_node = swarm_slam_visualizer.visualizer:main',
        ],
    },
)
