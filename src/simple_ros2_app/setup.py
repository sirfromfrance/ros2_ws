import os
from glob import glob
from setuptools import setup

package_name = 'simple_ros2_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vt',
    maintainer_email='vt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_control_node = simple_ros2_app.px4_control_node:main',
            'swarm_control_node = simple_ros2_app.swarm_control_node:main',
            'optical_flow_node = simple_ros2_app.optical_flow_node:main',
            'ai_tracker_node = simple_ros2_app.ai_tracking_node:main',
            
        ],
    },
)
