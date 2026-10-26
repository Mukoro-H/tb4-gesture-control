import os
from glob import glob

from setuptools import setup

package_name = 'turtlebot4_gesture_control'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]
models = glob('models/*.task')
if models:
    data_files.append((os.path.join('share', package_name, 'models'), models))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minh',
    maintainer_email='leducminh2212@gmail.com',
    description='Hand gesture control of TurtleBot 4 (MediaPipe + ROS 2 Humble)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognition_node = '
            'turtlebot4_gesture_control.gesture_recognition_node:main',
            'robot_behavior_node = turtlebot4_gesture_control.robot_behavior_node:main',
            'camera_test_node = turtlebot4_gesture_control.camera_test_node:main',
        ],
    },
)
