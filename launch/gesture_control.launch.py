"""Start gesture_recognition_node + robot_behavior_node.

ros2 launch turtlebot4_gesture_control gesture_control.launch.py camera_index:=0

Start the TurtleBot 4 simulation in another terminal first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('turtlebot4_gesture_control')
    default_params = os.path.join(share, 'config', 'params.yaml')

    params = LaunchConfiguration('params_file')
    camera_index = LaunchConfiguration('camera_index')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('camera_index', default_value='0',
                              description='webcam index (/dev/videoN)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='behaviour timing follows Gazebo /clock'),

        Node(
            package='turtlebot4_gesture_control',
            executable='gesture_recognition_node',
            name='gesture_recognition_node',
            output='screen',
            emulate_tty=True,
            parameters=[params,
                        {'camera_index': ParameterValue(camera_index, value_type=int)}],
        ),
        Node(
            package='turtlebot4_gesture_control',
            executable='robot_behavior_node',
            name='robot_behavior_node',
            output='screen',
            emulate_tty=True,
            parameters=[params,
                        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
