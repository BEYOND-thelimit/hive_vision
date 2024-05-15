import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    depth_sub = Node(
        package='hive_camera',
        executable='depth_sub',
        output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
        [
            depth_sub
        ]
    )
