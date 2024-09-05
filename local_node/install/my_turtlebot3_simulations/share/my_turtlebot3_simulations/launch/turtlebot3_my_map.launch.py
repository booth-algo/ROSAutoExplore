import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    world_file_name = 'minimal.world'
    world = os.path.join(get_package_share_directory('my_turtlebot3_simulations'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py']),
            launch_arguments={'world': world}.items(),
        ),
    ])
