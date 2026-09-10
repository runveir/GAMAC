from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():

    pkg_share = FindPackageShare('drone_sim').find('drone_sim')

    world_file = Path(pkg_share) / 'worlds' / 'drone_world.sdf'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path('/opt/ros/jazzy/share/ros_gz_sim/launch')
                / 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    return LaunchDescription([
        gazebo
    ])