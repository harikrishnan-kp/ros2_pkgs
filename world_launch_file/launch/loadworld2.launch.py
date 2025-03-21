#################################################################################################
# this launch file incorporate the gz_sim.launch.py launch file present in default gazebo package
#################################################################################################
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the GZ_SIM_RESOURCE_PATH environment variable for accessing prebuild models in sdf file
    SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/hari/ros2_ws/src/world_launch_file/models'
    )

    # Declare the 'world_file' launch argument
    DeclareLaunchArgument(
        'world_file',
        default_value='/home/hari/ros2_ws/src/world_launch_file/world/tugbotnew.sdf',
        description='Path to the world file to load'
    )

    # Get the path to the 'gz_sim.launch.py' file from the 'ros_gz_sim' package
    ros_gz_sim_launch_dir = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch'
    )

    # Include the 'gz_sim.launch.py' launch file
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_launch_dir, 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('world_file')}.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/home/hari/ros2_ws/src/world_launch_file/models'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='/home/hari/ros2_ws/src/world_launch_file/world/tugbotnew.sdf',
            description='Path to the world file to load'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_launch_dir, 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': LaunchConfiguration('world_file')}.items(),
        ),
    ])
