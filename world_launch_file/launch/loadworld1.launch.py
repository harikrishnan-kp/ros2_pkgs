############################################################################
# in this launch file we are invoking gz_sim directly within the launch file
#############################################################################
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():

    return LaunchDescription([
        # Set the GZ_SIM_RESOURCE_PATH environment variable for accessing prebuild models in sdf file
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/home/hari/ros2_ws/src/world_launch_file/models'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='/home/hari/ros2_ws/src/world_launch_file/world/tugbotnew.sdf',
            description='Path to the world file to load'
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world_file')],
            output='screen'
        ),
    ])
