############################################################################
# in this launch file we are invoking gz_sim directly within the launch file
# there is an option for changing 'world_file' argument during launch time
#############################################################################
from os import path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = "launch_gz_worlds"
    world_file = "tugbot_mod.sdf"

    return LaunchDescription([
        # Setting environment variable for accessing prebuild models included in world file sdf
        SetEnvironmentVariable(
                name ='GZ_SIM_RESOURCE_PATH',
                value = path.join(get_package_share_directory(package_name),"models")
        ),
        DeclareLaunchArgument(
                "world_file",
                default_value = path.join(get_package_share_directory(package_name),"world",world_file),
                description = "world file path"
        ),
        ExecuteProcess( 
                cmd=["gz", "sim", LaunchConfiguration("world_file")],
                output="screen"
        ),
    ])
