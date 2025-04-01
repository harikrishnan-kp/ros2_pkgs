#################################################################################################
# This launch file uses the gz_sim.launch.py launch file present in the ros_gz_sim package (bridge)
# also there is no option for launch time arguments
#################################################################################################
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():
    package_name = "launch_gz_worlds"
    world_file = "tugbot_mod.sdf"

    # Get the path to the 'gz_sim.launch.py' file in the 'ros_gz_sim' package
    ros_gz_sim_launch_file = path.join(get_package_share_directory('ros_gz_sim'), 'launch/gz_sim.launch.py')

    return LaunchDescription([
        # Setting environment variable for accessing prebuilt models included in the world file SDF
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=path.join(get_package_share_directory(package_name), "models")
        ),

        # Include the ROS GZ Sim launch file and pass the corrected world file argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_gz_sim_launch_file),
            launch_arguments={'gz_args': path.join(get_package_share_directory(package_name), "world", world_file)}.items(),
        ),
    ])
