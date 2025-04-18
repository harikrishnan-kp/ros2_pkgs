import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # this is the robot name in the Xacro file
    robotXacroName='differential_drive_robot'

    # this is the name of our package
    packageName = 'two_wheel_car'

    roboFilePath = os.path.join(get_package_share_directory(packageName),'urdf/two_wheel_car.xacro')

    worldFilePath = os.path.join(get_package_share_directory(packageName), 'worlds/tugbot_mod.sdf')

    robotDescription = xacro.process_file(roboFilePath).toxml()



    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource (os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))
    
    # this is the launch description
    # loading empty world model
    gazeboLaunch = IncludeLaunchDescription (gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf '],'on_exit_shutdown': 'true'}.items())
    # loading tugbot world
    # gazeboLaunch = IncludeLaunchDescription (gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 ', worldFilePath],'on_exit_shutdown': 'true'}.items())


    # Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-name', robotXacroName, '-topic', 'robot_description'],
        output='screen',
    )

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # gazebo ros bridging
    bridge_params = os.path.join(get_package_share_directory(packageName),'parameters','bridge_parameters.yaml')

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args','-p', f'config_file:={bridge_params}'],
        output='screen'
        )
    
    # here we create an empty launch description object 
    launchDescriptionObject = LaunchDescription()
    # we add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)
    # we add the two nodes
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    
    return launchDescriptionObject