from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    pkg_share = FindPackageShare('robo')
    urdf_filename = LaunchConfiguration('urdf')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', urdf_filename])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'rviz_config.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf', default_value='box_robo.urdf', description='URDF file path'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
