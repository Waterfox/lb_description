from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the directory where the URDF file is located
    package_dir = get_package_share_directory('lb_description')

    # Declare a launch argument for the URDF file
    urdf_file_path = os.path.join(package_dir, 'urdf', 'lb_description.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=urdf_file_path, description='Path to the URDF file'),

        # Launch the robot_state_publisher node with the specified URDF file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'lb_description.rviz')],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    ])