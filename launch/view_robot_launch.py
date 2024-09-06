import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    with tempfile.NamedTemporaryFile(prefix="%s_" % os.path.basename(xacro_path), delete=False) as xacro_file:
        urdf_path = xacro_file.name

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():
    # Get the directory where the URDF file is located
    package_dir = get_package_share_directory('lb_description')

    # Declare a launch argument for the URDF file
    urdf_file_path = os.path.join(package_dir, 'urdf', 'lb_description.urdf.xacro')

    urdf = to_urdf(urdf_file_path)
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=urdf_file_path, description='Path to the URDF file'),

        # Launch the robot_state_publisher node with the specified URDF file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf],
            #parameters=[{'robot_description': open(urdf_file_path).read()}],
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