import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package path
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_hunter_descripton = get_package_share_directory('hunter_drone_description')
    pkg_hunter_gazebo = get_package_share_directory('hunter_drone_gazebo')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )

    # Robot model
    xacro_file = os.path.join(pkg_hunter_descripton, 'urdf', 'hunter.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Robot state pub
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable ='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hunter_drone'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])