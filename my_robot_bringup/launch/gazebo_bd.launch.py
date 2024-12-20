from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Define URDF and world paths
    urdf_path = "/home/lk/ros2_ws/src/my_robot_description/urdf/ar_bd.urdf"

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    empty_world_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'worlds',
        'empty.world'
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world': empty_world_path
        }.items()
    )

    # Spawn the robot in Gazebo using URDF file
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path,  # Provide the URDF file path directly
                '-entity', 'my_robot',
                '-x', '1',
                '-y', '1',
                '-z', '-1',
                '-Y', '0'],
        output='screen'
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot
        # robot_state_publisher
    ])
