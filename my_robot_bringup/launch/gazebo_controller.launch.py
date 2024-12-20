from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode')
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Enable GUI')
    paused_arg = DeclareLaunchArgument('paused', default_value='true', description='Start Gazebo paused')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    height_arg = DeclareLaunchArgument('height', default_value='0', description='Initial height of the robot')

    # Define paths to URDF, Gazebo, and empty world
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'gazebo_urdf',
        'ar_bd.urdf'
    )
    
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
            'world': empty_world_path,
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-urdf',
            '-model', 'my_robot',
            '-file', urdf_path,
            '-z', LaunchConfiguration('height')
        ]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        remappings=[('/joint_states', '/my_robot/joint_states')]  # Adjust if using namespaces
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    # )

    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    # )

    return LaunchDescription([
        debug_arg,
        gui_arg,
        paused_arg,
        use_sim_time_arg,
        height_arg,
        gazebo,
        spawn_robot,
        robot_state_publisher
        # joint_state_publisher,
        # joint_state_publisher_gui
    ])
