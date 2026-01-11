import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package names
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('my_robot_description')  # Replace with your package name

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control for robot control if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('my_robot_gazebo'),  # Replace with your package name
            'worlds',
            'simple_robot.world'
        ),
        description='Choose one of the world files from `/my_robot_gazebo/worlds`'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless (without GUI) if true'
    )

    # Start Gazebo server and client (optional: off-screen rendering)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'headless': headless,
            'verbose': 'false',
        }.items()
    )

    # Robot State Publisher node
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindPackageShare('my_robot_description'), 'urdf', 'simple_humanoid.urdf']),  # Replace with your URDF
            ' use_sim:=true',
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content},
        ]
    )

    # Spawn entity (robot) in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay spawn_entity after robot_state_publisher starts
    delay_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher,
            on_exit=[spawn_entity],
        )
    )

    # Joint State Publisher GUI (for development)
    joint_state_publisher_gui = Node(
        condition=IfCondition(use_ros2_control),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS 2 Control spawners
    diff_drive_spawner = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_headless_cmd)

    # Add nodes and launch descriptions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(delay_spawn_entity)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    return ld