import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_lab = get_package_share_directory('python_project')

    xacro_file = os.path.join(pkg_lab, 'urdf', 'robot_model.xacro')
    world_path = PathJoinSubstitution([
        FindPackageShare('python_project'),
        'worlds', 'simple.world'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world_path, ' -r']}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]), value_type=str)
        }]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diff_drive', '-topic', 'robot_description', '-x', '1.5', '-y', '1', '-z', '0.2'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_lab, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    control_node = Node(
        package='ros_project_scene',
        executable='control_node',
        name='intelligent_controller',
        output='screen'
    )

    lidar_logger = Node(
        package='ros_project_scene',
        executable='lidar_logger',
        name='lidar_logger',
        output='screen'
    )

    camera_viewer = Node(
        package='image_view',
        executable='image_view',
        arguments=['--ros-args', '--remap', 'image:=/camera/image_raw'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('urdf_model', default_value=xacro_file),
        gz_sim,
        bridge,
        spawn,
        robot_state_publisher,
        control_node,
        lidar_logger,
        camera_viewer,
    ])