#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    # Get livox laser simulation package directory for plugin path
    livox_pkg_dir = get_package_share_directory('livox_laser_simulation')
    livox_plugin_path = os.path.join(livox_pkg_dir, '..', '..', 'lib')

    # 기존 경로를 보존하면서 Livox 플러그인만 추가
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    combined_plugin_path = (
        f"{existing_plugin_path}:{livox_plugin_path}"
        if existing_plugin_path else livox_plugin_path
    )

    # Set Gazebo plugin path to include livox plugin
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=combined_plugin_path
    )

    # Launch arguments
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    gazebo_world = LaunchConfiguration("world")
    description_path = LaunchConfiguration("description_path")

    # Package paths
    go2_mid360_nav_share = get_package_share_directory('go2_mid360_nav')

    # Default paths
    default_world_path = os.path.join(go2_mid360_nav_share, 'worlds', 'empty.world')
    default_model_path = os.path.join(go2_mid360_nav_share, 'urdf', 'go2_with_mid360.xacro')
    default_rviz_path = os.path.join(go2_mid360_nav_share, 'rviz', 'robot_with_lidar.rviz')
    gazebo_config = os.path.join(go2_mid360_nav_share, 'config', 'gazebo.yaml')

    # Config paths for champ controller
    joints_config = os.path.join(go2_mid360_nav_share, 'config', 'joints', 'joints.yaml')
    gait_config = os.path.join(go2_mid360_nav_share, 'config', 'gait', 'gait.yaml')
    links_config = os.path.join(go2_mid360_nav_share, 'config', 'links', 'links.yaml')

    # Declare launch arguments
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go2_with_mid360")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_gui = DeclareLaunchArgument("gui", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_gazebo_world = DeclareLaunchArgument("world", default_value=default_world_path)
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.35")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")
    declare_description_path = DeclareLaunchArgument(
        name="description_path",
        default_value=default_model_path,
        description="Absolute path to robot urdf file"
    )
    declare_base_link_frame = DeclareLaunchArgument("base_link_frame", default_value="base_link")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz")

    # Robot description
    robot_description = {
        "robot_description": Command(["xacro ", LaunchConfiguration("description_path")])
    }

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}]
    )

    # Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            gazebo_world,
            "--ros-args",
            "--params-file", gazebo_config
        ],
        output="screen",
    )

    # Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([" not ", headless])),
        cmd=["gzclient"],
        output="screen",
    )

    # Spawn entity
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", robot_name,
            "-topic", "/robot_description",
            "-x", world_init_x,
            "-y", world_init_y,
            "-z", world_init_z,
            "-R", "0",
            "-P", "0",
            "-Y", world_init_heading,
        ],
    )

    # Quadruped controller (handles cmd_vel)
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": True},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(['xacro ', description_path])},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    # State estimator
    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": False},
            {"urdf": Command(['xacro ', description_path])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    # Load controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_states_controller'],
        output='screen',
    )

    load_joint_trajectory_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_effort_controller'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return LaunchDescription([
        set_gazebo_plugin_path,
        declare_robot_name,
        declare_use_sim_time,
        declare_gui,
        declare_headless,
        declare_gazebo_world,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        declare_description_path,
        declare_base_link_frame,
        declare_rviz,
        robot_state_publisher,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
        load_joint_state_controller,
        load_joint_trajectory_effort_controller,
        quadruped_controller_node,
        state_estimator_node,
        rviz_node,
    ])
