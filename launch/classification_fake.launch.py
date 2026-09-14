import os

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mecharm_sim.classification_config import flatten_ros_parameters


def generate_launch_description():
    pkg_share = get_package_share_directory("mecharm_sim")
    model = os.path.join(pkg_share, "urdf", "mecharm_270_classification.urdf.xacro")
    world = os.path.join(pkg_share, "worlds", "classification_fake.sdf")
    controllers = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    classification_config = os.path.join(
        pkg_share, "config", "classification_fake.yaml"
    )
    with open(classification_config, "r", encoding="utf-8") as stream:
        task_config = yaml.safe_load(stream)
    controller_parameters = flatten_ros_parameters(task_config["controller"])

    mycobot_description_share = get_package_share_directory("mycobot_description")
    gazebo_resource_path = os.path.dirname(mycobot_description_share)
    gazebo_system_plugin_path = os.path.join(
        get_package_prefix("gz_ros2_control"), "lib"
    )
    use_moveit = LaunchConfiguration("use_moveit")
    use_classification = LaunchConfiguration("use_classification")
    robot_base_z = LaunchConfiguration("robot_base_z")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                model,
                " ros2_control_plugin:=",
                ros2_control_plugin,
                " controllers_file:=",
                controllers,
                " robot_base_z:=",
                robot_base_z,
            ]
        ),
        value_type=str,
    )

    try:
        ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Missing ROS package ros_gz_sim. Install the Humble Gazebo and "
            "ros2_control dependencies listed in UBUNTU_TODO.md."
        ) from exc

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [world, " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mecharm_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        condition=IfCondition(use_moveit),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "mecharm_270", "-topic", "robot_description", "-z", "0.0"],
        output="screen",
    )
    detach_targets_on_spawn = [
        ExecuteProcess(
            cmd=[
                "ign",
                "topic",
                "-t",
                str(target["detach_topic"]),
                "-m",
                "ignition.msgs.Empty",
                "-p",
                "unused: true",
            ],
            output="screen",
        )
        for target in task_config["objects"]
    ]

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[controllers],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[controllers],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[controllers],
        output="screen",
    )

    classification_pick_place = Node(
        package="mecharm_sim",
        executable="classification_pick_place",
        name="classification_pick_place",
        output="screen",
        condition=IfCondition(use_classification),
        parameters=[
            controller_parameters,
            {
                "config_file": classification_config,
                "use_sim_time": True,
                "use_moveit": ParameterValue(use_moveit, value_type=bool),
            },
        ],
    )
    classification_manager = Node(
        package="mecharm_sim",
        executable="classification_manager",
        name="classification_manager",
        output="screen",
        condition=IfCondition(use_classification),
        parameters=[{"config_file": classification_config, "use_sim_time": True}],
    )
    fake_vision = Node(
        package="mecharm_sim",
        executable="fake_vision",
        name="fake_vision",
        output="screen",
        condition=IfCondition(use_classification),
        parameters=[{"config_file": classification_config, "use_sim_time": True}],
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=0.2, actions=detach_targets_on_spawn),
                TimerAction(period=5.0, actions=[joint_state_spawner]),
                TimerAction(period=6.0, actions=[arm_spawner]),
                TimerAction(period=7.0, actions=[gripper_spawner]),
            ],
        )
    )
    start_task = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[
                TimerAction(period=2.0, actions=[move_group]),
                TimerAction(period=3.0, actions=[classification_pick_place]),
                TimerAction(period=5.0, actions=[classification_manager]),
                TimerAction(period=5.0, actions=[fake_vision]),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_classification", default_value="true"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("robot_base_z", default_value="0.34"),
            DeclareLaunchArgument(
                "ros2_control_plugin",
                default_value="gz_ros2_control/GazeboSimSystem",
                description=(
                    "Use ign_ros2_control/IgnitionSystem on older Humble Fortress installs."
                ),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    gazebo_resource_path,
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    gazebo_resource_path,
                    ":",
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                value=[
                    gazebo_system_plugin_path,
                    ":",
                    EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=[
                    gazebo_system_plugin_path,
                    ":",
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            gz_launch,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            start_controllers,
            start_task,
        ]
    )
