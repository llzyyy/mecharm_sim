import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("mecharm_sim")
    model = os.path.join(pkg_share, "urdf", "mecharm_270_pick_place.urdf.xacro")
    world = os.path.join(pkg_share, "worlds", "pick_place.sdf")
    controllers = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    poses = os.path.join(pkg_share, "config", "pick_place_poses.yaml")
    mycobot_description_share = get_package_share_directory("mycobot_description")
    gazebo_resource_path = os.path.dirname(mycobot_description_share)

    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    robot_base_z = LaunchConfiguration("robot_base_z")

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
            "Missing ROS package ros_gz_sim. Install runtime dependencies with: "
            "sudo apt install ros-humble-ros-gz-sim ros-humble-gz-ros2-control "
            "ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-control-msgs"
        ) from exc

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mecharm_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_moveit")),
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [world, " -r"],
            "on_exit_shutdown": "true",
        }.items(),
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
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        parameters=[controllers],
        output="screen",
    )

    pick_place = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "mecharm_sim",
            "pick_place",
            "--ros-args",
            "--params-file",
            poses,
            "-p",
            "use_sim_time:=true",
            "-p",
            ["use_moveit:=", LaunchConfiguration("use_moveit")],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_pick_place")),
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
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
                TimerAction(period=12.0, actions=[pick_place]),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_pick_place", default_value="true"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("robot_base_z", default_value="0.34"),
            DeclareLaunchArgument(
                "ros2_control_plugin",
                default_value="gz_ros2_control/GazeboSimSystem",
                description="Use ign_ros2_control/IgnitionSystem on older Humble Fortress installs.",
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
            gz_launch,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            start_controllers,
            start_task,
        ]
    )
