from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():

    # Find the ROS 2 scripts relative to this launch file.
    real_robot_dir = Path(__file__).resolve().parent.parent
    ros2_dir = real_robot_dir / 'ros2'

    bridge_script = (
        ros2_dir
        / 'robomaster_ros2_bridge.py'
    )

    controller_script = (
        ros2_dir
        / 'robomaster_experiment_controller.py'
    )

    # Both ROS 2 processes use the same local DDS settings.
    ros_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='0'
    )

    localhost_only = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY',
        value='1'
    )

    # The bridge stays running and handles communication
    # between ROS 2 and the Windows RoboMaster adapter.
    bridge = ExecuteProcess(
        cmd=[
            'python3',
            str(bridge_script)
        ],
        output='screen'
    )

    # The controller performs one fixed-point grasping cycle.
    # It exits automatically after success or failure.
    controller = ExecuteProcess(
        cmd=[
            'python3',
            str(controller_script)
        ],
        output='screen'
    )

    return LaunchDescription([
        ros_domain,
        localhost_only,
        bridge,
        controller
    ])
