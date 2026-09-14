import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RoboMasterExperimentController(Node):

    def __init__(self):
        super().__init__('robomaster_experiment_controller')

        # The YAML file is stored in real_robot/config.
        config_path = (
            Path(__file__).resolve().parent.parent
            / 'config'
            / 'robomaster_params.yaml'
        )

        # Load experiment parameters from YAML instead of
        # keeping them fixed inside the Python program.
        with open(config_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)

        self.command_timeout = float(
            config['controller']['command_timeout']
        )

        self.get_logger().info(
            f'Loaded configuration: {config_path}'
        )

        self.get_logger().info(
            f'Command timeout: {self.command_timeout:.1f} s'
        )

        # Commands are sent to the ROS 2 bridge through this topic.
        self.command_publisher = self.create_publisher(
            String,
            '/robomaster/command',
            10
        )

        # READY, DONE and ERROR messages come back through this topic.
        self.status_subscriber = self.create_subscription(
            String,
            '/robomaster/status',
            self.status_callback,
            10
        )

        # Fixed-point grasping sequence used for one experiment.
        self.commands = [
            'OPEN_GRIPPER',
            'ARM_DOWN',
            'CLOSE_GRIPPER',
            'ARM_UP',
            'ROTATE',
            'ARM_DOWN',
            'OPEN_GRIPPER',
            'ARM_UP'
        ]

        self.index = 0
        self.started = False
        self.waiting = False
        self.completed = False

        self.command_start_time = None

        # Check regularly whether the current action has timed out.
        self.timeout_timer = self.create_timer(
            0.2,
            self.check_timeout
        )

        self.get_logger().info(
            'ROS2 RoboMaster experiment controller ready'
        )

        self.get_logger().info(
            'Waiting for RoboMaster READY...'
        )

    def publish_current_command(self):

        # One complete grasping and placing cycle has finished.
        if self.index >= len(self.commands):

            self.completed = True
            self.waiting = False

            self.get_logger().info(
                'EXPERIMENT_SUCCESS'
            )

            self.get_logger().info(
                'All ROS2 commands completed successfully'
            )

            self.get_logger().info(
                'Experiment controller will now exit'
            )

            return

        command = self.commands[self.index]

        msg = String()
        msg.data = command

        self.command_publisher.publish(msg)

        self.waiting = True
        self.command_start_time = time.monotonic()

        self.get_logger().info(
            f'[ROS2 PUBLISH] {command}'
        )

    def status_callback(self, msg):

        status = msg.data

        self.get_logger().info(
            f'[ROBOT STATUS] {status}'
        )

        if self.completed:
            return

        # READY means the Windows adapter and robot are available.
        if status == 'READY':

            if not self.started:

                self.started = True

                self.get_logger().info(
                    'RoboMaster READY - starting experiment'
                )

                self.publish_current_command()

            return

        # Any hardware-side error stops the current experiment.
        if status.startswith('ERROR:'):

            self.get_logger().error(
                'Experiment failed: ' + status
            )

            self.waiting = False
            self.completed = True

            return

        if not self.waiting:
            return

        expected = (
            'DONE:'
            + self.commands[self.index]
        )

        # Only continue when the expected action is confirmed.
        if status == expected:

            self.waiting = False
            self.command_start_time = None

            self.get_logger().info(
                f'Step {self.index + 1} completed'
            )

            self.index += 1

            self.publish_current_command()

    def check_timeout(self):

        if self.completed:
            return

        if not self.waiting:
            return

        if self.command_start_time is None:
            return

        elapsed = (
            time.monotonic()
            - self.command_start_time
        )

        # Stop the experiment if an action does not finish in time.
        if elapsed > self.command_timeout:

            command = self.commands[self.index]

            self.get_logger().error(
                f'COMMAND_TIMEOUT: {command} did not complete '
                f'within {self.command_timeout:.1f} seconds'
            )

            self.get_logger().error(
                'Experiment stopped for safety'
            )

            self.waiting = False
            self.completed = True


def main():

    rclpy.init()

    node = RoboMasterExperimentController()

    try:
        # The controller exists only for one experiment cycle.
        # After success or failure, the process exits normally.
        while rclpy.ok() and not node.completed:

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

    except KeyboardInterrupt:

        node.get_logger().info(
            'Experiment stopped by user'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
