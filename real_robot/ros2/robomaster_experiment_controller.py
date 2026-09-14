import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class RoboMasterExperimentController(Node):

    def __init__(self):
        super().__init__(
            'robomaster_experiment_controller'
        )

        self.command_publisher = self.create_publisher(
            String,
            '/robomaster/command',
            10
        )

        self.status_subscriber = self.create_subscription(
            String,
            '/robomaster/status',
            self.status_callback,
            10
        )

        self.commands = [
            "OPEN_GRIPPER",
            "ARM_DOWN",
            "CLOSE_GRIPPER",
            "ARM_UP",
            "ROTATE",
            "ARM_DOWN",
            "OPEN_GRIPPER",
            "ARM_UP"
        ]

        self.index = 0
        self.started = False
        self.waiting = False
        self.completed = False

        self.get_logger().info(
            "ROS2 RoboMaster experiment controller ready"
        )

        self.get_logger().info(
            "Waiting for RoboMaster READY..."
        )

    def publish_current_command(self):

        if self.index >= len(self.commands):

            self.completed = True

            self.get_logger().info(
                "EXPERIMENT_SUCCESS"
            )

            self.get_logger().info(
                "All ROS2 commands completed successfully"
            )

            return

        command = self.commands[self.index]

        msg = String()
        msg.data = command

        self.command_publisher.publish(msg)

        self.waiting = True

        self.get_logger().info(
            f"[ROS2 PUBLISH] {command}"
        )

    def status_callback(self, msg):

        status = msg.data

        self.get_logger().info(
            f"[ROBOT STATUS] {status}"
        )

        if self.completed:
            return

        if status == "READY":

            if not self.started:

                self.started = True

                self.get_logger().info(
                    "RoboMaster READY - starting experiment"
                )

                self.publish_current_command()

            return

        if status.startswith("ERROR:"):

            self.get_logger().error(
                "Experiment failed: " + status
            )

            self.completed = True
            return

        if not self.waiting:
            return

        expected = (
            "DONE:" +
            self.commands[self.index]
        )

        if status == expected:

            self.waiting = False

            self.get_logger().info(
                f"Step {self.index + 1} completed"
            )

            self.index += 1

            self.publish_current_command()


def main():

    rclpy.init()

    node = RoboMasterExperimentController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
