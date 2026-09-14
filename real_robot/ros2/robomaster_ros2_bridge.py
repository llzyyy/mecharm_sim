import socket
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point


class RoboMasterBridge(Node):

    def __init__(self):
        super().__init__('robomaster_bridge')

        # Load the network settings used by the bridge.
        config_path = (
            Path(__file__).resolve().parent.parent
            / 'config'
            / 'robomaster_params.yaml'
        )

        with open(config_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)

        bridge_config = config['bridge']

        self.status_port = int(
            bridge_config['status_port']
        )

        self.command_port = int(
            bridge_config['command_port']
        )

        self.windows_address = (
            bridge_config['windows_address'],
            self.command_port
        )

        self.get_logger().info(
            f'Loaded configuration: {config_path}'
        )

        # READY, DONE and ERROR are published here.
        self.status_publisher = self.create_publisher(
            String,
            '/robomaster/status',
            10
        )

        # The real arm position is kept on a separate topic.
        self.position_publisher = self.create_publisher(
            Point,
            '/robomaster/arm_position',
            10
        )

        # Commands from the experiment controller arrive here.
        self.command_subscriber = self.create_subscription(
            String,
            '/robomaster/command',
            self.command_callback,
            10
        )

        # Receive status and arm position from the Windows adapter.
        self.status_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.status_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.status_socket.bind(
            ('0.0.0.0', self.status_port)
        )

        self.status_socket.setblocking(False)

        # Send ROS 2 commands to the Windows adapter.
        self.command_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        # Check the UDP socket regularly without blocking ROS 2.
        self.timer = self.create_timer(
            0.05,
            self.receive_status
        )

        self.get_logger().info(
            'RoboMaster ROS2 bidirectional bridge started'
        )

        self.get_logger().info(
            f'Status port: {self.status_port}'
        )

        self.get_logger().info(
            f'Command destination: '
            f'{self.windows_address[0]}:{self.command_port}'
        )

        self.get_logger().info(
            'Live arm position topic: /robomaster/arm_position'
        )

    def command_callback(self, msg):

        command = msg.data

        self.get_logger().info(
            f'ROS2 command: {command}'
        )

        try:
            # Forward the ROS 2 command to the hardware adapter.
            self.command_socket.sendto(
                command.encode('utf-8'),
                self.windows_address
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to send command: {e}'
            )

    def receive_status(self):

        try:
            data, address = self.status_socket.recvfrom(
                1024
            )

        except BlockingIOError:
            return

        except Exception as e:
            self.get_logger().error(
                f'UDP receive error: {e}'
            )
            return

        text = data.decode(
            'utf-8'
        ).strip()

        # Position messages are converted into a ROS 2 Point message.
        if text.startswith('ARM_POSITION:'):

            try:
                position_text = text.split(
                    ':',
                    1
                )[1]

                x_text, y_text = position_text.split(
                    ',',
                    1
                )

                position_msg = Point()

                position_msg.x = float(x_text)
                position_msg.y = float(y_text)
                position_msg.z = 0.0

                self.position_publisher.publish(
                    position_msg
                )

            except Exception as e:
                self.get_logger().warning(
                    f'Invalid arm position message: {text} ({e})'
                )

            return

        # Other messages are task-level status messages.
        status_msg = String()
        status_msg.data = text

        self.status_publisher.publish(
            status_msg
        )

        self.get_logger().info(
            f'Robot status: {text}'
        )

    def destroy_node(self):

        # Close both UDP sockets when the bridge stops.
        try:
            self.status_socket.close()
        except Exception:
            pass

        try:
            self.command_socket.close()
        except Exception:
            pass

        super().destroy_node()


def main():

    rclpy.init()

    node = RoboMasterBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Bridge stopped by user'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
