import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RoboMasterBridge(Node):

    def __init__(self):
        super().__init__('robomaster_bridge')

        # RoboMaster -> ROS2 状态
        self.status_publisher = self.create_publisher(
            String,
            '/robomaster/status',
            10
        )

        # ROS2 -> RoboMaster 指令
        self.command_subscriber = self.create_subscription(
            String,
            '/robomaster/command',
            self.command_callback,
            10
        )

        # 接收 Windows 返回的状态
        self.status_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.status_socket.bind(('0.0.0.0', 5005))
        self.status_socket.setblocking(False)

        # 向 Windows 发送控制指令
        self.command_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.windows_address = (
            '127.0.0.1',
            5006
        )

        self.timer = self.create_timer(
            0.05,
            self.receive_status
        )

        self.get_logger().info(
            'RoboMaster ROS2 bidirectional bridge started'
        )

    def command_callback(self, msg):

        command = msg.data

        self.get_logger().info(
            f'ROS2 command: {command}'
        )

        self.command_socket.sendto(
            command.encode('utf-8'),
            self.windows_address
        )

    def receive_status(self):

        try:
            data, address = self.status_socket.recvfrom(1024)
        except BlockingIOError:
            return

        text = data.decode('utf-8')

        msg = String()
        msg.data = text

        self.status_publisher.publish(msg)

        self.get_logger().info(
            f'Robot status: {text}'
        )


def main():

    rclpy.init()

    node = RoboMasterBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
