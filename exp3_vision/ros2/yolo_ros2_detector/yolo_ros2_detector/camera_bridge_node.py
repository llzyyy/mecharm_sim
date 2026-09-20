import socket
import struct
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraBridgeNode(Node):

    def __init__(self):
        super().__init__('camera_bridge')

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()

        self.host = '0.0.0.0'
        self.port = 5000

        self.running = True

        self.thread = threading.Thread(
            target=self.server_loop,
            daemon=True
        )
        self.thread.start()

        self.get_logger().info(
            f'Camera bridge listening on {self.host}:{self.port}'
        )

    def recv_exact(self, conn, size):
        data = b''

        while len(data) < size:
            packet = conn.recv(size - len(data))

            if not packet:
                return None

            data += packet

        return data

    def server_loop(self):

        server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        server.bind((self.host, self.port))
        server.listen(1)

        while self.running:

            self.get_logger().info(
                'Waiting for Windows camera sender...'
            )

            try:
                conn, addr = server.accept()
            except OSError:
                break

            self.get_logger().info(
                f'Camera connected: {addr}'
            )

            try:
                while self.running:

                    # 前4字节表示JPEG长度
                    header = self.recv_exact(conn, 4)

                    if header is None:
                        break

                    frame_size = struct.unpack('!I', header)[0]

                    jpeg_data = self.recv_exact(
                        conn,
                        frame_size
                    )

                    if jpeg_data is None:
                        break

                    array = np.frombuffer(
                        jpeg_data,
                        dtype=np.uint8
                    )

                    frame = cv2.imdecode(
                        array,
                        cv2.IMREAD_COLOR
                    )

                    if frame is None:
                        continue

                    msg = self.bridge.cv2_to_imgmsg(
                        frame,
                        encoding='bgr8'
                    )

                    msg.header.stamp = (
                        self.get_clock().now().to_msg()
                    )

                    msg.header.frame_id = 'camera'

                    self.publisher.publish(msg)

            except Exception as e:
                self.get_logger().error(str(e))

            finally:
                conn.close()
                self.get_logger().info(
                    'Camera sender disconnected'
                )

        server.close()


def main(args=None):
    rclpy.init(args=args)

    node = CameraBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    