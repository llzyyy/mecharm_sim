import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ultralytics import YOLO
import torch


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # 模型
        model_path = '/home/lenovo/exp3_ws/models/best.pt'
        self.model = YOLO(model_path)

        # GPU / CPU
        self.device = 0 if torch.cuda.is_available() else 'cpu'

        self.get_logger().info(f'YOLO model loaded: {model_path}')
        self.get_logger().info(f'Classes: {self.model.names}')
        self.get_logger().info(
            f'Device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"}'
        )

        self.bridge = CvBridge()

        # 订阅摄像头图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 发布检测结果
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )

        self.get_logger().info('Waiting for /camera/image_raw ...')


    def image_callback(self, msg):

        # ROS Image -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # YOLO
        results = self.model(
            frame,
            conf=0.25,
            device=self.device,
            verbose=False
        )

        result = results[0]

        # Detection2DArray
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in result.boxes:

            cls_id = int(box.cls[0])
            score = float(box.conf[0])

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            width = x2 - x1
            height = y2 - y1

            detection = Detection2D()

            detection.header = msg.header

            # bbox
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.center.theta = 0.0

            detection.bbox.size_x = width
            detection.bbox.size_y = height

            # class + confidence
            hypothesis = ObjectHypothesisWithPose()

            hypothesis.hypothesis.class_id = str(
                result.names[cls_id]
            )

            hypothesis.hypothesis.score = score

            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        self.detection_pub.publish(detection_array)

        if len(detection_array.detections) > 0:
            self.get_logger().info(
                f'Detected {len(detection_array.detections)} objects'
            )


def main(args=None):

    rclpy.init(args=args)

    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()