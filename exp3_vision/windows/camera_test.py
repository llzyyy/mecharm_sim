import cv2
from ultralytics import YOLO
from pathlib import Path
import socket
import struct


# =========================
# 配置
# =========================

CAMERA_ID = 0

MODEL_PATH = Path(
    r"C:\Users\Lenovo\Desktop\exp3windows\best_v3.pt"
)

CONF = 0.15

# WSL camera_bridge 地址
WSL_IP = "127.0.0.1"
PORT = 5000

# JPEG 压缩质量
JPEG_QUALITY = 80


# =========================
# 连接 WSL
# =========================

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

print(f"正在连接 WSL {WSL_IP}:{PORT} ...")

try:
    sock.connect((WSL_IP, PORT))
except Exception as e:
    print("连接 WSL 失败：", e)
    print("请确认 WSL 中 camera_bridge 已经启动。")
    exit()

print("WSL 连接成功")


# =========================
# 加载模型
# =========================

print("模型路径:", MODEL_PATH)

if not MODEL_PATH.exists():
    print("best.pt 不存在！")
    sock.close()
    exit()

model = YOLO(str(MODEL_PATH))

print("模型加载成功")
print("模型类别:", model.names)


# =========================
# 打开摄像头
# =========================

cap = cv2.VideoCapture(
    CAMERA_ID,
    cv2.CAP_DSHOW
)

if not cap.isOpened():
    print(f"无法打开摄像头 {CAMERA_ID}")
    print("尝试把 CAMERA_ID 改成 0、1 或 2")
    sock.close()
    exit()

cap.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    1280
)

cap.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    720
)

print("摄像头打开成功")
print("正在发送图像到 ROS 2")
print("按 Q 退出")


# =========================
# 实时循环
# =========================

try:

    while True:

        ret, frame = cap.read()

        if not ret:
            print("读取摄像头失败")
            break


        # =====================================
        # 1. 把原始图像发送到 WSL
        # =====================================

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                JPEG_QUALITY
            ]
        )

        if ok:

            data = encoded.tobytes()

            # 先发 4 字节长度
            header = struct.pack(
                "!I",
                len(data)
            )

            try:
                sock.sendall(header)
                sock.sendall(data)

            except (
                BrokenPipeError,
                ConnectionResetError,
                ConnectionAbortedError
            ):
                print("与 WSL 的连接已断开")
                break


        # =====================================
        # 2. Windows 本地 YOLO 推理
        # =====================================

        results = model.predict(
            source=frame,
            conf=CONF,
            imgsz=640,
            verbose=False
        )

        result = results[0]

        detection_count = len(result.boxes)


        # =====================================
        # 3. 手动画框
        # =====================================

        for box in result.boxes:

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0].tolist()
            )

            cls_id = int(
                box.cls[0]
            )

            confidence = float(
                box.conf[0]
            )

            class_name = model.names[
                cls_id
            ]

            label = (
                f"{class_name} "
                f"{confidence:.2f}"
            )


            # 画框
            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                3
            )


            # 类别 + 置信度
            cv2.putText(
                frame,
                label,
                (
                    x1,
                    max(y1 - 10, 30)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )


            # 输出到终端
            print(
                f"class={class_name}, "
                f"conf={confidence:.3f}, "
                f"box=({x1},{y1},{x2},{y2})"
            )


        # =====================================
        # 4. 显示检测状态
        # =====================================

        cv2.putText(
            frame,
            f"Detections: {detection_count}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2
        )


        cv2.putText(
            frame,
            "Streaming to ROS2",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )


        if detection_count == 0:

            cv2.putText(
                frame,
                "NO DETECTION",
                (10, 115),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )


        # =========================
# 缩小显示窗口，方便截图
# =========================

        display_frame = cv2.resize(
            frame,
            (600, 380)
        )

        cv2.imshow(
            "YOLO best.pt Test",
            display_frame
        )


        if (
            cv2.waitKey(1) & 0xFF
            == ord("q")
        ):
            break


except KeyboardInterrupt:
    print("用户中断程序")


finally:

    cap.release()

    sock.close()

    cv2.destroyAllWindows()

    print("程序结束")