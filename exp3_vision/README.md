\# Experiment 3 - Vision Module



Author: insomnia12270319



本目录为实验三“桌面物体自动分类整理”的视觉识别模块。



主要内容：

\- YOLO 目标检测模型

\- Windows USB 摄像头采集

\- Windows 到 WSL 图像传输

\- ROS 2 图像发布

\- ROS 2 YOLO 检测结果发布



\## Directory



exp3\_vision/

├── config/

│   └── classes.txt

├── models/

│   └── best\_v3.pt

├── ros2/

│   └── yolo\_ros2\_detector/

└── windows/

&#x20;   └── camera\_test.py



\## Model



模型文件：

best\_v3.pt



类别：

0 phone

1 cup

2 plug

3 cube



实验三实际主要使用：

cup -> bottle / 饮料瓶类

cube -> cube / 方块类



plug 和 phone 可由后续任务控制模块按需要忽略。



注意：

模型中的 cup 是为了兼容原始数据集保留的类别名称，

在本实验中主要表示瓶装饮料、水瓶、咖啡瓶等目标。



\## Dataset



dataset\_v3：



Total images : 1144

Train        : 915

Validation   : 229



标注框数量：



cup  : 641

plug : 207

cube : 423



\## Validation



best\_v3.pt：



all:

P=0.993

R=0.987

mAP50=0.994

mAP50-95=0.934



cup:

P=0.992

R=0.977

mAP50=0.994

mAP50-95=0.955



plug:

P=1.000

R=0.995

mAP50=0.995

mAP50-95=0.873



cube:

P=0.987

R=0.988

mAP50=0.992

mAP50-95=0.973



dataset\_v3 验证集中没有 phone 样本，

因此以上结果不代表 phone 类别的 v3 验证性能。



\## ROS 2 Interface



输入：



/camera/image\_raw

sensor\_msgs/msg/Image



输出：



/yolo/detections

vision\_msgs/msg/Detection2DArray



检测结果包含：

\- class\_id

\- confidence

\- bbox center

\- bbox size



\## Windows Camera



Windows 端运行：



conda activate yolo-project

python camera\_test.py



TCP：

host = 127.0.0.1

port = 5000



Windows USB 摄像头图像通过 TCP JPEG 发送至 WSL。



\## ROS 2



启动 camera bridge：



ros2 run yolo\_ros2\_detector camera\_bridge



启动 YOLO detector：



ros2 run yolo\_ros2\_detector yolo\_detector



数据链：



Windows USB Camera

→ TCP JPEG

→ camera\_bridge

→ /camera/image\_raw

→ YOLO detector

→ /yolo/detections



\## Model Path



当前代码保留实验过程中实际使用的模型路径。



WSL：

/home/lenovo/exp3\_ws/models/best.pt



Windows：

C:\\Users\\Lenovo\\Desktop\\exp3windows\\best\_v3.pt



在其他电脑运行时，需要修改为对应电脑上的模型路径。



\## Environment



Windows 11

WSL2 Ubuntu 22.04

ROS 2 Humble

Python 3.10

Ultralytics YOLO

NVIDIA RTX 4060 Laptop GPU

