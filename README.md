# ps_robotik
## Abstract:
The primary objective of this study is to present the
development of an AI-driven vehicle, JetBot AI, leveraging the
Nvidia Jetson Nano module. The focus is on implementing crucial
functionalities, including autonomous obstacle detection utilizing
camera inputs, navigation path planning, and vehicle motion
control. Through the analysis of environmental data, a safe
navigation path is generated to avoid obstacles effectively, and
the JetBot vehicle is accurately directed to execute the necessary
actions. The research incorporates advanced methodologies such
as map creation, obstacle identification, and obstacle avoidance
algorithms. The paper elaborates on the methods and procedures
employed to facilitate object detection, obstacle avoidance, and
path planning for the JetBot vehicle, employing mapping, tracing,
and sorting objects

---

# 🚗 JetBot: AI-Powered Autonomous Robot on NVIDIA Jetson Nano

![JetBot Demo](https://github.com/husamhamu/ps_robotik/assets/demo.gif) <!-- Replace with actual demo GIF or video link -->

> **JetBot** is an AI-powered autonomous robot built on the NVIDIA Jetson Nano, featuring deep learning object detection, obstacle localization via Apriltags, real-time path planning, and autonomous navigation through RRT algorithms and PID motor control.

## 🧠 Core Features

- 🔍 **Object Detection**: Integrated and benchmarked SSD, YOLOv7, and YOLOv3 for real-time object recognition on embedded hardware.
- 🧭 **Navigation & Obstacle Avoidance**: Localizes obstacles using camera calibration + Apriltag detection, and avoids them using advanced coordinate transformations and motion planning.
- 🗺️ **Path Planning**: Implements RRT and RRT* (Rapidly-exploring Random Tree) algorithms for dynamic pathfinding in a cluttered arena.
- 🤖 **Robot Control**: Uses a PID controller to guide motor behavior, based on camera input and object location.
- 📡 **ROS Integration**: Fully integrated with the Robot Operating System (ROS) for real-time data handling and visualization (rviz, rqt).
- 🔧 **Hardware Optimized**: Deployed on resource-constrained Jetson Nano using Docker and virtual memory (SWAP) optimization for heavy ML inference.

## 📚 Project Structure

```
jetbot/
│
├── deep_learning/              # YOLOv7, SSD, dataset handling & training
├── apriltag/                   # AprilTag setup, calibration, ROS integration
├── path_planning/             # RRT algorithm, coordinate transformation
├── motor_control/             # PID-based differential drive control
├── scripts/                   # ROS nodes & launch files
└── visualization/             # Real-time maps, rviz config, coordinate logs
```

## 📈 Visual Demo

### Object Detection (YOLOv3)
![YOLO Object Detection]![image](https://github.com/user-attachments/assets/67f3d23c-4762-4822-a489-80e233cc2c36)
 <!-- Replace with real screenshot -->

### Path Planning with RRT
![image](https://github.com/user-attachments/assets/cde0e555-9621-4ee4-8148-f3ca9b65fe1f)
 <!-- Replace with real screenshot -->

### AprilTag Localization Flow
![image](https://github.com/user-attachments/assets/aff197d6-ce69-448d-9e1d-6737363f098d)
 <!-- Replace with real screenshot -->

### Requirements
- Jetson Nano with camera module
- Ubuntu 20.04 + ROS Noetic
- Python 3.8, PyTorch, OpenCV
- Docker (for efficient training)


## 🧪 Tasks & Achievements

| Task | Description |
|------|-------------|
| Task 1 | Object recognition and 2D map generation using YOLO and Apriltags |
| Task 2 | Navigate around red (CW) and blue (CCW) cubes, avoid obstacles |
| Task 3 | Push dice into matching-colored bases and return to origin |

> 🏁 Achieved < 8 cm object localization error and ~2s image inference latency.

## 🧠 Technical Highlights

- 🤖 **YOLOv3/YOLOv7** with Roboflow preprocessing
- 🧮 **Perspective transformation** for 3D localization from 2D image
- 📐 **Camera calibration** using ROS tools and chessboard grid
- 🧭 **Proportional control** loop for smooth navigation
- 🔄 **TF tree & coordinate system transforms** with ROS

## 🛠 Technologies Used

![Python](https://img.shields.io/badge/Python-3.8-blue)
![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Jetson Nano](https://img.shields.io/badge/Platform-JetsonNano-green)
![YOLO](https://img.shields.io/badge/DeepLearning-YOLOv7-brightgreen)
![RRT](https://img.shields.io/badge/Algorithm-RRT%20%2B%20RRT*-orange)

## 👨‍💻 Contributors

- **Husam Hamu** – M.Sc. Mechatronics, TU Darmstadt  

[Full paper (PDF)](./docs/JetBot_Project_Report.pdf) <!-- Add path if uploaded -->

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
