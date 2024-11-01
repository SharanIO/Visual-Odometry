# 📷 Stereo Visual Odometry 🚀

This project demonstrates a **Stereo Visual Odometry** system built with ROS, combining **camera calibration**, **depth estimation**, and **odometry calculation** to enable precise perception and navigation capabilities. It’s designed to highlight my expertise in robotics software, especially in real-time perception and data processing.

---

## 🛠️ Skills Used

| **Skill Areas**        | **Tools & Frameworks** |
|------------------------|------------------------|
| **ROS Programming**    | ROS Noetic, ROS Bags   |
| **Computer Vision**    | OpenCV, Depth Estimation |
| **Image Processing**   | Stereo Matching        |
| **Odometry**           | RViz Visualization     |
| **Data Management**    | Camera Calibration     |
| **Real-Time Systems**  | ROS Nodes and Topics   |

This project serves as a foundation for more complex applications like autonomous navigation, robotic manipulation, and 3D mapping, all essential to real-world robotics. It showcases my ability to build robust, modular systems—a skill set directly applicable to robotics software engineering roles.

---

## 📖 Project Overview

Stereo visual odometry estimates a camera-equipped robot’s position and orientation by analyzing stereo image pairs. By deriving depth from stereo images, we can compute incremental motion over time, allowing the robot to navigate and interact in dynamic environments. This README details the technical breakdown of the project's three main components:

---

### ⚙️ Components

1. **Camera Calibration**
   - **Purpose**: Captures intrinsic and extrinsic camera parameters using a checkerboard pattern to correct image alignment.
   - **Published Topic**: `/camera_calibration`
   - **Command**:
     ```bash
     roslaunch your_package_name camera_calibration.launch
     ```

2. **Depth Estimation**
   - **Purpose**: Converts stereo images into depth data, providing valuable 3D information for robot perception.
   - **Published Topic**: `/depth_estimation`
   - **Command**:
     ```bash
     roslaunch your_package_name depth_estimation.launch
     ```

3. **Stereo Visual Odometry**
   - **Purpose**: Computes the robot's motion by analyzing depth and motion cues from stereo frames.
   - **Published Topic**: `/odometry_data`
   - **Visualization**: RViz (with additional ROS bag replay for robustness testing)
   - **Command**:
     ```bash
     roslaunch your_package_name stereo_visual_odometry.launch
     ```

Each component publishes data as ROS topics, supporting real-time processing and enabling integration with other robotic systems for a seamless operational workflow.

---

## 📂 Project Structure

