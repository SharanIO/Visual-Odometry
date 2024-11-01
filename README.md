# Stereo Visual Odometry

Stereo Visual Odometry (SVO) is an advanced, modular computer vision project that accurately estimates camera motion through 3D space by processing sequential stereo images. This implementation uses the ROS framework to perform stereo visual odometry in real-time, taking advantage of stereo camera calibration, depth calculation, and odometry visualization in a unified pipeline. With a focus on robotics applications, this project is designed to be both flexible and efficient, compatible with various stereo datasets and capable of integrating seamlessly with the ROS ecosystem.

To achieve precise motion estimation, the pipeline begins with **stereo camera calibration**, calculating the intrinsic matrix and publishing it as a ROS topic. This calibration step provides a foundation for accurate depth perception, which is then calculated from the stereo images in the ROS bag and published as a dedicated ROS topic. Using the depth data, SVO is performed on the image stream, generating odometry estimations that are visualized in **RViz** alongside the ground truth odometry from the ROS bag.

This project also makes extensive use of **OpenCV** tools to handle feature detection, matching, and depth calculation tasks, ensuring a streamlined integration of image processing with the ROS framework. By combining efficient algorithms with a scalable, ROS-compatible design, this SVO project represents a reliable solution for real-time visual odometry in dynamic environments, ideal for applications in autonomous navigation and SLAM.

## Table of Contents
- [About the Project](#about-the-project)
- [Technical Contributions](#technical-contributions)
- [Setup and Installation](#setup-and-installation)
- [Usage](#usage)
- [Examples](#examples)
- [Project Structure](#project-structure)
- [Datasets](#datasets)

## About the Project

This project represents a complete solution for stereo-based perception in robotics, using stereo visual odometry and real-time depth estimation for enhanced spatial understanding. This work has been implemented as ROS nodes, handling:

1. **Camera Calibration**: Computes intrinsic camera parameters via an asymmetric circle grid pattern.
2. **Depth Estimation**: Real-time computation of depth maps from stereo image feeds, critical for robotic depth perception.
3. **Stereo Visual Odometry (SVO)**: Position and orientation estimation based on feature matching in stereo images, essential for localization and navigation.


## Technical Contributions

- **Development of a ROS-integrated camera calibration node**: Using an asymmetric circle grid pattern and SimpleBlobDetector in OpenCV, the node computes the camera's intrinsic matrix, crucial for precise depth estimation and odometry.
  
- **Depth estimation using stereo vision**: Depth maps are generated using disparity between stereo images. A robust configuration of StereoSGBM is used for improved accuracy in noisy environments.
  
- **Visual Odometry for 3D localization**: Implemented a feature tracking-based stereo visual odometry pipeline. The system calculates the robot's 3D position and orientation with feature detection, matching, and triangulation techniques, which are critical for continuous localization.

- **ROS integration and real-time data handling**: Leveraged ROS publishers and subscribers to ensure smooth communication between nodes, demonstrating expertise in real-time data management in robotics software.


## Setup and Installation

1. **Clone the Repository**
    ```bash
    git clone https://github.com/yourusername/stereo_visual_odometry.git
    cd stereo_visual_odometry

2. **Install Dependencies** Ensure that ROS, OpenCV, and additional Python libraries are installed:
    ```bash
    sudo apt install ros-noetic-desktop-full
    pip install numpy opencv-python rospkg

3. **Build the ROS Workspace**
    ```bash
    catkin build
    source devel/setup.bash

