# Stereo Visual Odometry

Stereo Visual Odometry (SVO) is an advanced, modular computer vision project that accurately estimates camera motion through 3D space by processing sequential stereo images. This implementation uses the ROS framework to perform stereo visual odometry in real-time, taking advantage of stereo camera calibration, depth calculation, and odometry visualization in a unified pipeline. With a focus on robotics applications, this project is designed to be both flexible and efficient, compatible with various stereo datasets and capable of integrating seamlessly with the ROS ecosystem.

To achieve precise motion estimation, the pipeline begins with **stereo camera calibration**, calculating the intrinsic matrix and publishing it as a ROS topic. This calibration step provides a foundation for accurate depth perception, which is then calculated from the stereo images in the ROS bag and published as a dedicated ROS topic. Using the depth data, SVO is performed on the image stream, generating odometry estimations that are visualized in **RViz** alongside the ground truth odometry from the ROS bag.

This project also makes extensive use of **OpenCV** tools to handle feature detection, matching, and depth calculation tasks, ensuring a streamlined integration of image processing with the ROS framework. By combining efficient algorithms with a scalable, ROS-compatible design, this SVO project represents a reliable solution for real-time visual odometry in dynamic environments, ideal for applications in autonomous navigation and SLAM.

## Table of Contents
- [About the Project](#about-the-project)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Examples](#examples)
- [Project Structure](#project-structure)
- [Datasets](#datasets)

## About the Project

This project implements a Stereo Visual Odometry pipeline using stereo image sequences to estimate the position and orientation of a moving camera over time. The goal of the project is to provide an accurate and real-time solution for visual odometry using standard stereo datasets. It serves as a building block for applications in SLAM (Simultaneous Localization and Mapping) and autonomous navigation.

## Features

- **Camera Calibration**: Calculates and publishes the intrinsic matrix as a ROS topic for accurate stereo calibration.
- **Depth Calculation**: Computes depth from stereo images and publishes it as a ROS topic, ensuring depth data is available in real-time.
- **Stereo Visual Odometry (SVO)**: Performs odometry on the stereo image stream from the ROS bag and visualizes both calculated and ROS-bag-provided odometry in RViz.
- **OpenCV Integration**: Utilizes OpenCV for feature detection, matching, and depth calculation, ensuring robust image processing and compatibility with ROS.
