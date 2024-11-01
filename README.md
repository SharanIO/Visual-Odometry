# Stereo Visual Odometry

Stereo Visual Odometry (SVO) is an advanced, modular computer vision project that accurately estimates camera motion through 3D space by processing sequential stereo images. This implementation uses the ROS framework to perform stereo visual odometry in real-time, taking advantage of stereo camera calibration, depth calculation, and odometry visualization in a unified pipeline.

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

1. **Camera Calibration**: Computes intrinsic camera parameters via an asymmetric square grid pattern.
2. **Depth Estimation**: Real-time computation of depth maps from stereo image feeds, critical for robotic depth perception.
3. **Stereo Visual Odometry (SVO)**: Position and orientation estimation based on feature matching in stereo images, essential for localization and navigation.

### Technical Contributions

### 1. Development of a ROS-integrated Camera Calibration Node

Using an asymmetric square grid pattern and SimpleBlobDetector in OpenCV, the node computes the camera's intrinsic matrix, crucial for precise depth estimation and odometry.

- **Asymmetric Square Grid Pattern**: This pattern is used for accurate camera calibration. The grid pattern is detected using OpenCV's SimpleBlobDetector.
- **Intrinsic Matrix Calculation**: The intrinsic parameters of the camera, such as focal length and optical center, are calculated and published as a ROS topic.

![Camera Calibration](path/to/camera_calibration_image.png)

### 2. Depth Estimation using Stereo Vision

Depth maps are generated using the disparity between stereo images. A robust configuration of StereoSGBM is used for improved accuracy in noisy environments.

- **Disparity Calculation**: Disparity between the left and right stereo images is calculated using StereoSGBM (Semi-Global Block Matching).
- **Depth Map Generation**: The disparity map is converted into a depth map, which provides the distance of objects from the camera.

![Depth Estimation](path/to/depth_estimation_image.png)

### 3. Visual Odometry for 3D Localization

Implemented a feature tracking-based stereo visual odometry pipeline. The system calculates the robot's 3D position and orientation with feature detection, matching, and triangulation techniques, which are critical for continuous localization.

- **Feature Detection**: Key points in the images are detected using the ORB (Oriented FAST and Rotated BRIEF) algorithm. ORB is chosen for its efficiency and robustness in detecting and describing key points, making it suitable for real-time applications.

    **Method**:
    
      - Convert the image to grayscale.
      - Use the ORB detector to find key points and compute their descriptors.

        ```bash
        def detect_features(self, img):
            orb = cv2.ORB_create()
            kp, des = orb.detectAndCompute(img, None)
            return kp, des
        ```

- **Feature Matching**: Features between consecutive frames are matched using the BFMatcher (Brute Force Matcher) with Hamming distance and cross-checking. BFMatcher is selected for its simplicity and effectiveness in matching binary descriptors like those produced by ORB.
- **Triangulation**: The 3D positions of matched features are estimated using triangulation. This step is crucial for determining the spatial coordinates of the features in the scene.
- **Pose Estimation**: The camera's position and orientation are estimated using the PnP (Perspective-n-Point) algorithm with RANSAC. PnP is used for its ability to estimate the pose of the camera from 3D-2D point correspondences.

![Visual Odometry](path/to/visual_odometry_image.gif)

### 4. ROS Integration and Real-time Data Handling

Leveraged ROS publishers and subscribers to ensure smooth communication between nodes, demonstrating expertise in real-time data management in robotics software.

- **ROS Publishers and Subscribers**: Used for real-time data handling and communication between nodes.
- **RViz Visualization**: Depth maps and odometry estimations are visualized in RViz.

![ROS Integration](path/to/ros_integration_image.png)

These contributions collectively provide a robust and efficient pipeline for stereo visual odometry, enabling accurate 3D localization and navigation in dynamic environments.

## Setup and Installation

1. **Clone the Repository**

    ```bash
    git clone https://github.com/yourusername/stereo_visual_odometry.git
    cd stereo_visual_odometry
    ```

2. **Install Dependencies** Ensure that OpenCV and additional Python libraries are installed:

    ```bash
    sudo apt install python3-pip
    pip3 install numpy opencv-python rospkg
    ```

3. **Build the ROS Workspace**

    ```bash
    catkin_make
    source devel/setup.bash
    ```

## Usage

1. **Launch the Calibration Node**

    ```bash
    roslaunch stereo_visual_odometry calibrate.launch
    ```

2. **Launch the Depth Estimation Node**

    ```bash
    roslaunch stereo_visual_odometry depth_estimation.launch
    ```

3. **Launch the SVO Node**

    ```bash
    roslaunch stereo_visual_odometry svo.launch
    ```

## Examples

- **RViz Visualization**: Visualize the depth maps and odometry estimations in RViz using the provided configuration files in the `rviz` directory.

## Project Structure

- `calib/`: Calibration files and data.
- `launch/`: ROS launch files.
- `rviz/`: RViz configuration files.
- `src/`: Source code for the ROS nodes.
  - `calibrate.py`: Node for camera calibration.
  - `get_depth.py`: Node for depth estimation.
  - `svo.py`: Node for stereo visual odometry.

## Datasets

- Ensure you have the appropriate stereo datasets for testing and validation of the SVO pipeline.
