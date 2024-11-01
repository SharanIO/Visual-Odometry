# Stereo Visual Odometry

Stereo Visual Odometry (SVO) is a computer vision project that estimates the motion of a camera through 3D space by analyzing the sequence of images from stereo camera inputs. This project is designed to be modular, efficient, and capable of working with different stereo datasets.

## Table of Contents
- [About the Project](#about-the-project)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Examples](#examples)
- [Project Structure](#project-structure)
- [Datasets](#datasets)
- [Contributing](#contributing)
- [License](#license)

## About the Project

This project implements a Stereo Visual Odometry pipeline using stereo image sequences to estimate the position and orientation of a moving camera over time. The goal of the project is to provide an accurate and real-time solution for visual odometry using standard stereo datasets. It serves as a building block for applications in SLAM (Simultaneous Localization and Mapping) and autonomous navigation.

### Key Objectives
- Real-time estimation of camera position and orientation.
- Flexibility to work with various stereo camera configurations.
- Modular and extensible architecture for testing and improving algorithms.

## Features

- **Feature Detection**: Detects features in stereo images using algorithms like ORB, SIFT, or SURF.
- **Feature Matching**: Matches features between consecutive frames to estimate motion.
- **Pose Estimation**: Calculates the camera’s position and orientation from matched features.
- **Scale Recovery**: Uses stereo information to estimate scale in real-world measurements.
- **Visualization**: Displays the trajectory and keyframes of the estimated motion path.

## Installation

### Prerequisites

- Python 3.x
- OpenCV
- NumPy
- Matplotlib
- [Dataset](#datasets)

### Setup

1. **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/stereo-visual-odometry.git
    cd stereo-visual-odometry
    ```

2. **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

## Usage

### Running the Main Script

To run the visual odometry pipeline, use the following command:
```bash
python main.py --dataset_path /path/to/dataset
