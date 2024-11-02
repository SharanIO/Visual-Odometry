#!/usr/bin/env python3
"""
Camera Calibration Node using ROS and OpenCV.

This script performs camera calibration using a set of images
containing an asymmetric circle grid pattern. It detects the keypoints 
using SimpleBlobDetector and computes the intrinsic camera matrix.

The intrinsic matrix is published to a ROS topic continuously.
"""

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
import rospkg
import random
import os
import imageio

def load_images_from_folder(folder, image_count):
    """
    Load a random set of images from the given folder.

    Parameters:
        folder (str): The directory path containing calibration images.
        image_count (int): Number of images to randomly select for calibration.

    Returns:
        list: List of file paths for the selected images.
    """
    all_files = os.listdir(folder)
    selected_files = random.choices(all_files, k=image_count)
    return [os.path.join(folder, f) for f in selected_files]

def initialize_blob_detector():
    """
    Initialize and configure SimpleBlobDetector with custom parameters.

    Returns:
        cv2.SimpleBlobDetector: Configured blob detector object.
    """
    blob_params = cv2.SimpleBlobDetector_Params()

    # Thresholds for detecting blobs
    blob_params.minThreshold = 8
    blob_params.maxThreshold = 255

    # Filter blobs by size (area)
    blob_params.filterByArea = True
    blob_params.minArea = 64
    blob_params.maxArea = 2500

    # Filter blobs by circularity
    blob_params.filterByCircularity = True
    blob_params.minCircularity = 0.1

    # Filter blobs by convexity
    blob_params.filterByConvexity = True
    blob_params.minConvexity = 0.87

    # Filter blobs by inertia (elongation)
    blob_params.filterByInertia = True
    blob_params.minInertiaRatio = 0.01

    return cv2.SimpleBlobDetector_create(blob_params)

def prepare_object_points():
    """
    Generate 3D object points for the asymmetric circle grid pattern.

    Returns:
        np.ndarray: 3D coordinates of the circle grid points.
    """
    objp = np.zeros((44, 3), np.float32)
    coords = [(x, y) for x in range(0, 361, 36) for y in range(0, 253, 72)]
    for i, (x, y) in enumerate(coords):
        objp[i] = [x, y, 0]  # Z-coordinate is 0 for all points (2D grid)
    return objp

def find_and_draw_keypoints(image, detector):
    """
    Detect keypoints using SimpleBlobDetector and draw them on the image.

    Parameters:
        image (np.ndarray): Grayscale image to detect keypoints.
        detector (cv2.SimpleBlobDetector): Blob detector object.

    Returns:
        np.ndarray: Image with detected keypoints drawn on it.
    """
    keypoints = detector.detect(image)
    return cv2.drawKeypoints(
        image, keypoints, None, (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )

def calibrate_camera(objpoints, imgpoints, image_size):
    """
    Perform camera calibration using object and image points.

    Parameters:
        objpoints (list): List of 3D object points.
        imgpoints (list): List of 2D image points.
        image_size (tuple): Size of the images used for calibration (width, height).

    Returns:
        tuple: Intrinsic matrix, distortion coefficients.
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )
    return mtx, dist

def create_gif_from_images(images, gif_filename='calibration_result.gif', duration=0.5):
    """
    Create a GIF from a sequence of images.

    Parameters:
        images (list): List of file paths for images to be included in the GIF.
        gif_filename (str): Name of the output GIF file.
        duration (float): Duration for each frame in the GIF, in seconds.
    """
    frames = []

    # Read each image and append it to the frames list
    for image_file in images:
        img = cv2.imread(image_file)
        if img is None:
            rospy.logwarn(f"Failed to load image for GIF: {image_file}")
            continue
        frames.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  # Convert BGR to RGB for proper color rendering in GIF

    # Convert images to RGB and append to frames list
    # for img in images:
    #     frames.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  # Convert BGR to RGB for proper color rendering in GIF


    # Write frames to a GIF file
    if frames:
        imageio.mimsave(gif_filename, frames, format='GIF', duration=duration)
        rospy.loginfo(f"GIF saved as: {gif_filename}")
    else:
        rospy.logerr("GIF creation failed: No valid frames found.")

def main():
    """Main function to perform calibration and publish intrinsic matrix."""
    rospy.init_node("calib")  # Initialize the ROS node
    publisher = rospy.Publisher("/car_1/camera/intrinsic", String, queue_size=1)

    # Get the path to the calibration image folder
    rospack = rospkg.RosPack()
    calib_folder = os.path.join(rospack.get_path("stereo_visual_odometry"), "calib")

    # Load random images for calibration based on the ROS parameter
    image_count = rospy.get_param("calib_image_num", 10)
    image_files = load_images_from_folder(calib_folder, image_count)

    # Initialize the blob detector and prepare object points
    blob_detector = initialize_blob_detector()
    objp = prepare_object_points()

    objpoints, imgpoints = [], []  # Lists to store object and image points

    key_images = []
    # Process each selected image for calibration
    for image_file in image_files:
        img = cv2.imread(image_file)
        if img is None:
            rospy.logwarn(f"Failed to load image: {image_file}")
            continue  # Skip if the image couldn't be loaded

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

        # Detect keypoints and draw them on the image
        keypoint_img = find_and_draw_keypoints(gray, blob_detector)
        key_images.append(keypoint_img)

        # Try to find the asymmetric circle grid in the image
        ret, corners = cv2.findCirclesGrid(
            gray, (4, 11), flags=cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING
        )
        if ret:
            objpoints.append(objp)  # Store 3D object points
            imgpoints.append(corners)  # Store 2D image points
            cv2.drawChessboardCorners(img, (4, 11), corners, ret)

        # Optional: Display the keypoint image (for debugging)
        # cv2.imshow('Keypoints', keypoint_img)
        # cv2.waitKey(1)

    # Create a GIF from the selected images
    gif_filename = os.path.join(calib_folder, 'calibration_result.gif')
    create_gif_from_images(image_files, gif_filename)


    if objpoints and imgpoints:
        # Perform camera calibration
        mtx, dist = calibrate_camera(objpoints, imgpoints, gray.shape[::-1])

        # Flatten the intrinsic matrix and format as a string
        intrinsic_str = " ".join(map(str, mtx.flatten()))
        rospy.loginfo(f"Intrinsic Matrix: {intrinsic_str}")

        # Publish the intrinsic matrix continuously
        rate = rospy.Rate(1)  # 1 Hz publishing rate
        while not rospy.is_shutdown():
            publisher.publish(intrinsic_str)
            rate.sleep()
    else:
        rospy.logerr("Calibration failed: No valid object or image points found.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass  # Handle ROS shutdown gracefully
