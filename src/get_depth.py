#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage  # ROS message types
from std_msgs.msg import String  # ROS message type for string data
from cv_bridge import CvBridge  # Converts between ROS and OpenCV images
import cv2  # OpenCV for image processing
import numpy as np  # NumPy for numerical operations
import os  # For file operations

class DepthEstimation:
    """
    A ROS node for stereo depth estimation using two cameras.
    This node subscribes to the intrinsic parameters and stereo images,
    computes a depth map, publishes both the raw depth map and its color-mapped visualization,
    and saves video animations of the raw images, depth maps, and color maps.
    """

    def __init__(self):
        rospy.init_node('depth_estimation_node', anonymous=True)  # Initialize the ROS node

        # Set up CvBridge for image conversions
        self.bridge = CvBridge()

        # ROS Subscribers to receive calibration data and stereo images
        rospy.Subscriber('/car_1/camera/intrinsic', String, self.calib_callback)
        rospy.Subscriber('/car_1/camera/left/image_raw/compressed', CompressedImage, self.left_img_callback, queue_size=1)
        rospy.Subscriber('/car_1/camera/right/image_raw/compressed', CompressedImage, self.right_img_callback, queue_size=1)

        # ROS Publishers for the depth map and its color-mapped visualization
        self.depth_pub = rospy.Publisher('/car_1/camera/depth', Image, queue_size=10)
        self.color_map_pub = rospy.Publisher('/car_1/camera/depth_colormap', Image, queue_size=10)

        # Baseline and focal length (to be updated with calibration data)
        self.baseline = 0.07  # Baseline distance between cameras in meters
        self.focal_length = 0.0  # Focal length from intrinsic matrix

        # Storage for the left grayscale image
        self.left_gray = None
        self.frame_size = None  # Initialize frame size to None

        # Video writers for saving animations in MP4 format (initialized later)
        self.raw_video_writer = None
        self.gray_depth_writer = None
        self.colormap_writer = None

    def calib_callback(self, data):
        """Receive and extract the focal length from the intrinsic matrix."""
        matrix = data.data
        res = list(map(float, matrix[0:-1].split(' ')))
        self.focal_length = res[0]  # Assign the focal length

    def left_img_callback(self, data):
        """Receive and convert the left camera image to grayscale."""
        left_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.initialize_video_writers(left_img.shape[:2][::-1])  # Initialize videos if needed
        self.left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)

    def right_img_callback(self, data):
        """Receive the right camera image, compute depth map, and publish visualization."""
        if self.left_gray is None:
            rospy.logwarn("Left image not yet received.")
            return

        # Convert the right camera image to grayscale
        right_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        dst_1 = cv2.GaussianBlur(self.left_gray, (7, 7), 1.5)
        dst_2 = cv2.GaussianBlur(right_gray, (7, 7), 1.5)

        # Compute disparity using StereoSGBM
        stereo = cv2.StereoSGBM_create(numDisparities=16, blockSize=5)
        disparity = stereo.compute(dst_1, dst_2).astype(np.float32) / 16.0

        # Compute depth map
        value = self.focal_length * self.baseline
        depth = np.where(disparity > 0, value / (disparity + 1e-6), 0)
        depth = cv2.medianBlur(depth, 5)  # Reduce noise

        # Normalize depth map for visualization
        norm_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        color_map = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)

        # Publish the raw depth map and color-mapped visualization
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth, "32FC1"))
        self.color_map_pub.publish(self.bridge.cv2_to_imgmsg(color_map, "bgr8"))

        # Save the frames to video
        self.raw_video_writer.write(right_img)
        self.gray_depth_writer.write(norm_depth)
        self.colormap_writer.write(color_map)

    def initialize_video_writers(self, frame_size):
        """Initialize the video writers with the correct frame size."""
        if self.raw_video_writer is None:
            video_dir = "/home/sharan/Projects/ROS_Projects/VO_ws/src/stereo_visual_odometry/src"
            os.makedirs(video_dir, exist_ok=True)

            self.raw_video_writer = cv2.VideoWriter(
                f"{video_dir}/raw_images.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 10, frame_size
            )
            self.gray_depth_writer = cv2.VideoWriter(
                f"{video_dir}/gray_depth.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 10, frame_size, isColor=False
            )
            self.colormap_writer = cv2.VideoWriter(
                f"{video_dir}/depth_colormap.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 10, frame_size
            )

    def run(self):
        """Main loop to keep the node running."""
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            rate.sleep()

    def __del__(self):
        """Release video writer resources."""
        if self.raw_video_writer:
            self.raw_video_writer.release()
        if self.gray_depth_writer:
            self.gray_depth_writer.release()
        if self.colormap_writer:
            self.colormap_writer.release()

# Main entry point of the program
if __name__ == '__main__':
    depth_obj = DepthEstimation()
    try:
        depth_obj.run()
    except rospy.ROSInterruptException:
        pass
