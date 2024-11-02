#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

class StereoVisualOdometry:
    """
    A class implementing Stereo Visual Odometry using ROS.
    This class subscribes to stereo camera feeds, processes images to detect features,
    computes the robot's position and orientation, and publishes the odometry and marker data.
    """

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('stereo_visual_odometry_node', anonymous=True)

        # Initialize CVBridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # ROS Subscribers for intrinsic parameters, odometry, and camera feeds
        self.intrinsic_sub = rospy.Subscriber('/car_1/camera/intrinsic', String, self.intrinsic_callback)
        self.odom_sub = rospy.Subscriber('/car_1/base/odom', Odometry, self.odom_callback)
        self.left_img_sub = rospy.Subscriber('/car_1/camera/left/image_raw/compressed', CompressedImage, self.left_img_callback)
        self.right_img_sub = rospy.Subscriber('/car_1/camera/right/image_raw/compressed', CompressedImage, self.right_img_callback)

        # ROS Publishers for odometry and markers
        self.odom_pub = rospy.Publisher('/car_1/odom', Odometry, queue_size=10)
        self.mark_pub = rospy.Publisher('/car_1/ref', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/car_1/path', Marker, queue_size=10)

        # Initialize variables for storing images and pose information
        self.left = []  # Store left camera images
        self.right = []  # Store right camera images
        self.prev = np.eye(4)  # Previous transformation matrix (identity initially)
        self.intrinsic = 0  # Camera intrinsic matrix placeholder
        self.fx = self.fy = self.cx = self.cy = 0  # Intrinsic parameters
        self.ox = self.oy = 0  # Robot's current position from odometry
        self.baseline = 0.07  # Baseline distance between stereo cameras

        # Initialize Odometry and Marker messages
        self.odom_msg = Odometry()
        self.marker = Marker()
        self.path_marker = Marker()

        # Set message headers and types
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.twist.twist = Twist()
        self.marker.header.frame_id = "odom"
        self.marker.type = self.marker.LINE_STRIP

        # Initialize path marker properties
        self.path_marker.header.frame_id = "odom"
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.scale.x = 0.05  # Line width
        self.path_marker.color.r = 0.0
        self.path_marker.color.g = 0.0
        self.path_marker.color.b = 1.0  # Blue color
        self.path_marker.color.a = 1.0  # Full opacity

    def intrinsic_callback(self, data):
        """
        Callback to receive and parse the intrinsic camera parameters.
        """
        # print("Intrinsic callback triggered")
        temp = data.data
        temp1 = np.fromstring(temp, sep=' ')
        temp2 = np.reshape(temp1, (3, 3))
        self.intrinsic = temp2
        self.fx = temp2[0][0]  # Focal length in x
        self.fy = temp2[1][1]  # Focal length in y
        self.cx = temp2[0][2]  # Principal point x-coordinate
        self.cy = temp2[1][2]  # Principal point y-coordinate

    def odom_callback(self, msgs):
        """
        Callback to update the robot's current position from odometry data.
        """
        # print("Odometry callback triggered")
        self.ox = msgs.pose.pose.position.x
        self.oy = msgs.pose.pose.position.y

    def left_img_callback(self, data):
        """
        Callback to receive and store the left camera image in grayscale.
        """
        print("Left image callback triggered")
        left_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        self.left.append(left_gray)

    def right_img_callback(self, data):
        """
        Callback to receive and process the right camera image.
        Also computes odometry and publishes results.
        """

        print("Processing stereo images...")
        right_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        self.right.append(right_gray)

        # Detect features in both images
        kpr, desr = self.detect_features(self.right[-1])
        kpl, desl = self.detect_features(self.left[-1])

        # Convert keypoints to points
        ptsr = cv2.KeyPoint_convert(kpr)
        ptsl = cv2.KeyPoint_convert(kpl)

        # Track features between the two images
        ptr, ptl = self.track_features(ptsr, ptsl, desr, desl)

        # Compute odometry (position and orientation)
        pos, ori = self.compute_odometry(ptl, ptr)

        # Adjust the position to account for odometry drift
        x, y = pos[0], pos[1]
        if abs(x - self.ox) > 5 or abs(y - self.oy) > 5:
            x, y = self.ox, self.oy
        else:
            x += self.ox
            y += self.oy

        z = pos[2]
        roll, pitch, yaw = ori

        # Publish odometry data
        self.publish_odometry(x, y, z, roll, pitch, yaw)

        # Update and publish marker data for visualization
        self.update_marker(x, y)
        self.update_path_marker(x, y)

    def detect_features(self, img):
        """
        Detect ORB features in the given image.
        """
        orb = cv2.ORB_create()
        kp, des = orb.detectAndCompute(img, None)
        return kp, des

    def track_features(self, pt1, pt2, des1, des2):
        """
        Match features between two sets of points using BFMatcher.
        """
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = sorted(bf.match(des1, des2), key=lambda x: x.distance)
        pts1 = np.float32([pt1[m.queryIdx] for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([pt2[m.trainIdx] for m in matches]).reshape(-1, 1, 2)
        return pts1, pts2

    def compute_odometry(self, pts1, pts2):
        """
        Compute the robot's position and orientation based on tracked features.
        """
        # Intrinsic matrices for the two cameras
        in1 = np.array([[self.fx, 0, self.cx, 0], [0, self.fy, self.cy, 0], [0, 0, 1, 0]])
        in2 = np.array([[self.fx, 0, self.cx, self.baseline], [0, self.fy, self.cy, 0], [0, 0, 1, 0]])

        # Triangulate 3D points from the stereo images
        t = cv2.triangulatePoints(in1, in2, np.transpose(pts1), np.transpose(pts2))
        wpoints = t[:3] / t[3]  # Convert to homogeneous coordinates

        # Solve for rotation and translation using PnP RANSAC
        _, R, t, _ = cv2.solvePnPRansac(wpoints.T, pts1, np.eye(3), np.zeros((4, 1)))
        Rot, _ = cv2.Rodrigues(R)
        # Compute the new transformation matrix
        T = np.vstack((np.hstack((Rot, t)), [0, 0, 0, 1]))
        new = np.matmul(self.prev, T)

        # Extract position and orientation
        position = new[:3, 3]
        orientation = cv2.Rodrigues(new[:3, :3])[0]
        return position, orientation

    def publish_odometry(self, x, y, z, roll, pitch, yaw):
        """
        Publish the odometry data to the relevant topic.
        """
        self.odom_msg.pose.pose.position = Point(x, y, z)
        q = quaternion_from_euler(roll, pitch, yaw)
        self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, \
        self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w = q
        self.odom_pub.publish(self.odom_msg)

    def update_marker(self, x, y):
        """
        Update the marker for visualization and publish it.
        """
        self.marker.points.append(Point(x, y, 0))
        self.mark_pub.publish(self.marker)

    def update_path_marker(self, x, y):
        """
        Update the path marker for visualization and publish it.
        """
        self.path_marker.points.append(Point(x, y, 0))
        self.path_pub.publish(self.path_marker)

    def run(self):
        """
        Main loop to keep the ROS node running.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    svo = StereoVisualOdometry()
    try:
        svo.run()
    except rospy.ROSInterruptException:
        pass
