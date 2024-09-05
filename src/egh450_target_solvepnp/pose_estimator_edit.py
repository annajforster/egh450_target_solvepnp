#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf_conversions

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class PoseEstimator():
    def __init__(self):
        # Set up the CV Bridge
        self.bridge = CvBridge()

        # Load in parameters from ROS
        self.param_use_compressed = rospy.get_param("~use_compressed", False)
        self.param_circle_radius = rospy.get_param("~circle_radius", 1.0)
        self.param_hue_center = rospy.get_param("~hue_center", 170)
        self.param_hue_range = rospy.get_param("~hue_range", 20) / 2
        self.param_sat_min = rospy.get_param("~sat_min", 50)
        self.param_sat_max = rospy.get_param("~sat_max", 255)
        self.param_val_min = rospy.get_param("~val_min", 50)
        self.param_val_max = rospy.get_param("~val_max", 255)

        # Set additional camera parameters
        self.got_camera_info = False
        self.camera_matrix = None
        self.dist_coeffs = None

        # Set up the publishers, subscribers, and tf2
        self.sub_info = rospy.Subscriber("~camera_info", CameraInfo, self.callback_info)

        if self.param_use_compressed:
            self.sub_img = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.callback_img)
            self.pub_mask = rospy.Publisher("~debug/image_raw/compressed", CompressedImage, queue_size=2)
            self.pub_overlay = rospy.Publisher("~overlay/image_raw/compressed", CompressedImage, queue_size=2)
        else:
            self.sub_img = rospy.Subscriber("~image_raw", Image, self.callback_img)
            self.pub_mask = rospy.Publisher("~debug/image_raw", Image, queue_size=2)
            self.pub_overlay = rospy.Publisher("~overlay/image_raw", Image, queue_size=2)

        self.tfbr = tf2_ros.TransformBroadcaster()

        # Subscribe to the detected ArUco marker pose
        self.sub_aruco_pose = rospy.Subscriber("/processed_aruco/pose", PoseStamped, self.callback_aruco_pose)

        # Generate the model for the pose solver
        r = self.param_circle_radius
        self.model_object = np.array([(0.0, 0.0, 0.0),
                                      (r, r, 0.0),
                                      (r, -r, 0.0),
                                      (-r, r, 0.0),
                                      (-r, -r, 0.0)])

    def shutdown(self):
        # Unregister anything that needs it here
        self.sub_info.unregister()
        self.sub_img.unregister()
        self.sub_aruco_pose.unregister()

    # Collect camera characteristics
    def callback_info(self, msg_in):
        self.dist_coeffs = np.array([[msg_in.D[0], msg_in.D[1], msg_in.D[2], msg_in.D[3], msg_in.D[4]]], dtype="double")

        self.camera_matrix = np.array([
            (msg_in.P[0], msg_in.P[1], msg_in.P[2]),
            (msg_in.P[4], msg_in.P[5], msg_in.P[6]),
            (msg_in.P[8], msg_in.P[9], msg_in.P[10])],
            dtype="double")

        if not self.got_camera_info:
            rospy.loginfo("Got camera info")
            self.got_camera_info = True

    def callback_img(self, msg_in):
        if not self.got_camera_info:
            return

        try:
            if self.param_use_compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_in, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg_in, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)
            return

        # Perform image processing (if necessary)
        mask_image = self.process_image(cv_image)

        # If you want to display or use processed images, continue processing
        self.publish_images(mask_image, cv_image)

    def process_image(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image = None

        hue_lower = (self.param_hue_center - self.param_hue_range) % 180
        hue_upper = (self.param_hue_center + self.param_hue_range) % 180

        thresh_lower = np.array([hue_lower, self.param_val_min, self.param_val_min])
        thresh_upper = np.array([hue_upper, self.param_val_max, self.param_val_max])

        if hue_lower > hue_upper:
            thresh_lower_wrap = np.array([180, self.param_sat_max, self.param_val_max])
            thresh_upper_wrap = np.array([0, self.param_sat_min, self.param_val_min])

            mask_lower = cv2.inRange(hsv_image, thresh_lower, thresh_lower_wrap)
            mask_upper = cv2.inRange(hsv_image, thresh_upper_wrap, thresh_upper)

            mask_image = cv2.bitwise_or(mask_lower, mask_upper)
        else:
            mask_image = cv2.inRange(hsv_image, thresh_lower, thresh_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, kernel)

        return mask_image

    def publish_images(self, mask_image, cv_image):
        try:
            if self.param_use_compressed:
                self.pub_mask.publish(self.bridge.cv2_to_compressed_imgmsg(mask_image, "png"))
                self.pub_overlay.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image, "png"))
            else:
                self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask_image, "mono8"))
                self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except (CvBridgeError, TypeError) as e:
            rospy.loginfo(e)

    def callback_aruco_pose(self, msg):
        # Use ArUco marker pose for further processing or estimation
        rospy.loginfo(f"Received ArUco Marker Pose: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")

        # Here, implement any pose estimation logic you want using the ArUco marker pose.
        # For example, using the solvePnP function to estimate the precise pose.
        
        # Example using solvePnP:
        object_points = self.model_object
        image_points = np.array([
            [msg.pose.position.x, msg.pose.position.y],
            [msg.pose.position.x + 0.1, msg.pose.position.y + 0.1],  # Example points
            [msg.pose.position.x + 0.1, msg.pose.position.y - 0.1],
            [msg.pose.position.x - 0.1, msg.pose.position.y + 0.1],
            [msg.pose.position.x - 0.1, msg.pose.position.y - 0.1]
        ], dtype=np.float32)

        # Perform pose estimation
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
        
        if success:
            # Publish the transform to TF
            self.publish_transform(msg, rvec, tvec)

    def publish_transform(self, msg, rvec, tvec):
        # Create and publish TransformStamped for TF
        msg_out = TransformStamped()
        msg_out.header = msg.header
        msg_out.child_frame_id = "aruco_marker"
        msg_out.transform.translation.x = tvec[0]
        msg_out.transform.translation.y = tvec[1]
        msg_out.transform.translation.z = tvec[2]
        q = tf_conversions.transformations.quaternion_from_euler(rvec[0], rvec[1], rvec[2])
        msg_out.transform.rotation.x = q[0]
        msg_out.transform.rotation.y = q[1]
        msg_out.transform.rotation.z = q[2]
        msg_out.transform.rotation.w = q[3]

        self.tfbr.sendTransform(msg_out)

if __name__ == '__main__':
    rospy.init_node('pose_estimator')
    estimator = PoseEstimator()
    rospy.spin()