#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class PoseEstimator():
	def __init__(self):
		# Set up the CV Bridge
		self.bridge = CvBridge()
		self.sub_object_pose = rospy.Subscriber("/object_pose", Float32MultiArray, self.callback_obj_pose)
		self.pub_camera_pose = rospy.Publisher("/camera/pose", TransformStamped, queue_size=2)

		# Define self.model_image
		self.model_image = None

		# Define sets of Marker Data
		self.object_data = {}
		self.published_objects = set()

		# Load in parameters from ROS
		self.param_use_compressed = rospy.get_param("~use_compressed", False)

		# Set additional camera parameters
		self.got_camera_info = False
		self.camera_matrix = None
		self.dist_coeffs = None

		# Set up the publishers, subscribers, and tf2
		self.sub_info = rospy.Subscriber("~camera_info", CameraInfo, self.callback_info)

		if self.param_use_compressed:
			self.sub_img = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.callback_img)
			self.pub_overlay = rospy.Publisher("~overlay/image_raw/compressed", CompressedImage, queue_size=1)
		else:
			self.sub_img = rospy.Subscriber("~image_raw", Image, self.callback_img)
			self.pub_overlay = rospy.Publisher("~overlay/image_raw", Image, queue_size=1)

		self.tfbr = tf2_ros.TransformBroadcaster()

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_info.unregister()
		self.sub_img.unregister()

	# Collect in the camera characteristics
	def callback_info(self, msg_in):
		self.dist_coeffs = np.array([[msg_in.D[0], msg_in.D[1], msg_in.D[2], msg_in.D[3], msg_in.D[4]]], dtype=np.float32)

		self.camera_matrix = np.array([
                 (msg_in.P[0], msg_in.P[1], msg_in.P[2]),
                 (msg_in.P[4], msg_in.P[5], msg_in.P[6]),
                 (msg_in.P[8], msg_in.P[9], msg_in.P[10])],
				 dtype=np.float32)

		if not self.got_camera_info:
			rospy.loginfo("Got camera info")
			self.got_camera_info = True

	def callback_obj_pose(self, msg_in):
		if len(msg_in.data) < 10:	# Ensure data has at least ID, 4 sets of coordinates and x and y lengths
			rospy.logwarn(" Received malformed target data.")
			return
		
		# Parse incoming coordinates
		obj_array = msg_in.data
		num_detect = len(obj_array) // 10

		for i in range(num_detect):
        	# Each detection block contains: [class_id, 4 sets of xy coordinates, marker length x, marker length y]
			class_id = int(obj_array[i * 10])  # Extract the class ID
			corners = np.array(obj_array[i * 10 + 1: i * 10 + 9]).reshape((4,2))
			marker_length_x = obj_array[i * 10 + 9]
			marker_length_y = obj_array[i * 10 + 10]

			if class_id == 101:
				target = "backpack"
			elif class_id == 102:
				target = "person"
			else:
				rospy.logwarn("The target identifier is unknown")
				return
			
			rospy.loginfo(f"Received Target: {target} with coordinates")

			#store raw corner data keyed by marker ID
			self.object_data[class_id] = corners

		# Generate the model for the pose solver
		# There are 5 points, one in the center, and one in each corner
		self.model_object = np.array([
			(0.0, 0.0, 0.0), 	# Centre point
    		(-marker_length_x / 2, marker_length_y / 2, 0.0),  # Top-left corner
    		(marker_length_x / 2, marker_length_y / 2, 0.0),  # Top-right corner
    		(marker_length_x / 2, -marker_length_y / 2, 0.0),  # Bottom-right corner
    		(-marker_length_x / 2, -marker_length_y / 2, 0.0)  # Bottom-left corner
			])

	def callback_img(self, msg_in):
		# Don't bother to process image if we don't have the camera calibration
		if self.got_camera_info:
			#Convert ROS image to CV image
			cv_image = None

			try:
				if self.param_use_compressed:
					cv_image = self.bridge.compressed_imgmsg_to_cv2( msg_in, "bgr8" )
				else:
					cv_image = self.bridge.imgmsg_to_cv2( msg_in, "bgr8" )
			except CvBridgeError as e:
				rospy.logerr(e)
				return
			
			for class_id, corners in self.object_data.items():
				if class_id not in self.published_objects:
					if corners is not None and len(corners) == 4:
						self.model_image = np.array([
        					(corners[1][0], corners[1][1]),  # Top-left
        					(corners[2][0], corners[2][1]),  # Top-right
        					(corners[3][0], corners[3][1]),  # Bottom-right
        					(corners[0][0], corners[0][1])   # Bottom-left
						])

					# Do the SolvePnP method
					(success, rvec, tvec) = cv2.solvePnP(self.model_object, self.model_image, self.camera_matrix, self.dist_coeffs)

					# If a result was found, send to TF2
					if success:
						msg_out = TransformStamped()
						msg_out.header = msg_in.header
						msg_out.child_frame_id = f"{class_id}"
						msg_out.transform.translation.x = tvec[0] * 10e-2
						msg_out.transform.translation.y = tvec[1] * 10e-2
						msg_out.transform.translation.z = tvec[2] * 10e-2
						q = tf_conversions.transformations.quaternion_from_euler(rvec[0], rvec[1], rvec[2])
						msg_out.transform.rotation.w = q[3]	# Could use rvec, but need to convert from DCM to quaternion first
						msg_out.transform.rotation.x = q[0]
						msg_out.transform.rotation.y = q[1]
						msg_out.transform.rotation.z = q[2]

						#self.tfbr.sendTransform(msg_out)
						self.pub_camera_pose.publish(msg_out)
						#Add the marker ID to the published set
						self.published_markers.add(class_id)

						rospy.loginfo("Translation Coordinates for ROI are: [x: %0.2f; y: %0.2f; z: %0.2f]" 
				   	   % (msg_out.transform.translation.x, msg_out.transform.translation.y, msg_out.transform.translation.z))
						

				# Draw the corners for visualisation
				for point in self.model_image[0:]:
					cv2.circle(cv_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), 3)

			#Convert CV image to ROS image and publish the mask / overlay
			try:
				if self.param_use_compressed:
					self.pub_overlay.publish( self.bridge.cv2_to_compressed_imgmsg( cv_image, "png" ) )
				else:
					self.pub_overlay.publish( self.bridge.cv2_to_imgmsg( cv_image, "bgr8" ) )
			except (CvBridgeError,TypeError) as e:
				rospy.logerr(e)
