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
		self.sub_aruco_pose = rospy.Subscriber("/aruco_pose", Float32MultiArray, self.callback_aruco_pose)
		
		self.pub_camera_pose = rospy.Publisher("/camera/pose", TransformStamped, queue_size=2)

		# Define self.model_image
		self.model_image = None

		# Define sets of Marker Data
		self.marker_data = {}
		self.published_markers = set()

		# Define sets of Object data
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

		# Generate the model for the pose solver
		# There are 5 points, one in the center, and one in each corner
		marker_side_length = 0.2
		self.model_object = np.array([
			(0.0, 0.0, 0.0), 	# Centre point
    		(-marker_side_length / 2, marker_side_length / 2, 0.0),  # Top-left corner
    		(marker_side_length / 2, marker_side_length / 2, 0.0),  # Top-right corner
    		(marker_side_length / 2, -marker_side_length / 2, 0.0),  # Bottom-right corner
    		(-marker_side_length / 2, -marker_side_length / 2, 0.0)  # Bottom-left corner
			])
		
		# Collect in the camera characteristics - hard coded to minimise communication errors
		self.dist_coeffs = np.array([[-0.10818, 0.12793, 0.00000, 0.00000, -0.04204]], dtype=np.float32)

		self.camera_matrix = np.array([
                 (615.381, 0.0, 320.0),
                 (0.0, 615.381, 240.0),
                 (0.0, 0.0, 1.0)],
				 dtype=np.float32)

		if not self.got_camera_info:
			rospy.loginfo("Got camera info")
			self.got_camera_info = True

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_info.unregister()
		self.sub_img.unregister()

	def callback_aruco_pose(self, msg_in):
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

		if len(msg_in.data) < 9:	# Ensure data has at least ID and 4 corner points
			rospy.logwarn(" Received malformed ArUco data.")
			return
		
		# Parse incoming coordinates
		id_array = msg_in.data
		num_markers = len(id_array) // 9

		for i in range(num_markers):		
			marker_ID = int(id_array[i * 9])
			corners = np.array(id_array[i * 9 + 1: i * 9 + 9]).reshape((4, 2))

			rospy.loginfo(f"Received ArUco Marker {marker_ID} with corners: {corners}")

			#store raw corner data keyed by marker ID
			self.marker_data[marker_ID] = corners
		
		# Set corners into 2D box			
		for marker_ID, corners in self.marker_data.items():
			if marker_ID not in self.published_markers:
				if corners is not None and len(corners) == 4:
					self.model_image = np.array([
						((corners[0][0] + corners[1][0]) / 2, (corners[0][1] + corners[1][1]) / 2),  # Center point
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
					msg_out.child_frame_id = f"{marker_ID}"
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
					self.published_markers.add(marker_ID)

					rospy.loginfo("Translation Coordinates for ROI are: [x: %0.2f; y: %0.2f; z: %0.2f]" 
			   	   % (msg_out.transform.translation.x, msg_out.transform.translation.y, msg_out.transform.translation.z))
						
			# Draw the ArUco marker corners for visualisation
			for point in self.model_image[0:]:
				cv2.circle(cv_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), 3)

	def callback_obj_pose(self, msg_in):
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

		# Get data from the detection model	
		if len(msg_in.data) < 8:	# Ensure data has at least label, confidence and 4 corner coordinates
			rospy.logwarn(" Received malformed object data data.")
			return
		
		# Parse the incoming coordinates
		detection_array = msg_in.data
		num_detections = len(detection_array) // 8

		for i in range(num_detections):
        # Each detection block contains: [class_id, confidence, xmin, ymin, xmax, ymax, lengthx, lengthy]
			class_id = int(detection_array[i * 6])  # Extract the class ID
			confidence = detection_array[i * 6 + 1]  # Extract the confidence score
			xmin = detection_array[i * 6 + 2]
			ymin = detection_array[i * 6 + 3]
			xmax = detection_array[i * 6 + 4]
			ymax = detection_array[i * 6 + 5]
			marker_length_x = detection_array[i * 6 + 6]
			marker_length_y = detection_array[i* 6 + 7]

        # Convert bounding box data into a numpy array or use it as needed
        # Example: Represent bounding box as four corner points
			self.model_image = np.array([
       	     	[xmin, ymin],  # Top-left corner
        	    [xmax, ymin],  # Top-right corner
        	    [xmax, ymax],  # Bottom-right corner
        	    [xmin, ymax]   # Bottom-left corner
        	])

			rospy.loginfo(f"Received detection for class {class_id} with confidence {confidence} and bbox: {bbox_corners}")










		# #Convert CV image to ROS image and publish the mask / overlay
		# try:
		# 	if self.param_use_compressed:
		# 		self.pub_overlay.publish( self.bridge.cv2_to_compressed_imgmsg( cv_image, "png" ) )
		# 	else:
		# 		self.pub_overlay.publish( self.bridge.cv2_to_imgmsg( cv_image, "bgr8" ) )
		# except (CvBridgeError,TypeError) as e:
		# 	rospy.logerr(e)
