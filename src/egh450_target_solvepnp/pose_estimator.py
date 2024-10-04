

#!/usr/bin/env python3

# import rospy
# import cv2
# import numpy as np
# import tf2_ros
# import tf_conversions
# from geometry_msgs.msg import TransformStamped
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Image, CompressedImage, CameraInfo
# from cv_bridge import CvBridge, CvBridgeError

# class PoseEstimator:
#     def __init__(self):
#         # Set up the CV Bridge
#         self.bridge = CvBridge()
#         self.sub_object_pose = rospy.Subscriber("/object_pose", Float32MultiArray, self.callback_obj_pose, buff_size=4)
#         self.pub_camera_pose = rospy.Publisher("/camera/pose", TransformStamped, queue_size=50)

#         # Initialize variables
#         self.model_image = None
#         self.object_data = {}
#         self.published_objects = set()
#         self.got_camera_info = False
#         self.camera_matrix = None
#         self.dist_coeffs = None

#         # Flag to ensure output is printed only once per object
#         self.output_printed = {}

#         # Set ROS parameters
#         self.param_use_compressed = rospy.get_param("~use_compressed", False)

#         # Set up subscribers and publishers
#         self.sub_info = rospy.Subscriber("~camera_info", CameraInfo, self.callback_info)
#         if self.param_use_compressed:
#             self.sub_img = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.callback_img)
#             self.pub_overlay = rospy.Publisher("~overlay/image_raw/compressed", CompressedImage, queue_size=1)
#         else:
#             self.sub_img = rospy.Subscriber("~image_raw", Image, self.callback_img)
#             self.pub_overlay = rospy.Publisher("~overlay/image_raw", Image, queue_size=1)

#         self.tfbr = tf2_ros.TransformBroadcaster()

#     def shutdown(self):
#         # Unregister anything that needs it here
#         self.sub_info.unregister()
#         self.sub_img.unregister()

#     # Callback for CameraInfo messages
#     def callback_info(self, msg_in):
#         self.dist_coeffs = np.array([[msg_in.D[0], msg_in.D[1], msg_in.D[2], msg_in.D[3], msg_in.D[4]]], dtype="double")
#         self.camera_matrix = np.array([
#             [msg_in.P[0], msg_in.P[1], msg_in.P[2]],
#             [msg_in.P[4], msg_in.P[5], msg_in.P[6]],
#             [msg_in.P[8], msg_in.P[9], msg_in.P[10]]],
#             dtype="double")
        
#         if not self.got_camera_info:
#             rospy.loginfo("Got camera info")
#             self.got_camera_info = True

#     # Callback for object pose messages
#     def callback_obj_pose(self, msg_in):
#         obj_array = msg_in.data
#         class_id = int(obj_array[0])  # Extract the class ID
#         corners = obj_array[1:9]
#         marker_length_x = obj_array[9]
#         marker_length_y = obj_array[10]

#         # Verify if we have a known target
#         if class_id == 101:
#             target = "Drone"
#         elif class_id == 102:
#             target = "Phone"
#         else:
#             rospy.logwarn("Unknown target identifier")
#             return

#         # Store raw corner data keyed by marker ID
#         self.object_data[class_id] = corners

#         # Initialize the flag for this object ID
#         if class_id not in self.output_printed:
#             self.output_printed[class_id] = False

#         # Create a model object based on detected marker size
#         self.model_object = np.array([
#             (-marker_length_x / 2, marker_length_y / 2, 0.0),  # Top-left
#             (marker_length_x / 2, marker_length_y / 2, 0.0),   # Top-right
#             (marker_length_x / 2, -marker_length_y / 2, 0.0),  # Bottom-right
#             (-marker_length_x / 2, -marker_length_y / 2, 0.0)  # Bottom-left
#         ])

#     # Callback for images
#     def callback_img(self, msg_in):
#         if not self.got_camera_info:
#             rospy.logwarn("Waiting for camera info.")
#             return

#         # Convert ROS image to OpenCV image
#         try:
#             if self.param_use_compressed:
#                 cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_in, "bgr8")
#             else:
#                 cv_image = self.bridge.imgmsg_to_cv2(msg_in, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#             return

#         # If we have object data, use it to estimate pose
#         for class_id, corners in self.object_data.items():
#             # Ensure we have exactly 4 points
#             if len(corners) != 8:
#                 rospy.logwarn("Incorrect number of corners")
#                 continue

#             self.model_image = np.array([
#                 (corners[0], corners[1]),  # Top-left
#                 (corners[2], corners[3]),  # Top-right
#                 (corners[4], corners[5]),  # Bottom-right
#                 (corners[6], corners[7])   # Bottom-left
#             ], dtype=np.float32)

#             # Check that we have at least 4 points
#             if self.model_object.shape[0] < 4 or self.model_image.shape[0] < 4:
#                 rospy.logwarn("Not enough points for solvePnP")
#                 continue

#             # Perform pose estimation using solvePnP
#             success, rvec, tvec = cv2.solvePnP(self.model_object, self.model_image, self.camera_matrix, self.dist_coeffs)

#             if success:
#                 # Check if the output for this object has already been printed
#                 if self.output_printed.get(class_id, False):
#                     continue  # Skip if already printed

#                 msg_out = TransformStamped()
#                 msg_out.header = msg_in.header
#                 msg_out.child_frame_id = f"{class_id}"
#                 msg_out.transform.translation.x = tvec[0]
#                 msg_out.transform.translation.y = tvec[1]
#                 msg_out.transform.translation.z = tvec[2]
                
#                 # Convert rotation vector to quaternion
#                 q = tf_conversions.transformations.quaternion_from_euler(rvec[0], rvec[1], rvec[2])
#                 msg_out.transform.rotation.x = q[0]
#                 msg_out.transform.rotation.y = q[1]
#                 msg_out.transform.rotation.z = q[2]
#                 msg_out.transform.rotation.w = q[3]
                
#                 rospy.loginfo("Translation x: %f",  msg_out.transform.translation.x)
#                 rospy.loginfo("Translation y: %f",  msg_out.transform.translation.y)
#                 rospy.loginfo("Translation z: %f",  msg_out.transform.translation.z)
                
#                 # Broadcast the pose and publish the message
#                 self.tfbr.sendTransform(msg_out)
#                 self.pub_camera_pose.publish(msg_out)

#                 # Add the object to published set
#                 self.published_objects.add(class_id)

#                 # Set the flag to True after printing
#                 self.output_printed[class_id] = True

#                 # Visualize the corners
#                 for point in self.model_image:
#                     cv2.circle(cv_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), 3)

#         # Publish overlay image
#         try:
#             if self.param_use_compressed:
#                 self.pub_overlay.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image, "png"))
#             else:
#                 self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#         except (CvBridgeError, TypeError) as e:
#             rospy.logerr(e)

# if __name__ == "__main__":
#     rospy.init_node("pose_estimator")
#     pe = PoseEstimator()
#     rospy.spin()
#!/usr/bin/env python3

import rospy 
import cv2
import numpy as np
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class PoseEstimator:
    def __init__(self):
        # Initialise the CV Bridge for converting ROS IMage messages to Open CV
        # Set up the CV Bridge
        self.bridge = CvBridge()

        # Subscriber for ArUco marker detection. It listens to /aruco_detection topic, 
        # which provides marker ID and the 2D corner points of the marker.
        self.sub_object_pose = rospy.Subscriber("/object_pose", Float32MultiArray, self.callback_obj_pose, queue_size=50)

        self.sub_aruco_pose = rospy.Subscriber("/aruco_detection", Float32MultiArray, self.callback_aruco_pose, queue_size=50)
        # Publisher for the estimated camera pose based on object/marker detections. 
        # It publishes the camera's position and orientation (as a transformation) relative to the detected object.
        self.pub_camera_pose = rospy.Publisher("/camera/pose", TransformStamped, queue_size=50)

        # Initialize variables to store image data, object detection data, and camera parameters
        self.model_image = None # Variable for storing 2D image points (corners of detected objects)
        self.object_data = {}   # Dictionary to store detection data such as corners, marker lengths
        self.published_objects = set() # Set to store IDs of objects/markers whose pose has been published
        self.got_camera_info = False # Flag to ensure camera info (intrinsics) has been received before using them
        self.camera_matrix = None  # Intrinsic camera matrix (from CameraInfo message)
        self.dist_coeffs = None  # Distortion coefficients (from CameraInfo message)

        # Dictionary to track which objects have already had their output printed, 
        # preventing redundant printing and pose broadcasting
        self.output_printed = {}

        # ROS parameter to check if the node should use compressed images (JPEG/PNG) or raw images (uncompressed)
        self.param_use_compressed = rospy.get_param("~use_compressed", False)

         # Subscribe to the camera info topic to receive the intrinsic parameters and distortion coefficients of the camera
        self.sub_info = rospy.Subscriber("~camera_info", CameraInfo, self.callback_info)
         # Subscribe to the image topic. Depending on whether the images are compressed or not, subscribe accordingly.
        # If compressed, subscribe to the compressed image topic and publish compressed images with overlays.
        if self.param_use_compressed:
            self.sub_img = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.callback_img)
            self.pub_overlay = rospy.Publisher("~overlay/image_raw/compressed", CompressedImage, queue_size=1)
        else:
             # If uncompressed (raw), subscribe to the uncompressed image topic and publish uncompressed images with overlays.
            self.sub_img = rospy.Subscriber("~image_raw", Image, self.callback_img)
            self.pub_overlay = rospy.Publisher("~overlay/image_raw", Image, queue_size=1)

        # Create a TransformBroadcaster object to broadcast the estimated camera pose (TransformStamped message)
        self.tfbr = tf2_ros.TransformBroadcaster()

    def shutdown(self):
        # Method to unregister subscribers when the node shuts down. This ensures no further messages are processed.
        self.sub_info.unregister()
        self.sub_img.unregister()

  # Callback function for processing camera intrinsic parameters (CameraInfo message)
    def callback_info(self, msg_in):
          # Extract the distortion coefficients and intrinsic camera matrix from the CameraInfo message.
        self.dist_coeffs = np.array(msg_in.D, dtype=np.float32)
        self.camera_matrix = np.array([
            [msg_in.K[0], msg_in.K[1], msg_in.K[2]], # First row of the camera matrix
            [msg_in.K[3], msg_in.K[4], msg_in.K[5]],  # Second row of the camera matrix
            [msg_in.K[6], msg_in.K[7], msg_in.K[8]]],   # Third row of the camera matrix
            dtype=np.float32)
         # Log information only once after receiving the camera info to prevent redundant logging
        if not self.got_camera_info:
            rospy.loginfo("Got camera info")
            self.got_camera_info = True

    # Callback function for handling object pose messages from YOLOv5 (Float32MultiArray format)
    def callback_obj_pose(self, msg_in):
        obj_array = msg_in.data
        if len(obj_array) < 11:
            rospy.logwarn("Object POSE not transmitted correctly")
            return

        # Extract the class ID of the detected object (e.g., 101 for Drone, 102 for Phone)
        class_id = int(obj_array[0])  # Extract the class ID
        corners = obj_array[1:9]  # 8 values representing the 4 corner points (x1, y1, x2, y2, x3, y3, x4, y4)
        marker_length_x = obj_array[9]  # Marker/object length in X direction
        marker_length_y = obj_array[10]   # Marker/object length in Y direction

        # Verify if we have a known target
        if class_id == 101:
            target = "Drone"
        elif class_id == 102:
            target = "Phone"
        else:
            rospy.logwarn("Unknown target identification")
            return

        # Store detection data (corners, marker lengths) in the object_data dictionary
        self.object_data[class_id] = (corners, marker_length_x, marker_length_y)

        # Initialize the output_printed flag for this object class to avoid redundant logging
        if class_id not in self.output_printed:
            self.output_printed[class_id] = False

    # Callback function for handling ArUco marker detection (Float32MultiArray format)
    def callback_aruco_pose(self, msg_in):
         # Extract the detection data array, which includes the marker ID and corner points
        obj_array = msg_in.data
        if len(obj_array) != 9:
            rospy.logwarn("ArUco POSE not transmitted correctly")
            return
         # Extract the marker ID (first element) and corner points (next 8 elements)
        marker_id = int(obj_array[0])  # Extract the marker ID
        corners = obj_array[1:9]  #  8 values representing the 4 corner points (x1, y1, x2, y2, x3, y3, x4, y4)

        # Known marker size (adjust to your actual marker size in meters)
        marker_length = 0.2  # For example, 5 cm markers

        # Store the ArUco marker detection data (corners, marker size) in the object_data dictionary
        self.object_data[marker_id] = (corners, marker_length, marker_length)

        # Initialize the output_printed flag for this marker to avoid redundant logging
        if marker_id not in self.output_printed:
            self.output_printed[marker_id] = False

     # Callback function for processing image messages
    def callback_img(self, msg_in):
        # Ensure that the camera info (intrinsics) has been received before processing the image
        if not self.got_camera_info:
            rospy.logwarn("Waiting for camera info.")
            return

       # Convert the incoming ROS image message to an OpenCV image (compressed or raw based on the parameter)
        try:
            if self.param_use_compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_in, "bgr8")  # Convert compressed image
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg_in, "bgr8") # Convert raw image
        except CvBridgeError as e:
            rospy.logerr(e)
            return

         # Iterate through all detected objects or markers
        for class_id, data in self.object_data.items():
            # Data contains corners and marker lengths for each detected object/marker
            corners = data[0]
            marker_length_x = data[1]
            marker_length_y = data[2]

             # Ensure that there are exactly 8 corner points
            if len(corners) != 8:
                rospy.logwarn("Incorrect number of corners")
                continue
            # Create a 4-point 2D model of the detected object/marker corners in image space
            self.model_image = np.array([
                (corners[0], corners[1]),  # Top-left
                (corners[2], corners[3]),  # Top-right
                (corners[4], corners[5]),  # Bottom-right
                (corners[6], corners[7])   # Bottom-left
            ], dtype=np.float32)

              # Ensure that we have enough points (at least 4) for pose estimation
            if self.model_image.shape[0] < 4:
                rospy.logwarn("Not enough points for solvePnP")
                continue

           
            # Define the 3D model points of the object's corners in real-world coordinates
            self.model_object = np.array([
                (-marker_length_x / 2, marker_length_y / 2, 0.0),  # Top-left
                (marker_length_x / 2, marker_length_y / 2, 0.0),   # Top-right
                (marker_length_x / 2, -marker_length_y / 2, 0.0),  # Bottom-right
                (-marker_length_x / 2, -marker_length_y / 2, 0.0)  # Bottom-left
            ], dtype=np.float32)

            # Perform pose estimation using OpenCV's solvePnP function to estimate the camera's pose
            success, rvec, tvec = cv2.solvePnP(self.model_object, self.model_image, self.camera_matrix, self.dist_coeffs)

            if success:
                 # Avoid re-processing or re-publishing the pose for the same object/marker
                if self.output_printed.get(class_id, False):
                    continue  # Skip if already printed
                  # Create a TransformStamped message to store the estimated pose (translation and rotation)
                msg_out = TransformStamped()
                msg_out.header = msg_in.header   # Use the same header as the image message
                msg_out.child_frame_id = f"{class_id}"  # Set the frame ID to the object/marker ID
                msg_out.transform.translation.x = tvec[0][0] # Translation in X
                msg_out.transform.translation.y = tvec[1][0] # Translation in Y
                msg_out.transform.translation.z = tvec[2][0] # Translation in Z
                
                
                # Convert the rotation vector (rvec) to a quaternion (for easier interpretation in 3D space)
                q = tf_conversions.transformations.quaternion_from_euler(rvec[0][0], rvec[1][0], rvec[2][0])
                msg_out.transform.rotation.x = q[0]
                msg_out.transform.rotation.y = q[1]
                msg_out.transform.rotation.z = q[2]
                msg_out.transform.rotation.w = q[3]
                 # Log the translation values (for debugging or visualization purposes)
                rospy.loginfo("Translation x: %f",  msg_out.transform.translation.x)
                rospy.loginfo("Translation y: %f",  msg_out.transform.translation.y)
                rospy.loginfo("Translation z: %f",  msg_out.transform.translation.z)
                
                # Broadcast the estimated pose (for integration with other nodes that need the transform)
                self.tfbr.sendTransform(msg_out)
                     # Publish the camera pose (translation and rotation)
                self.pub_camera_pose.publish(msg_out)

                # Mark the object as processed (to avoid re-processing)
                self.published_objects.add(class_id)

                # Set the flag to True after printing
                self.output_printed[class_id] = True

                 # Visualize the corners of the detected object/marker by drawing circles on the image
                for point in self.model_image:
                    cv2.circle(cv_image, (int(point[0]), int(point[1])), 5, (0, 255, 0), 3)

        # Publish overlay image
        try:
            if self.param_use_compressed:
                self.pub_overlay.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image, "png")) # Publish compressed image
            else:
                self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except (CvBridgeError, TypeError) as e:
            rospy.logerr(e)  # Log any errors during image conversion or publishing

if __name__ == "__main__":
     # Initialize the ROS node with the name 'egh450_target_solvepnp'
    rospy.init_node("egh450_target_solvepnp")
     # Create an instance of the PoseEstimator class
    pe = PoseEstimator()
      # Keep the node running and processing callbacks
    rospy.spin()
