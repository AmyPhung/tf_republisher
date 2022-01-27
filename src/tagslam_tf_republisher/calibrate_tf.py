#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import Transform, Vector3
from std_msgs.msg import Bool

# Because of transformations
import tf_conversions
from geometry_msgs.msg import Transform, TransformStamped, Quaternion
import tf.transformations as tf_trans

from tf_republisher.cfg import CalibrationConfig
from dynamic_reconfigure.server import Server
from utils import save_tf



## TODO: update package.xml with dependencies
## TODO: use apriltags
## TODO: save config
## TODO: update values in .cfg ons ave

"""
Frames we care about

Base > map (kinect)
Fisheye_Model > map (fisheye)


Origin tag: 570

want to adjust tf between kinect and base, but in origin tag frame. This script
is needed because the kinect's position is currently defined in the base frame

From origin tag, make copy of tf, and publish tfs:
	- base > origin tag copy (static)
	- base > other tag copies (dynamic - lookup)
	- origin tag copy > kinect (dynamic - reconfigure)

"""

BASE = "Base"
TAGSLAM_BASE = "tagslam_base"
ORIGIN_TAG = "tag_570"
TAGSLAM_CAMERA = "tagslam_kinect"
TAGSLAM_ROOT = "map"
CAMERA = "kinect2_rgb_optical_frame"

BASE_TAGS = ["tag_571","tag_572","tag_573"]


class TfCalibration():
	def __init__(self):
		rospy.init_node('calibrate_tf')
		self.rate = rospy.Rate(20) # 10 Hz
		self.save_file = rospy.get_param("~save_file")

		# Start dynamic reconfigure server
		self.srv = Server(CalibrationConfig, self.paramCallback)

		# Transform offset for camera to origin tag
		self.x = 0
		self.y = 0
		self.z = 0
		self.roll  = 0
		self.pitch = 0
		self.yaw   = 0
		self.done = False

		# Tf listener
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()

	def run(self):
		while not rospy.is_shutdown():
			if self.done:
				self.srv.update_configuration({"done": False})

				# Save tf between base and kinect
				t = self.tf_buffer.lookup_transform(BASE, CAMERA, rospy.Time(0))

				# Write file to save
				save_tf(t.transform, self.save_file)
				break

			# Continously update tagslam estimate for quick tf updates
			for tag in BASE_TAGS:
				try:
					# Update tfs based on tagslam and republish with new header
					tag_tf = self.tf_buffer.lookup_transform(TAGSLAM_BASE,
															 tag,
												   			 rospy.Time(0))
					tag_tf.header.stamp = rospy.Time.now()
					tag_tf.header.frame_id = BASE
					tag_tf.child_frame_id = f"{tag}_copy"
					self.tf_broadcaster.sendTransform(tag_tf)
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
					rospy.logwarn_throttle(1, f"No tf between {TAGSLAM_BASE} and {tag}")
					pass

			try:
				# Update tfs based on tagslam and republish with new header
				origin_tf = self.tf_buffer.lookup_transform(TAGSLAM_BASE,
															ORIGIN_TAG,
											   				rospy.Time(0))
				origin_tf.header.stamp = rospy.Time.now()
				origin_tf.header.frame_id = BASE
				origin_tf.child_frame_id = f"{ORIGIN_TAG}_copy"
				self.tf_broadcaster.sendTransform(origin_tf)


			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logwarn_throttle(1, f"No tf between {TAGSLAM_BASE} and {ORIGIN_TAG}")
				pass

			try:
				camera_tf = self.tf_buffer.lookup_transform(TAGSLAM_CAMERA,
															ORIGIN_TAG,
															rospy.Time(0))

				# Update header info
				camera_tf.header.stamp = rospy.Time.now()
				camera_tf.header.frame_id = f"{ORIGIN_TAG}_copy"
				camera_tf.child_frame_id = CAMERA

				# Multiply quaternions
				q1 = [camera_tf.transform.rotation.x,
					  camera_tf.transform.rotation.y,
					  camera_tf.transform.rotation.z,
					  camera_tf.transform.rotation.w]

				q2 = tf_conversions.transformations.quaternion_from_euler(self.roll,
																		  self.pitch,
																		  self.yaw)

				rot = tf_trans.quaternion_multiply(q2, q1)

				camera_tf.transform.rotation = Quaternion(*tf_trans.quaternion_multiply(q1, q2))

				trans = [camera_tf.transform.translation.x,
					  	 camera_tf.transform.translation.y,
					  	 camera_tf.transform.translation.z]

				rot = [camera_tf.transform.rotation.x,
					   camera_tf.transform.rotation.y,
					   camera_tf.transform.rotation.z,
					   camera_tf.transform.rotation.w]

				# Invert tf
				transform = tf_trans.concatenate_matrices(tf_trans.translation_matrix(trans),
												 		  tf_trans.quaternion_matrix(rot))
				inversed_transform = tf_trans.inverse_matrix(transform)

				camera_tf.transform.translation = Vector3(*tf_trans.translation_from_matrix(inversed_transform))
				camera_tf.transform.rotation = Quaternion(*tf_trans.quaternion_from_matrix(inversed_transform))

				# Add tfs together
				camera_tf.transform.translation.x += self.x
				camera_tf.transform.translation.y += self.y
				camera_tf.transform.translation.z += self.z

				# Display tf
				self.tf_broadcaster.sendTransform(camera_tf)

				# # Update tagslam root
				# camera_tf.header.frame_id = f"{ORIGIN_TAG}_copy"
				# camera_tf.child_frame_id = TAGSLAM_ROOT
				# self.tf_broadcaster.sendTransform(camera_tf)


			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logwarn_throttle(1, f"No tf between {ORIGIN_TAG} and {TAGSLAM_CAMERA}")
				pass

			self.rate.sleep()

	def paramCallback(self, config, level):
		self.x = config.x
		self.y = config.y
		self.z = config.z
		self.roll  = config.roll
		self.pitch = config.pitch
		self.yaw   = config.yaw

		self.done = config.done
		return config

if __name__ == '__main__':
	tf_cal = TfCalibration()
	tf_cal.run()
