#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Bool

# Because of transformations
import tf_conversions

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
"""

class TfCalibration():
	def __init__(self):
		rospy.init_node('calibrate_tf')
		self.rate = rospy.Rate(10) # 10 Hz
		self.tf_offset_pub = rospy.Publisher("tf_offset",
											 Transform,
											 queue_size=1)

		self.exit_pub = rospy.Publisher("exit", Bool, queue_size=1)

		self.source_frame = rospy.get_param("~source_frame")
		self.map_frame = rospy.get_param("~map_frame")
		self.save_file = rospy.get_param("~save_file")

		# Start dynamic reconfigure server
		self.srv = Server(CalibrationConfig, self.paramCallback)

		# Transform
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

	def run(self):
		while not rospy.is_shutdown():
			if self.done:
				self.exit_pub.publish(Bool(True))
				self.srv.update_configuration({"done": False})

				t = self.tf_buffer.lookup_transform(self.source_frame, self.map_frame, rospy.Time(0))

				# Write file to save
				save_tf(t.transform, self.save_file)
				break

			t = Transform()
			t.translation.x = self.x
			t.translation.y = self.y
			t.translation.z = self.z

			q = tf_conversions.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
			t.rotation.x = q[0]
			t.rotation.y = q[1]
			t.rotation.z = q[2]
			t.rotation.w = q[3]

			self.tf_offset_pub.publish(t)
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
