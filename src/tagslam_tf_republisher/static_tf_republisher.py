#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_multiply
from utils import load_tf

class StaticTfRepublisher():
    def __init__(self):
        rospy.init_node('tagslam_tf_republisher')

        self.rate = rospy.Rate(5)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.source_frame = rospy.get_param("~source_frame")
        self.target_frame = rospy.get_param("~target_frame")
        self.tf_file = rospy.get_param("~tf_file")

    def run(self):
        while not rospy.is_shutdown():
            output_tf = TransformStamped()

            output_tf.header.stamp = rospy.Time.now()
            output_tf.header.frame_id = self.source_frame
            output_tf.child_frame_id = self.target_frame

            output_tf.transform = load_tf(self.tf_file)

            # Ouptut message
            self.tf_broadcaster.sendTransform(output_tf)
            self.rate.sleep()


if __name__ == '__main__':
    tf_publisher = TagslamTfRepublisher()
    tf_publisher.run()
