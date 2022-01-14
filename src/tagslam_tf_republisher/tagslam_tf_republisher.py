#!/usr/bin/env python3
"""
When using tagslam to update the transform between the kinect and the arm base,
tagslam will publish the tf-tree "upside-down." We want everything transformed
with respect to the arm base, while tagslam transforms everything with respect
to the camera.

To fix this, this node takes the transform between the kinect and the arm base
as reported by tagslam, then publish a tf between the arm base to the tagslam
root frame
"""

import rospy
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, Quaternion
from std_msgs.msg import Bool
from tf.transformations import quaternion_multiply

class TagslamTfRepublisher():
    def __init__(self):
        rospy.init_node('tagslam_tf_republisher')

        self.rate = rospy.Rate(5)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.base_frame = rospy.get_param("base_frame", "Base")
        self.tagslam_base_frame = rospy.get_param("~tagslam_base_frame")#, "tagslam_kinect")#"tagslam_base")
        self.tagslam_camera_frame = rospy.get_param("~tagslam_map_frame")#, "map")
        self.tf_msg = Transform()
        self.tf_msg.rotation.w = 1.0

        self.offset_sub = rospy.Subscriber("tf_offset", Transform, self.tf_callback)
        self.offset_msg = Transform()
        self.offset_msg.rotation.w = 1.0

        self.exit_sub = rospy.Subscriber("exit", Bool, self.exit_callback)
        self.done = False

    def tf_callback(self, msg):
        self.offset_msg = msg

    def exit_callback(self, msg):
        self.done = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if self.done:
                break
            try:
                t = self.tf_buffer.lookup_transform(self.tagslam_base_frame,
                                                    self.tagslam_camera_frame,
                                                    rospy.Time())
                self.tf_msg = t.transform

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.logwarn_throttle(3, "Error in tagslam tf republisher - is tagslam running with the base tags visible?")

            output_tf = TransformStamped()

            # Update header info
            output_tf.header.stamp = rospy.Time.now()
            output_tf.header.frame_id = self.base_frame
            output_tf.child_frame_id = self.tagslam_camera_frame

            # Add tfs together
            output_tf.transform.translation.x = \
                self.tf_msg.translation.x + self.offset_msg.translation.x
            output_tf.transform.translation.y = \
                self.tf_msg.translation.y + self.offset_msg.translation.y
            output_tf.transform.translation.z = \
                self.tf_msg.translation.z + self.offset_msg.translation.z

            # Multiply quaternions
            q1 = [self.tf_msg.rotation.x,
                  self.tf_msg.rotation.y,
                  self.tf_msg.rotation.z,
                  self.tf_msg.rotation.w]
            q2 = [self.offset_msg.rotation.x,
                  self.offset_msg.rotation.y,
                  self.offset_msg.rotation.z,
                  self.offset_msg.rotation.w]

            output_tf.transform.rotation = Quaternion(*quaternion_multiply(q2, q1))

            # Ouptut message
            self.tf_broadcaster.sendTransform(output_tf)

            self.rate.sleep()


if __name__ == '__main__':
    tf_publisher = TagslamTfRepublisher()
    tf_publisher.run()
