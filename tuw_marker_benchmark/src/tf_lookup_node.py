#!/usr/bin/env python
import rospy
import os
from tf2_py import ExtrapolationException
from tf2_ros import Buffer, TransformListener, TFMessage, TransformStamped


class TfLookupNode:

    def __init__(self):
        self.tfb = Buffer()
        self.tfSub = None
        self.tf_records = []

        # Read parameters
        self.tf_target_frame = rospy.get_param("~tf_target_frame", None)
        self.tf_source_frame = rospy.get_param("~tf_source_frame", None)
        if self.tf_target_frame is None or self.tf_source_frame is None:
            raise ValueError("Parameter tf_target_frame or/and tf_source_frame not set")

    def run(self):
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)

    def tf_callback(self, msg):
        try:
            timeout = 1.0
            tfs = self.tfb.lookup_transform(self.tf_target_frame, self.tf_source_frame, rospy.Time(0), rospy.Duration(timeout))
            print(tfs)
            self.tf_records.append(tfs.transform)

            if(len(self.tf_records) > 50):
                self.calculate_transform_avg(self.tf_records)

        except (rospy.ROSException, ExtrapolationException) as e:
            rospy.logerr("TF lookup not ready yet...")
            return

    def calculate_transform_avg(self, tf_records):
        # TODO: Implement? For now, just print the latest
        tf = tf_records[len(tf_records)-1]
        rospy.loginfo("Translation: %s Rotation: %s" %(tf.translation, tf.rotation))
        os._exit(0)

if __name__ == '__main__':
    rospy.init_node('tfLookupNode', anonymous=True)
    try:
        node = TfLookupNode()
        node.run()
        rospy.spin()
    except StandardError as e:
        rospy.logerr("Failed to start %s: %s" % (rospy.get_name(), e.message))
        rospy.signal_shutdown("Failed to start %s: %s" % (rospy.get_name(), e.message))
        exit(1)

