#!/usr/bin/env python
import rospy
from tf2_ros import Buffer, TransformListener, TFMessage, TransformStamped, TransformBroadcaster


class TfInvNode:

    def __init__(self):
        self.tfb = Buffer()
        self.tfSub = None

    def run(self):
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        self.tfPub = TransformBroadcaster()

    def tf_callback(self, msg):
        try:
            timeout = 1.0
            tfs = self.tfb.lookup_transform("checkerboard", "camera", rospy.Time(0), rospy.Duration(timeout))

            t = TransformStamped()
            t.header.stamp = tfs.header.stamp
            t.header.frame_id = "checkerboard_inv"
            t.child_frame_id = "camera_inv"
            t.transform.translation = tfs.transform.translation
            t.transform.rotation = tfs.transform.rotation
            self.tfPub.sendTransform(t)
        except rospy.ROSException as e:
            rospy.logerr(e)
            return

if __name__ == '__main__':
    rospy.init_node('tfInvNode', anonymous=True)
    node = TfInvNode()
    node.run()
    rospy.spin()
