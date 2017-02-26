#!/usr/bin/env python
import rospy
import re
from marker_msgs.msg import MarkerDetection, Marker, MarkerWithCovarianceArray, MarkerWithCovariance
from geometry_msgs.msg import Pose, Transform
from tf2_py import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Buffer, TransformListener, TFMessage, TransformStamped, TransformBroadcaster


# This node accepts a list of TFs (~tf_list) as parameter and republishes
# these poses relative to a given TF node (~tf_frame_id) as marker map.
# If ~partial_maps is True it will also publish maps if not all TFs are available.
class TfToMarkerMapNode:

    def __init__(self):
        self.tfb = Buffer()
        self.tfSub = None
        self.markerMapPub = None
        self.seq = 0

        # Read parameters
        self.tf_list = re.split("\s*,+\s*", rospy.get_param("~tf_list"))
        self.tf_frame_id = rospy.get_param("~tf_frame_id")
        if len(self.tf_list) <= 0:
            raise ValueError("tf_list parameter is empty")
        self.partial_maps = rospy.get_param("~partial_maps", False)

    def run(self):
        self.tfSub = TransformListener(self.tfb)
        self.markerMapPub = rospy.Publisher("map", MarkerWithCovarianceArray, queue_size=10)

    def publish_marker_map(self):
        msg = MarkerWithCovarianceArray()
        msg.header.frame_id = self.tf_frame_id
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()

        markers = []
        for idx, tf in enumerate(self.tf_list):
            try:
                tfs = self.tfb.lookup_transform(self.tf_frame_id, tf, rospy.Time(0))

                marker = Marker()
                marker.ids = [idx]
                marker.ids_confidence = [1]
                marker.pose = Pose()
                marker.pose.position = tfs.transform.translation
                marker.pose.orientation = tfs.transform.rotation

                marker_cov = MarkerWithCovariance()
                marker_cov.marker = marker
                markers.append(marker_cov)
            except (StandardError, LookupException, ConnectivityException, ExtrapolationException) as e:
                rospy.logwarn("Tf %s to %s lookup failed: %s" % (tf, self.tf_frame_id, e.message))

                # If not all TF lookups succeed return and do not publish a marker map message
                if not self.partial_maps:
                    return False

        msg.markers = markers
        self.markerMapPub.publish(msg)
        return True

if __name__ == '__main__':
    rospy.init_node('tfToMarkerMapNode', anonymous=True)
    try:
        node = TfToMarkerMapNode()
        node.run()
    except StandardError as e:
        rospy.logerr("Failed to start %s: %s" % (rospy.get_name(), e.message))
        rospy.signal_shutdown("Failed to start %s: %s" % (rospy.get_name(), e.message))
        exit(1)

    # publish with a rate of 10 Hz the marker map
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            node.publish_marker_map()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass