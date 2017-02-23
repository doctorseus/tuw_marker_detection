#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from marker_msgs.msg import MarkerDetection, Marker

class MarkerDetectionToPose:

    def __init__(self):
        pass

    def run(self):
        rospy.Subscriber("markerDetection", MarkerDetection, self.marker_detection_callback)
        self.posePub = rospy.Publisher("markerDetectionPose", PoseWithCovarianceStamped, queue_size=10)

    def marker_detection_callback(self, mmsg):
        for marker in mmsg.markers:
            pmsg = PoseWithCovarianceStamped()
            pmsg.header = mmsg.header
            pmsg.pose.pose = marker.pose
            self.posePub.publish(pmsg)

if __name__ == '__main__':
    rospy.init_node('detectionToPoseNode', anonymous=True)
    node = MarkerDetectionToPose()
    node.run()
    rospy.spin()