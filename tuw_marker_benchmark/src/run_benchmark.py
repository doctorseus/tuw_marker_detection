#!/usr/bin/env python
import rospy
import math
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from marker_msgs.msg import MarkerWithCovarianceArray

class VisualHelper:

    @staticmethod
    def createMarker(frame, namespace, id, scale, (r, g, b, a)):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.id = id

        marker.scale.x = scale
        marker.scale.y = scale

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        return marker

    @staticmethod
    def createPoints(frame, namespace, id, scale, color):
        marker = VisualHelper.createMarker(frame, namespace, id, scale, color)
        marker.type = Marker.POINTS
        return marker


class BenchmarkNode:

    def __init__(self):
        pass

    def run(self):
        self.tfb = Buffer()
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/map", MarkerWithCovarianceArray, self.map_callback)

        self.vizPub = rospy.Publisher("benchmark_visualization", Marker, queue_size=10)

    def tf_callback(self, data):
        #rospy.loginfo('%s', self.tfBuf.all_frames_as_string())

        try:
            tf = self.tfb.lookup_transform("p3dx/odom", "p3dx/camera", rospy.Time(0))
            #rospy.loginfo('%s', tf)

            # http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
            marker = VisualHelper.createPoints("p3dx/odom", "camera", 1, 0.2, (1, 0, 0, 1))

            '''
            f = 0.0
            for i in range(0, 100):
                x = 5 * math.sin(f + i / 100.0 * 2 * math.pi)
                y = 5 * math.cos(f + i / 100.0 * 2 * math.pi)

                p = Point()
                p.x = x
                p.y = y
                p.z = 0

                marker.points.append(p)
            '''
            p = Point()
            p.x = tf.transform.translation.x
            p.y = tf.transform.translation.y
            p.z = 0
            marker.points.append(p)

            self.vizPub.publish(marker)
        except Exception as e:
            rospy.logerr(e.message)

    def map_callback(self, data):
        #rospy.loginfo('Map update...')
        pass

if __name__ == '__main__':
    rospy.init_node('markerBenchmarkNode', anonymous=True)
    node = BenchmarkNode()
    node.run()
    rospy.spin()
