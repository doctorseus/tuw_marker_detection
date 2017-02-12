#!/usr/bin/env python
import rospy
import math
from Queue import *
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker as VisualMarker
from geometry_msgs.msg import Point
from marker_msgs.msg import MarkerWithCovarianceArray
from marker_msgs.msg import MarkerWithCovariance
from marker_msgs.msg import Marker
from marker_msgs.msg import MarkerDetection
from benchmark import Benchmark
from models import BMap, BMarkerDetection

class VisualHelper:

    @staticmethod
    def createVisualMarker(frame, namespace, id, scale, (r, g, b, a)):
        marker = VisualMarker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.action = VisualMarker.ADD

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
        marker = VisualHelper.createVisualMarker(frame, namespace, id, scale, color)
        marker.type = VisualMarker.POINTS
        return marker

class BenchmarkNode:

    def __init__(self):
        self.tfb = Buffer()
        self.map = None
        self.marker_detection_queue = Queue()
        self.benchmark = None

    def run(self):
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/map", MarkerWithCovarianceArray, self.map_callback)
        rospy.Subscriber("/markerDetection", MarkerDetection, self.marker_detection_callback)
        self.vizPub = rospy.Publisher("benchmark_visualization", VisualMarker, queue_size=10)

    def tf_callback(self, msg):
        pass
        '''
        try:
            tf = self.tfb.lookup_transform("p3dx/odom", "p3dx/camera", rospy.Time(0))
            #rospy.loginfo('%s', tf)

            # http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
            marker = VisualHelper.createPoints("p3dx/odom", "camera", 1, 0.2, (1, 0, 0, 1))


            # f = 0.0
            # for i in range(0, 100):
            #     x = 5 * math.sin(f + i / 100.0 * 2 * math.pi)
            #     y = 5 * math.cos(f + i / 100.0 * 2 * math.pi)
            #
            #     p = Point()
            #     p.x = x
            #     p.y = y
            #     p.z = 0
            #
            #     marker.points.append(p)

            p = Point()
            p.x = tf.transform.translation.x
            p.y = tf.transform.translation.y
            p.z = 0
            marker.points.append(p)

            self.vizPub.publish(marker)
        except Exception as e:
            rospy.logerr(e.message)
        '''

    def map_callback(self, msg):
        # Map is read once and cached from there on (new messages will not update the map used)
        if self.map is None:
            rospy.loginfo('Read marker map message. Found %s markers.' % len(msg.markers))
            self.map = BMap.from_MarkerWithCovarianceArray_msg(msg)
            rospy.loginfo(str(self.map))

        # Poke processing if map arrives
        self.process_backlog()

    def marker_detection_callback(self, msg):
        self.marker_detection_queue.put(msg)

        # Process the new message
        self.process_backlog()

    def process_backlog(self):
        # Only process messages if every component is ready
        if self.is_every_requirement_available():
            while not self.marker_detection_queue.empty():
                self.process_marker_detection(self.marker_detection_queue.get())

    def is_every_requirement_available(self):
        if self.map is not None:
            return True
        else:
            return False

    def process_marker_detection(self, msg):
        # Init benchmark if not set
        if self.benchmark is None:
            self.benchmark = Benchmark(self.map)
        self.benchmark.process_marker_detection(BMarkerDetection.from_MarkerDetection_msg(msg))

if __name__ == '__main__':
    rospy.init_node('markerBenchmarkNode', anonymous=True)
    node = BenchmarkNode()
    node.run()
    rospy.spin()
