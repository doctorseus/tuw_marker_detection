#!/usr/bin/env python
import rospy
import os
import json
from tf2_ros import Buffer, TransformListener, TFMessage
from tf2_geometry_msgs import *
from geometry_msgs.msg import PoseStamped
from tuw_marker_benchmark.srv import StoreData, ClearData
from marker_msgs.msg import MarkerWithCovarianceArray
from marker_msgs.msg import MarkerDetection
from models import BMap, BMarkerDetection, BPose

from visualization_msgs.msg import Marker as VisualMarker
from geometry_msgs.msg import Point


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


def transform_bpose(tfb, bpose, frame_id, target_frame):
    p = PoseStamped()
    p.header.stamp = rospy.Time(0)
    p.header.frame_id = frame_id
    p.pose.position.x = bpose.position[0]
    p.pose.position.y = bpose.position[1]
    p.pose.position.z = bpose.position[2]
    p.pose.orientation.x = bpose.orientation[0]
    p.pose.orientation.y = bpose.orientation[1]
    p.pose.orientation.z = bpose.orientation[2]
    p.pose.orientation.w = bpose.orientation[3]
    tp = tfb.transform(p, target_frame)
    return BPose.from_Pose_msg(tp.pose)


class DataCollectorNode:

    def __init__(self):
        self.tfb = Buffer()
        self.map = None
        self.marker_detections = []

    def run(self):
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/map", MarkerWithCovarianceArray, self.map_callback)
        rospy.Subscriber("/markerDetection", MarkerDetection, self.marker_detection_callback)

        rospy.Service(rospy.get_name() + '/store_data', StoreData, self.store_data_callback)
        rospy.Service(rospy.get_name() + '/clear_data', ClearData, self.clear_data_callback)

        self.vizPub = rospy.Publisher("benchmark_visualization", VisualMarker, queue_size=10)

    def clear_data_callback(self, req):
        count = len(self.marker_detections)
        self.map = None
        self.marker_detections = []
        return True, 'Successfully cleared data: Removed %d marker detections.' % count

    def store_data_callback(self, req):
        directory_path = req.directory_path
        self.store_data(directory_path, self.map, self.marker_detections)
        rospy.loginfo('Stored data in %s. [map=1, marker_detections=%d]' % (str(directory_path), len(self.marker_detections)))
        return True, 'Successfully stored data in %s. [map=1, marker_detections=%d]' % (str(directory_path), len(self.marker_detections))

    def tf_callback(self, msg):
        pass

    def map_callback(self, msg):
        self.map = BMap.from_MarkerWithCovarianceArray_msg(msg)

    def marker_detection_callback(self, msg):
        # Test if TF lookup is available withing timeout frame
        try:
            timeout = 1.0
            self.tfb.lookup_transform('p3dx/camera', 'p3dx/odom', rospy.Time(0), rospy.Duration(timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('TF lookup (\'p3dx/camera\', \'p3dx/odom\') failed after waiting for %d seconds. Skipping marker detection frame. TF publish to slow?', timeout)
            return

        # Marker are detected relative to camera position. Convert positions to absolute values.
        bmarker_detection = BMarkerDetection.from_MarkerDetection_msg(msg)

        for marker in bmarker_detection.markers:
            marker.pose = transform_bpose(self.tfb, marker.pose, 'p3dx/camera', 'p3dx/odom')

        self.marker_detections.append(bmarker_detection)

        ### FOR DEBUG PURPOSES ###
        # Show absolute 2D position of camera and detected markers via visualization msgs
        camera_tf = self.tfb.lookup_transform("p3dx/odom", "p3dx/camera", rospy.Time(0))

        vizmarker = VisualHelper.createPoints("p3dx/odom", "camera", 1, 0.2, (1, 0, 0, 1))
        p = Point()
        p.x = camera_tf.transform.translation.x
        p.y = camera_tf.transform.translation.y
        p.z = 0
        vizmarker.points.append(p)

        for marker in bmarker_detection.markers:
            p = Point()
            p.x = marker.pose.position[0]
            p.y = marker.pose.position[1]
            p.z = 0
            vizmarker.points.append(p)

        self.vizPub.publish(vizmarker)

    @staticmethod
    def store_data(directory, bmap, bmarker_detections):
        # Save map
        path_map = os.path.join(directory, 'map.json')
        with open(path_map, 'w') as f:
            json.dump(BMap.to_json(bmap), indent=2, separators=(',', ': '), fp=f)
            pass

        # Save detections
        path_marker_detections = os.path.join(directory, 'marker_detections.json')
        with open(path_marker_detections, 'w') as f:
            json_marker_detections = map(BMarkerDetection.to_json, bmarker_detections)
            json.dump(json_marker_detections, indent=2, fp=f)
            pass

if __name__ == '__main__':
    rospy.init_node('benchmarkDataCollectorNode', anonymous=True)
    node = DataCollectorNode()
    node.run()
    rospy.spin()
