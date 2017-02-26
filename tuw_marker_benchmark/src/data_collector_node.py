#!/usr/bin/env python
import rospy
import os
import json
import time
from tf2_ros import Buffer, TransformListener, TFMessage, TransformStamped
from tf2_py import ExtrapolationException, TransformException, LookupException, ConnectivityException
from tf2_geometry_msgs import *
from geometry_msgs.msg import PoseStamped
from tuw_marker_benchmark.srv import StoreData, ClearData
from marker_msgs.msg import MarkerWithCovarianceArray
from marker_msgs.msg import MarkerDetection
from models import BMap, BMarkerDetectionWithCameraPose, BPose

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


def transform_bpose(tfb, bpose, stamp, frame_id, target_frame, timeout=rospy.Duration(0.0)):
    p = PoseStamped()
    p.header.stamp = stamp  # Use MarkerDetection stamp for TF lookup (In case it's delayed)
    p.header.frame_id = frame_id
    p.pose.position.x = bpose.position[0]
    p.pose.position.y = bpose.position[1]
    p.pose.position.z = bpose.position[2]
    p.pose.orientation.x = bpose.orientation[0]
    p.pose.orientation.y = bpose.orientation[1]
    p.pose.orientation.z = bpose.orientation[2]
    p.pose.orientation.w = bpose.orientation[3]
    tp = tfb.transform(p, target_frame, timeout)
    return BPose.from_Pose_msg(tp.pose)


class DataCollectorNode:

    class Config:
        tf_camera = 'camera'
        tf_odom = 'odom'
        timeout = 0
        directory_path = '~/'

    config = Config()

    def __init__(self):
        self.tfSub = None
        self.tfb = Buffer(cache_time=rospy.Duration(30.0))
        self.map = None
        self.marker_detections = []
        self.store_timeout = None

        self.config.tf_camera = rospy.get_param("~tf_camera", self.config.tf_camera)
        self.config.tf_odom = rospy.get_param("~tf_odom", self.config.tf_odom)
        self.config.timeout = rospy.get_param("~timeout", self.config.timeout)
        self.config.directory_path = rospy.get_param("~directory_path", self.config.directory_path)

        # Timeout handler
        self.time_last_update = time.time()
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

    def run(self):
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/map", MarkerWithCovarianceArray, self.map_callback)
        rospy.Subscriber("/markerDetection", MarkerDetection, self.marker_detection_callback, queue_size=10)

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
        self.store_data(directory_path)
        return True, 'Successfully stored data in %s. [map=1, marker_detections=%d]' % (str(directory_path), len(self.marker_detections))

    def timer_callback(self, timer_ob):
        # Timeout handler
        if self.config.timeout > 0:
            time_delta = time.time() - self.time_last_update
            if time_delta > + self.config.timeout:
                self.store_data(self.config.directory_path)
                rospy.signal_shutdown('Timeout reached.')

    def tf_callback(self, msg):
        pass

    def map_callback(self, msg):
        self.map = BMap.from_MarkerWithCovarianceArray_msg(msg)

    def marker_detection_callback(self, msg):
        # Reset timeout handler
        self.time_last_update = time.time()

        try:
            # Get camera absolute position
            camera_tf = self.tfb.lookup_transform(self.config.tf_odom, self.config.tf_camera, msg.header.stamp)
            camera_pose = BPose.from_Transform_msg(camera_tf)

            # Marker are detected relative to camera position. Convert positions to absolute values.
            bmarker_detection = BMarkerDetectionWithCameraPose.from_MarkerDetection_msg(msg, camera_pose)

            for marker in bmarker_detection.markers:
                marker.pose = transform_bpose(self.tfb, marker.pose, msg.header.stamp, self.config.tf_camera, self.config.tf_odom)
            bmarker_detection.header['frame_id'] = self.config.tf_odom

            self.marker_detections.append(bmarker_detection)

        except (ExtrapolationException, LookupException, ConnectivityException) as e:
            rospy.logwarn('Skipped marker detection message: %s' % e)

        '''
        # marker_detections=577
        # marker_detections=578

        ### FOR DEBUG PURPOSES ###
        # Show absolute 2D position of camera and detected markers via visualization msgs
        camera_tf = self.tfb.lookup_transform(self.config.tf_odom, self.config.tf_camera, rospy.Time(0))

        vizmarker = VisualHelper.createPoints(self.config.tf_odom, "camera", 1, 0.2, (1, 0, 0, 1))
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
        '''

    def store_data(self, directory_path):
        bmap = self.map
        bmarker_detections = self.marker_detections

        if not os.path.exists(directory_path):
            os.makedirs(directory_path)

        # Save map
        path_map = os.path.join(directory_path, 'map.json')
        with open(path_map, 'w') as f:
            json.dump(BMap.to_json(bmap), indent=2, separators=(',', ': '), fp=f)
            pass

        # Save detections
        path_marker_detections = os.path.join(directory_path, 'marker_detections.json')
        with open(path_marker_detections, 'w') as f:
            json_marker_detections = map(BMarkerDetectionWithCameraPose.to_json, bmarker_detections)
            json.dump(json_marker_detections, indent=2, fp=f)
            pass

        rospy.loginfo('Stored data in %s. [map=1, marker_detections=%d]' % (str(directory_path), len(bmarker_detections)))

if __name__ == '__main__':
    rospy.init_node('benchmarkDataCollectorNode', anonymous=True)
    node = DataCollectorNode()
    node.run()
    rospy.spin()
    print("closed...")
