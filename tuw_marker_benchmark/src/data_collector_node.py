#!/usr/bin/env python
import rospy
import os
import json
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage
from tuw_marker_benchmark.srv import StoreData, ClearData
from marker_msgs.msg import MarkerWithCovarianceArray
from marker_msgs.msg import MarkerDetection
from models import BMap, BMarkerDetection


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
        self.marker_detections.append(BMarkerDetection.from_MarkerDetection_msg(msg))

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
