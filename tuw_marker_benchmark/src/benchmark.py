import os
import json
import rospy
import math
from models import BMap, BMarkerDetection
from scipy.spatial import KDTree
import numpy
from tf2_ros import Buffer, TransformListener, TFMessage, TransformStamped
import tf.transformations as transformations


def BPose_to_Transform_msg(bpose, frame_id, child_frame_id):
    tf = TransformStamped()
    tf.header.stamp = rospy.Time(0)
    tf.header.frame_id = frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = bpose.position[0]
    tf.transform.translation.y = bpose.position[1]
    tf.transform.translation.z = bpose.position[2]
    tf.transform.rotation.x = bpose.orientation[0]
    tf.transform.rotation.y = bpose.orientation[1]
    tf.transform.rotation.z = bpose.orientation[2]
    tf.transform.rotation.w = bpose.orientation[3]
    return tf


class Benchmark:

    def __init__(self, samples):
        self.samples = samples

    def run(self):
        processed = 0
        for sample in self.samples:
            if self.process_sample(sample):
                processed += 1
        print('Total samples processed: %d' % processed)

    def process_sample(self, sample):
        bmap = sample.bmap
        marker_detection = sample.bmarker_detection

        kdtree = self.create_kdtree(bmap)
        for detected_marker in marker_detection.markers:
            map_marker = self.match_marker(kdtree, bmap, detected_marker)
            
            if map_marker is None:
                print('Marker could not be matched to any map marker.')
                return False

            #print('%s == %s' % (str(detected_marker.id), str(map_marker.id)))

            # Setup TF tree
            tfb = Buffer(cache_time=rospy.Duration(30.0), debug=False)
            tfb.set_transform(BPose_to_Transform_msg(marker_detection.camera, 'odom', 'camera'), 'default_authority')
            tfb.set_transform(BPose_to_Transform_msg(map_marker.pose, 'odom', 'map_marker'), 'default_authority')
            tfb.set_transform(BPose_to_Transform_msg(detected_marker.pose, 'odom', 'detected_marker'), 'default_authority')

            marker_tfm = tfb.lookup_transform('map_marker', 'detected_marker', rospy.Time(0))


            q_camera = marker_detection.camera.orientation
            q_map_marker = map_marker.pose.orientation

            dot = q_camera[0] * q_map_marker[0] +  q_camera[1] * q_map_marker[1] + q_camera[2] * q_map_marker[2] + q_camera[3] * q_map_marker[3]
            angle = math.acos(dot)

            #print(angle)
        return True

    def create_kdtree(self, bmap):
        position_data = map(lambda marker: marker.pose.position, bmap.markers)
        return KDTree(position_data)

    def match_marker(self, kdtree, bmap, detected_marker):
        # TODO: Implement id matcher

        # Search for nearest neighbor marker
        detected_position = detected_marker.pose.position
        found = kdtree.query(detected_position, 1)
        dist = found[0]
        idx = found[1]

        #print('dist=%f, idx=%d' % (dist, idx))

        return bmap.markers[idx]

    def store_results(self):
        pass
