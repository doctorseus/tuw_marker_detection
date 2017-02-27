from benchmark import Benchmark

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

def calculate_mean(data):
    return sum(data) / (len(data) - 1)

def calculate_variance(data):
    mean = calculate_mean(data)
    return sum(map(lambda x: math.pow(x - mean, 2), data)) / (len(data) - 1)

def calculate_distribution(data):
    return math.sqrt(calculate_variance(data))

class DataPoint():

    def __init__(self, data_id):
        self.data_id = data_id
        self.x = []
        self.y = []

    def add(self, x, y):
        self.x.append(x)
        self.y.append(y)

    def __str__(self):
        return 'DataPoint[id=%s, x[mean=%s, var=%s], y[mean=%s, var=%s]' % (self.data_id, calculate_mean(self.x), calculate_distribution(self.x), calculate_mean(self.y), calculate_distribution(self.y))


class StaticBenchmark(Benchmark):

    def __init__(self, samples):
        self.samples = samples
        self.data_points = {}

    def run(self):
        processed = 0
        for sample in self.samples:
            if self.process_sample(sample):
                processed += 1
        print('Total samples processed: %d' % processed)

    def euler_degree_from_quaternion(self, quaternion):
        euler = transformations.euler_from_quaternion(list(reversed(quaternion)), axes='rzyx')
        return map(lambda radian: radian * 180/math.pi, euler)

    def get_data_point_id(self, map_marker):
        angles = self.euler_degree_from_quaternion(map_marker.pose.orientation)
        angle_diff = 180 + round(angles[0], 2)
        return '%s' % str(angle_diff)

    def process_sample(self, sample):
        bmap = sample.bmap
        marker_detection = sample.bmarker_detection

        # Only one marker should be present
        if len(marker_detection.markers) > 1:
            return False

        kdtree = self.create_kdtree(bmap)

        for detected_marker in marker_detection.markers:
            map_marker = self.match_marker(kdtree, bmap, detected_marker)

            if map_marker is None:
                print('Marker could not be matched to any map marker.')
                return False

            # print('%s == %s' % (str(detected_marker.id), str(map_marker.id)))

            data_point_id = self.get_data_point_id(map_marker)

            # Setup TF tree
            tfb = Buffer(cache_time=rospy.Duration(30.0), debug=False)
            tfb.set_transform(BPose_to_Transform_msg(marker_detection.camera, 'odom', 'camera'), 'default_authority')
            tfb.set_transform(BPose_to_Transform_msg(map_marker.pose, 'odom', 'map_marker'), 'default_authority')
            tfb.set_transform(BPose_to_Transform_msg(detected_marker.pose, 'odom', 'detected_marker'), 'default_authority')

            marker_tfm = tfb.lookup_transform('map_marker', 'detected_marker', rospy.Time(0))

            x = marker_tfm.transform.translation.x * 100
            y = marker_tfm.transform.translation.y * 100

            self.data_points.setdefault(data_point_id, DataPoint(data_point_id)).add(x, y)


            '''
            q_camera = marker_detection.camera.orientation
            q_camera = [y for x in [[q_camera[3]], q_camera[0:3]] for y in x]
            q_map_marker = map_marker.pose.orientation
            q_map_marker = [y for x in [[q_map_marker[3]], q_map_marker[0:3]] for y in x]

            e_camera = transformations.euler_from_quaternion(q_camera)
            e_map_marker = transformations.euler_from_quaternion(q_map_marker)

            #print(map(lambda radian: radian * 180/math.pi, e_camera))
            #print(map(lambda radian: radian * 180/math.pi, e_map_marker))

            dot = q_camera[0] * q_map_marker[0] +  q_camera[1] * q_map_marker[1] + q_camera[2] * q_map_marker[2] + q_camera[3] * q_map_marker[3]
            #dot = e_camera[0] * e_map_marker[0] + e_camera[1] * e_map_marker[1] + e_camera[2] * e_map_marker[2]

            angle = math.acos(dot)
            #angle = math.acos(dot / math.sqrt(sum(map(lambda v: math.pow(v, 2), e_camera))) / math.sqrt(sum(map(lambda v: math.pow(v, 2), e_map_marker))))
            angle_deg = angle * 180/math.pi

            #print(angle_deg)

            if angle_deg not in self.state.setdefault('data_points', []):
                self.state.setdefault('data_points', []).append(angle_deg)
            '''
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
        keys = map(float, self.data_points.keys())
        keys.sort()

        for key in keys:
             print(str(self.data_points[str(key)]))

        #for key in sorted(map(lambda d: float(d), self.data_points.keys())):
        #    print(str(self.data_points[key]))
