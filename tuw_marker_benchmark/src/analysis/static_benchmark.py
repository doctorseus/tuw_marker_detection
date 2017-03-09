from matplotlib.lines import Line2D
from benchmark import Benchmark

import os
import json
import rospy
import math
from scipy.spatial import KDTree
import numpy
from tf2_ros import Buffer, TransformListener, TFMessage, TransformStamped
import tf.transformations as transformations


import numpy as np
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import patches
matplotlib.use('SVG')
import matplotlib.pyplot as plt


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

def calculate_standard_deviation(data):
    return math.sqrt(calculate_variance(data))

def calculate_fences(data):
    if len(data) == 0:
        return 0, 0
    datac = list(data)
    datac.sort()
    q1 = datac[len(datac)/4]
    q3 = datac[(len(datac)/4)*3]
    h = 1.5 * (q3 - q1)
    return q1-h, q3+h

def get_values(data):
    return calculate_mean(data), calculate_variance(data), calculate_standard_deviation(data), calculate_fences(data)


class DataPoint:

    def __init__(self, data_id):
        self.data_id = data_id
        self.x = []
        self.y = []
        self.z = []

    def add(self, x, y, z):
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

    def get_values_x(self):
        return get_values(self.x)

    def get_values_y(self):
        return get_values(self.y)

    def get_inner_x(self):
        (lower_fence, higher_fence) = calculate_fences(self.x)
        return filter(lambda x: lower_fence <= x <= higher_fence, self.x)

    def get_inner_y(self):
        (lower_fence, higher_fence) = calculate_fences(self.y)
        return filter(lambda x: lower_fence <= x <= higher_fence, self.y)

    def get_inner_z(self):
        (lower_fence, higher_fence) = calculate_fences(self.z)
        return filter(lambda x: lower_fence <= x <= higher_fence, self.z)

    def __str__(self):
        values_x = self.get_values_x()
        values_y = self.get_values_y()
        return 'DataPoint[id=%s, x[mean=%s, var=%s, q=%s, fences=%s], y[mean=%s, var=%s, q=%s, fences=%s]' % (self.data_id, values_x[0], values_x[1], values_x[2], values_x[3], values_y[0], values_y[1], values_y[2], values_y[3])


class StaticBenchmark(Benchmark):

    def __init__(self, name, samples):
        self.name = name
        self.samples = samples
        self.data_points = {
            '22.5'   : DataPoint('22.5'),
            '45.0'   : DataPoint('45.0'),
            '67.5'   : DataPoint('67.5'),
            '90.0'   : DataPoint('90.0'),
            '112.5'  : DataPoint('112.5'),
            '135.0'  : DataPoint('135.0'),
            '157.5'  : DataPoint('157.5')
        }

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
            map_marker, dist = self.match_marker(kdtree, bmap, detected_marker)

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
            z = marker_tfm.transform.translation.z * 100

            #self.data_points.setdefault(data_point_id, DataPoint(data_point_id)).add(x, y)
            self.data_points[data_point_id].add(x, y, z)

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

        return bmap.markers[idx], dist

    def store_results(self):
        keys = map(float, self.data_points.keys())
        keys.sort()

        fig = plt.figure(num=None, figsize=(50, 10), dpi=90, facecolor='w', edgecolor='k')

        i = 0
        num = len(keys)
        for key in keys:
            data_point = self.data_points[str(key)]

            subp = fig.add_subplot(100 + (10 * num) + (i+1), aspect=1)

            values_x = data_point.get_values_x()
            values_y = data_point.get_values_y()

            print(values_x)
            print(values_y)
            '''
            elps = patches.Ellipse((values_x[0], values_y[0]), values_x[2], values_y[2], angle=0, linewidth=1, fill=False, zorder=2)
            subp.add_patch(elps)

            plt.xlim(-1.5, 1.5)
            plt.ylim(-1.5, 1.5)
            '''

            '''
            elps = patches.Ellipse((0, 0), values_x[2], values_y[2], angle=0, linewidth=1, fill=False, zorder=2)
            subp.add_patch(elps)

            plt.xlim(-0.5, 0.5)
            plt.ylim(-0.5, 0.5)
            '''

            elps = patches.Ellipse((values_x[0], values_y[0]), values_x[2], values_y[2], angle=0, linewidth=1, fill=False, zorder=2)
            subp.add_patch(elps)

            arrow = patches.Arrow(0, 0, values_x[0], values_y[0], 0.3, facecolor='#F44336', edgecolor='#3F51B5')
            subp.add_patch(arrow)

            plt.xlim(-1.5, 1.5)
            plt.ylim(-1.5, 1.5)

            fig.subplots_adjust(wspace=0.1, hspace=0.1)

            i += 1

        plt.savefig('fig_%s.svg' % self.name, bbox_inches='tight')
        plt.savefig('fig_%s.pdf' % self.name, bbox_inches='tight')

        ####

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        i = 0
        num = len(keys)

        nucolors = map(Color.from_hex, ['#c02e1d', '#d94e1f', '#f16c20', '#ef8b2c', '#ecaa38', '#ebc844', '#a2b86c', '#5ca793', '#1395ba', '#117899', '#0f5b78', '#0d3c55'][::2])
        colors = map(Color.from_hex, ['#F44336', '#3F51B5', '#009688', '#FFC107', '#FF5722', '#9C27B0', '#03A9F4', '#8BC34A', '#FF9800', '#E91E63', '#323232', '#2196F3','#4CAF50', '#FFEB3B', '#673AB7', '#00BCD4', '#CDDC39', '#795548', '#9E9E9E', '#607D8B', '#F44336', '#3F51B5', '#009688', '#323232', '#F44336', '#3F51B5', '#009688', '#FFC107', '#9C27B0', '#03A9F4', '#8BC34A', '#FF9800', '#E91E63', '#2196F3', '#4CAF50', '#FFEB3B', '#673AB7', '#00BCD4', '#CDDC39', '#795548', '#9E9E9E', '#607D8B'])

        f = open('test.html', 'w')
        f.write('<html><body>%s</body></html>' % map(lambda c: ('<div style="width: 40px; height: 40px; background-color: %s"></div>' % c.to_hex()), nucolors))
        f.close()

        for key, color in zip(keys, nucolors):
            data_point = self.data_points[str(key)]



            #subp = fig.add_subplot(100 + (10 * num) + (i+1), aspect=1, projection='3d')
            ax.scatter(xs=data_point.x, ys=data_point.y, zs=data_point.z, c=color.to_hex(), edgecolor=color.darker(2).to_hex(), linewidth=1, marker='o', label=str(key))

            #plt.savefig(('fig_3d_%s_%sgrad' % (self.name, str(key).replace('.', '-'))))

            i += 1

        (xlim_low, xlim_up) = (-4.0, 4.0)
        (ylim_low, ylim_up) = (-1.0, 1.0)
        (zlim_low, zlim_up) = (-4.0, 4.0)

        #ax.plot([xlim_low, xlim_up], [0, 0], [0, 0], c='b')
        #ax.plot([0, 0], [ylim_low, ylim_up], [0, 0], c='r')
        #ax.plot([0, 0], [0, 0], [zlim_low, zlim_up], c='g')
        ax.plot([0, xlim_up*0.5], [0, 0], [0, 0], c='b')
        ax.plot([0, 0], [0, ylim_up*0.5], [0, 0], c='r')
        ax.plot([0, 0], [0, 0], [0, zlim_up*0.5], c='g')

        ax.set_xlim(xlim_low, xlim_up)
        ax.set_ylim(ylim_low, ylim_up)
        ax.set_zlim(zlim_low, zlim_up)

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        #fig.dist = 6
        #ax.dist = 9
        fig.subplots_adjust(wspace=0.1, hspace=0.1)

        #plt.tight_layout()
        plt.legend()
        plt.savefig('fig_3d_%s.svg' % self.name, bbox_inches='tight')
        plt.savefig('fig_3d_%s.pdf' % self.name, bbox_inches='tight')


        '''
        fig = plt.figure()

        subp = fig.add_subplot(111, aspect=1s)



        y = 1.0
        x = 1.0

        for key in keys:
            data_point = self.data_points[str(key)]
            values_x = data_point.get_values_x()
            values_y = data_point.get_values_y()

            print(values_x)
            print(values_y)
             #print(str(self.data_points[str(key)]))

            off_x = x + values_x[0]
            off_y = y + values_y[0]

            elps = patches.Ellipse((off_x, off_y), values_x[2], values_y[2], angle=0, linewidth=1, fill=False, zorder=2)
            subp.add_patch(elps)


            refc = patches.Ellipse((x, y), 0.05, 0.05, angle=0, linewidth=0, fill=True, zorder=2, facecolor='red')
            subp.add_patch(refc)

            #line = matplotlib.lines.Line2D
            line = Line2D(xdata=[0, 1], ydata=[0, 1])
            subp.add_line(line)
            #, facecolor='yellow', edgecolor='yellow'

            x += 1


        plt.xlim(0, 10.0)
        plt.ylim(0.5, 1.5)

        plt.savefig('myfig')

        '''


def color_variant(hex_color, brightness_offset=1):
    """ takes a color like #87c95f and produces a lighter or darker variant """
    if len(hex_color) != 7:
        raise Exception("Passed %s into color_variant(), needs to be in #87c95f format." % hex_color)
    rgb_hex = [hex_color[x:x+2] for x in [1, 3, 5]]
    new_rgb_int = [int(hex_value, 16) + brightness_offset for hex_value in rgb_hex]
    new_rgb_int = [min([255, max([0, i])]) for i in new_rgb_int] # make sure new values are between 0 and 255
    # hex() produces "0x88", we want just "88"
    return "#" + "".join([hex(i)[2:] for i in new_rgb_int])


class Color:

    def __init__(self, r, g, b):
        self.r = r % 255
        self.g = g % 255
        self.b = b % 255

    @staticmethod
    def from_hex(hex_color):
        from matplotlib.colors import hex2color
        (r, g, b) = hex2color(hex_color)
        return Color(r*255, g*255, b*255)

    def brighter(self, steps=1):
        val = 15 * steps
        return Color(min(self.r + val, 255), min(self.g + val, 255), min(self.b + val, 255))

    def darker(self, steps=1):
        val = 15 * steps
        return Color(max(0, self.r - val), max(0, self.g - val), max(self.b - val, 0))

    def to_hex(self):
        return '#%.2x%.2x%.2x' % (self.r, self.g, self.b)
