#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class BenchmarkNode:

    def __init__(self):
        pass

    def run(self):
        self.tfb = Buffer()
        self.tfSub = TransformListener(self.tfb)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)

        self.vizPub = rospy.Publisher("benchmark_visualization", Marker, queue_size=10)


    def tf_callback(self, data):
        #rospy.loginfo('%s', self.tfBuf.all_frames_as_string())

        #t = self.tfb.getLatestCommonTime("/p3dx/camera", "/p3dx/odom")
        try:
            tf = self.tfb.lookup_transform("p3dx/odom", "p3dx/camera", rospy.Time(0))
            rospy.loginfo('%s', tf)

            # http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
            points = Marker()
            points.header.frame_id = "p3dx/odom"
            points.header.stamp = rospy.Time.now()

            points.ns = "points_and_lines"
            points.action = Marker.ADD

            points.pose.orientation.w = 1.0

            points.id = 0

            points.type = Marker.POINTS

            # POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.1
            points.scale.y = 0.1

            # Points are green
            points.color.g = 1.0
            points.color.a = 1.0

            '''
            f = 0.0
            for i in range(0, 100):
                x = 5 * math.sin(f + i / 100.0 * 2 * math.pi)
                y = 5 * math.cos(f + i / 100.0 * 2 * math.pi)

                p = Point()
                p.x = x
                p.y = y
                p.z = 0

                points.points.append(p)
            '''
            p = Point()
            p.x = tf.transform.translation.x
            p.y = tf.transform.translation.y
            p.z = 0
            points.points.append(p)

            self.vizPub.publish(points)
        except Exception as e:
            rospy.logerr(e.message)



        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

if __name__ == '__main__':
    rospy.init_node('markerBenchmarkNode', anonymous=True)
    node = BenchmarkNode()
    node.run()
    rospy.spin()

'''
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
'''