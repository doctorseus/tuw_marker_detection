#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage

class BenchmarkNode:

    def __init__(self):
        pass

    def run(self):
        self.tfBuf = Buffer()
        self.tfSub = TransformListener(self.tfBuf)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)


    def tf_callback(self, data):
        rospy.loginfo('%s', self.tfBuf.all_frames_as_string())

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