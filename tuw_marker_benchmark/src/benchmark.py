import rospy


class Benchmark:
    def __init__(self, map):
        self.map = map

    def process_marker_detection(self, marker_detection):
        rospy.loginfo('Process %s' % (str(marker_detection)))