import re


class BMap:

    def __init__(self, markers):
        self.markers = markers

    def __str__(self):
        return '[' + (', '.join(map(str, self.markers))) + ']'

    @staticmethod
    def from_MarkerWithCovarianceArray_msg(msg):
        return BMap(map(lambda mm: BMarker.from_Marker_msg(mm.marker), msg.markers))


class BMarker:

    def __init__(self, id, pose):
        self.id = id
        self.pose = pose

    def __str__(self):
        pose_str = re.sub('\s+', ' ', str(self.pose)).strip()
        return 'Marker[id=%s, pose=%s]' % (str(self.id), pose_str)

    @staticmethod
    def from_Marker_msg(msg):
        id = msg.ids[0] if len(msg.ids) > 0 else None;
        pose = msg.pose
        return BMarker(id, pose)


class BMarkerDetection:

    def __init__(self, markers):
        self.markers = markers

    def __str__(self):
        return '[' + (', '.join(map(str, self.markers))) + ']'

    @staticmethod
    def from_MarkerDetection_msg(msg):
        return BMap(map(lambda marker: BMarker.from_Marker_msg(marker), msg.markers))