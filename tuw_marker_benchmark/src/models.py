import re


class BMap:

    def __init__(self, markers):
        self.markers = markers

    def __str__(self):
        return 'BMap[' + (', '.join(map(str, self.markers))) + ']'

    @staticmethod
    def from_MarkerWithCovarianceArray_msg(msg):
        return BMap(map(lambda mm: BMarker.from_Marker_msg(mm.marker), msg.markers))

    @staticmethod
    def to_json(map_obj):
        return map(BMarker.to_json, map_obj.markers)


class BMarker:

    def __init__(self, id, pose):
        self.id = id
        self.pose = pose

    def __str__(self):
        return 'BMarker[id=%s, pose=%s]' % (str(self.id), str(self.pose))

    @staticmethod
    def from_Marker_msg(msg):
        id = msg.ids[0] if len(msg.ids) > 0 else None;
        pose = BPose.from_Pose_msg(msg.pose)
        return BMarker(id, pose)

    @staticmethod
    def to_json(marker_detection_obj):
        return {
            'id': marker_detection_obj.id,
            'pose': BPose.to_json(marker_detection_obj.pose)
        }

class BPose:

    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

    def __str__(self):
        return 'BPose[]'

    @staticmethod
    def from_Pose_msg(msg):
        pos = (msg.position.x, msg.position.y, msg.position.z)
        quad = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        return BPose(pos, quad)

    @staticmethod
    def to_json(pose_obj):
        return {
            'position': pose_obj.position,
            'orientation': pose_obj.orientation
        }


class BMarkerDetection:

    def __init__(self, header, markers):
        self.header = header
        self.markers = markers

    def __str__(self):
        return 'BMarkerDetection[' + (', '.join(map(str, self.markers))) + ']'

    @staticmethod
    def from_MarkerDetection_msg(msg):
        header = {
            'seq': msg.header.seq,
            'stamp': msg.header.stamp.to_time(),
            'frame_id': msg.header.frame_id
        }
        return BMarkerDetection(header, map(lambda marker: BMarker.from_Marker_msg(marker), msg.markers))

    @staticmethod
    def to_json(marker_detection_obj):
        return {
            'header': marker_detection_obj.header,
            'markers': map(BMarker.to_json, marker_detection_obj.markers)
        }
