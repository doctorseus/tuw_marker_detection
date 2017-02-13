import os
import json
from models import BMap, BMarkerDetection
from scipy.spatial import KDTree

class Benchmark:

    def __init__(self, bmap, bmarker_detetcions):
        self.bmap = bmap
        self.bmarker_detetcions = bmarker_detetcions
        self.bmap_kdt = self.create_kdtree()

    def create_kdtree(self):
        position_data = map(lambda marker: marker.pose.position, self.bmap.markers)
        return KDTree(position_data)

    def run(self):

        for marker_detection in self.bmarker_detetcions:
            for detected_marker in marker_detection.markers:
                map_marker = self.match_marker(detected_marker)

                if map_marker is None:
                    print('Marker could not be matched to any map marker.')

                print('%d == %d' % (detected_marker.id, map_marker.id))

    def match_marker(self, detected_marker):
        # TODO: Implement id matcher

        # Search for nearest neighbor marker
        detected_position = detected_marker.pose.position
        found = self.bmap_kdt.query(detected_position, 1)
        dist = found[0]
        idx = found[1]

        print('dist=%f, idx=%d' % (dist, idx))

        return self.bmap.markers[idx]

    def store_results(self):
        pass
