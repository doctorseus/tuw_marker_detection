#!/usr/bin/env python
import json
from benchmark import Benchmark
from models import BMap, BMarkerDetection, BMarkerDetectionWithCameraPose


class DataCollection:

    def __init__(self, bmap, bdetections):
        self.bmap = bmap
        self.bdetections = bdetections

if __name__ == '__main__':
    bmap = None
    bdetections = []

    with open('../data/static/aruco/map_base0_pos3/map.json', 'r') as f:
        bmap = BMap.from_json(json.load(f))

    with open('../data/static/aruco/map_base0_pos3/marker_detections.json', 'r') as f:
        detection_objs = json.load(f)
        bdetections = map(BMarkerDetectionWithCameraPose.from_json, detection_objs)

    benchmark = Benchmark(bmap, bdetections)
    benchmark.run()
    benchmark.store_results()
