#!/usr/bin/env python
import json
import sys
import os
from analysis import StaticBenchmark
from models import BMap, BMarkerDetection, BMarkerDetectionWithCameraPose


class SamplePair:

    def __init__(self, bmap, bmarker_detection):
        self.bmap = bmap
        self.bmarker_detection = bmarker_detection

if __name__ == '__main__':
    samples = []

    sample_directory_paths = sys.argv[1:]
    sample_paths = map(lambda directory_path: (os.path.join(directory_path, 'map.json'), os.path.join(directory_path, 'marker_detections.json')), sample_directory_paths)

    for (map_path, detections_path) in sample_paths:
        print('Load %s - %s' % (map_path, detections_path))

        with open(map_path, 'r') as f:
            bmap = BMap.from_json(json.load(f))

        with open(detections_path, 'r') as f:
            detection_objs = json.load(f)
            bdetections = map(BMarkerDetectionWithCameraPose.from_json, detection_objs)

        print(' - Found %d marker detections.' % len(bdetections))

        for bdetection in bdetections:
            samples.append(SamplePair(bmap, bdetection))

    print('Total samples found: %d' % len(samples))

    benchmark = StaticBenchmark(samples)
    benchmark.run()
    benchmark.store_results()
