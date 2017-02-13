#!/usr/bin/env python
import json
from benchmark import Benchmark
from models import BMap, BMarkerDetection

if __name__ == '__main__':
    bmap = None
    bdetections = []

    with open('../data/map.json', 'r') as f:
        bmap = BMap.from_json(json.load(f))

    with open('../data/marker_detections.json', 'r') as f:
        detection_objs = json.load(f)
        bdetections = map(BMarkerDetection.from_json, detection_objs)

    benchmark = Benchmark(bmap, bdetections)
    benchmark.run()
    benchmark.store_results()
