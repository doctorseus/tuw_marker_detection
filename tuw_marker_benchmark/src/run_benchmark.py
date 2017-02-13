#!/usr/bin/env python
from benchmark import Benchmark

if __name__ == '__main__':
    benchmark = Benchmark()
    benchmark.run()
    benchmark.store_results()
