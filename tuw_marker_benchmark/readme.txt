# To store collected data call the StoreData service providing a directory path
# Map file will be safed to map.json
# MarkerDetections will be saved to marker_detections.json
#
rosservice call /tuw_marker_benchmark/store_data /home/privacy/Documents/ros/workspace/src/tuw_marker_detection/tuw_marker_benchmark/data


# To run benchmark for data directories

# Static:
#  - ArUco:
./run_benchmark.py aruco ../data/static/aruco/map_base0_pos0/ ../data/static/aruco/map_base0_pos1/ ../data/static/aruco/map_base0_pos2/ ../data/static/aruco/map_base0_pos3/ ../data/static/aruco/map_base0_pos4/ ../data/static/aruco/map_base0_pos5/ ../data/static/aruco/map_base0_pos6/
#  - Ellipses
./run_benchmark.py ellipses ../data/static/ellipses/map_base0_pos0/ ../data/static/ellipses/map_base0_pos1/ ../data/static/ellipses/map_base0_pos2/ ../data/static/ellipses/map_base0_pos3/ ../data/static/ellipses/map_base0_pos4/ ../data/static/ellipses/map_base0_pos5/ ../data/static/ellipses/map_base0_pos6/
#  - Dice
./run_benchmark.py dice ../data/static/dice/map_base0_pos0/ ../data/static/dice/map_base0_pos1/ ../data/static/dice/map_base0_pos2/ ../data/static/dice/map_base0_pos3/ ../data/static/dice/map_base0_pos4/ ../data/static/dice/map_base0_pos5/ ../data/static/dice/map_base0_pos6/
#  - Checkerboard
./run_benchmark.py checkerboard ../data/static/checkerboard/map_base0_pos0/ ../data/static/checkerboard/map_base0_pos1/ ../data/static/checkerboard/map_base0_pos2/ ../data/static/checkerboard/map_base0_pos3/ ../data/static/checkerboard/map_base0_pos4/ ../data/static/checkerboard/map_base0_pos5/ ../data/static/checkerboard/map_base0_pos6/