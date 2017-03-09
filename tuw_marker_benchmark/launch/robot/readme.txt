### localization ###
roslaunch tuw_marker_benchmark robot_bag_localization.launch bag_file:=raw/bag_45d_2.bag room:=roblab_corridor_marker_45d
roslaunch tuw_marker_benchmark robot_bag_localization.launch bag_file:=raw/bag_0d_1.bag room:=roblab_corridor_marker_0d

### inital pose ###

rostopic pub /p3dx/amcl/initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { seq: 0, stamp: { secs: 1488818071, nsecs: 78390393 }, frame_id: map }, pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] } }"

Type: geometry_msgs/PoseWithCovarianceStamped

/p3dx/amcl/initialpose
header: 
  seq: 0
  stamp: 
    secs: 1488818071
    nsecs:  78390393
  frame_id: map
pose: 
  pose: 
    position: 
      x: 0.0336727797985
      y: 0.0628539323807
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0125449260988
      w: 0.999921309318
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

### rosbag ###

rosbag record --duration=30 /camera/camera_info /camera/image_raw/compressed /markerMap /markerDetectionAruco /rosout /rosout_agg /tf /tf_static -O robot_test.bag

/camera/camera_info
/camera/image_raw/compressed
/markerMap
/rosout
/rosout_agg
/tf
/tf_static

- Everything
rosbag record --duration=60 /camera/camera_info /camera/image_raw/compressed /camera/parameter_descriptions /camera/parameter_updates /diagnostics /p3dx/RosAria/parameter_descriptions /p3dx/RosAria/parameter_updates /p3dx/aria/odom /p3dx/battery_recharge_state /p3dx/battery_state_of_charge /p3dx/battery_voltage /p3dx/bumper_state /p3dx/cmd_vel /p3dx/hokuyo/parameter_descriptions /p3dx/hokuyo/parameter_updates /p3dx/joint_cmds /p3dx/joint_measures /p3dx/joint_states /p3dx/joy /p3dx/laser_front/scan /p3dx/laser_front/scan/raw /p3dx/motors_state /p3dx/parameter_descriptions /p3dx/parameter_updates /rosout /rosout_agg /tf /tf_static