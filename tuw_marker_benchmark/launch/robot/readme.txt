rostopic pub /p3dx/amcl/initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { seq: 0, stamp: { secs: 1488818071, nsecs: 78390393 }, frame_id: map }, pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] } }"

inital pose:

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

rosbag:

rosbag record --duration=30 /camera/camera_info /camera/image_raw/compressed /markerMap /markerDetectionAruco /rosout /rosout_agg /tf /tf_static -O robot_test.bag

/camera/camera_info
/camera/image_raw/compressed
/markerMap
/rosout
/rosout_agg
/tf
/tf_static


/arMarkerAruco/parameter_descriptions
/arMarkerAruco/parameter_updates
/arPoseEstimationAruco/parameter_descriptions
/arPoseEstimationAruco/parameter_updates
/camera/camera_info
/camera/image_raw
/camera/image_raw/compressed
/camera/image_raw/compressed/parameter_descriptions
/camera/image_raw/compressed/parameter_updates
/camera/image_raw/compressedDepth
/camera/image_raw/compressedDepth/parameter_descriptions
/camera/image_raw/compressedDepth/parameter_updates
/camera/image_raw/theora
/camera/image_raw/theora/parameter_descriptions
/camera/image_raw/theora/parameter_updates
/camera/image_thumbnail
/camera/image_thumbnail/compressed
/camera/image_thumbnail/compressed/parameter_descriptions
/camera/image_thumbnail/compressed/parameter_updates
/camera/image_thumbnail/compressedDepth
/camera/image_thumbnail/compressedDepth/parameter_descriptions
/camera/image_thumbnail/compressedDepth/parameter_updates
/camera/image_thumbnail/theora
/camera/image_thumbnail/theora/parameter_descriptions
/camera/image_thumbnail/theora/parameter_updates
/camera/parameter_descriptions
/camera/parameter_updates
/diagnostics
/fiducialsAruco
/map
/map_metadata
/markerDetectionAruco
/markerMap
/markersAruco
/p3dx/amcl/initialpose
/p3dx/amcl/parameter_descriptions
/p3dx/amcl/parameter_updates
/p3dx/aria/odom
/p3dx/battery_recharge_state
/p3dx/battery_state_of_charge
/p3dx/battery_voltage
/p3dx/bumper_state
/p3dx/cmd_vel
/p3dx/hokuyo/parameter_descriptions
/p3dx/hokuyo/parameter_updates
/p3dx/joint_cmds
/p3dx/joint_measures
/p3dx/joint_states
/p3dx/joy
/p3dx/laser_front/scan
/p3dx/laser_front/scan/raw
/p3dx/motors_state
/p3dx/parameter_descriptions
/p3dx/parameter_updates
/p3dx/particlecloud
/p3dx/pose
/p3dx/sonar
/p3dx/sonar_pointcloud2
/rosout
/rosout_agg
/tf
/tf_static
