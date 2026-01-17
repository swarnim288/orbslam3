# orbslam3

implementing orbslam 
connect realsesne to usb3.0 and check 
lsusb | grep Intel
source ~/ros2_ws/install/setup.bash
ros2 run realsense2_camera realsense2_camera_node --ros-args   -p enable_infra1:=true   -p enable_infra2:=true   -p enable_color:=false   -p enable_depth:=false   -p depth_module.emitter_enabled:=0   -p depth_module.profile:=848x480x30
ros2 topic list | grep infra
You must see:

/infra1/image_rect_raw
/infra2/image_rect_raw
ros2 topic echo /infra1/image_rect_raw --once
cd ~/ORB_SLAM3
ros2 run orbslam3 stereo \
  /home/robo/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/robo/orbslam3_config/D435i_STEREO.yaml \
  false
EKF PIPELINE
SAVING ORBSLAM3 POSE AS ONE AND IMU TOPICS AS OTHER 
