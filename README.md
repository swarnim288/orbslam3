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

# ORB-SLAM3 with Intel RealSense D435i (Stereo Infra) on ROS 2

This guide explains how to run **ORB-SLAM3 in stereo mode** using the **infrared stereo cameras** of the **Intel RealSense D435i**, connected via **USB 3.0**, using **ROS 2**.

The setup disables depth and RGB streams and uses only **infra1 + infra2**, which is the recommended configuration for stereo ORB-SLAM.

---

## 1. Hardware & Prerequisites

### Hardware

* Intel RealSense **D435i**
* USB **3.0** port (mandatory for stable stereo streaming)

### Software

* Ubuntu (20.04 / 22.04 recommended)
* ROS 2 (Humble / Iron)
* `realsense2_camera` ROS 2 package
* ORB-SLAM3 (built with ROS 2 support)

---

## 2. Verify RealSense USB Connection

Ensure the camera is detected and running on USB 3.0:

```bash
lsusb | grep Intel
```

You should see an Intel RealSense device listed.

⚠️ If the camera shows up as USB 2.0, ORB-SLAM will likely fail due to bandwidth limitations.

---

## 3. Source ROS 2 Workspace

Source your ROS 2 workspace where `realsense2_camera` is installed:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## 4. Launch RealSense Stereo Infra Streams

Run the RealSense camera node with **only infrared streams enabled**:

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_infra1:=true \
  -p enable_infra2:=true \
  -p enable_color:=false \
  -p enable_depth:=false \
  -p depth_module.emitter_enabled:=0 \
  -p depth_module.profile:=848x480x30
```

### Parameter Explanation

* `enable_infra1 / enable_infra2`: Enables left & right stereo cameras
* `enable_color:=false`: Disables RGB stream
* `enable_depth:=false`: Disables depth computation
* `emitter_enabled:=0`: Turns off IR projector (important for feature tracking)
* `848x480x30`: Stable resolution for ORB-SLAM3 stereo mode

---

## 5. Verify Infra Topics

Check that the infrared image topics are being published:

```bash
ros2 topic list | grep infra
```

You **must** see:

```
/infra1/image_rect_raw
/infra2/image_rect_raw
```

Test a single message to confirm valid data:

```bash
ros2 topic echo /infra1/image_rect_raw --once
```

If this works, the camera pipeline is correct.

---

## 6. Run ORB-SLAM3 (Stereo Mode)

Navigate to the ORB-SLAM3 directory:

```bash
cd ~/ORB_SLAM3
```

Run ORB-SLAM3 with stereo configuration:

```bash
ros2 run orbslam3 stereo \
  /home/robo/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/robo/orbslam3_config/D435i_STEREO.yaml
```

### Required Files

* **ORB Vocabulary**

  ```
  ORB_SLAM3/Vocabulary/ORBvoc.txt
  ```
* **Camera Configuration**

  ```
  orbslam3_config/D435i_STEREO.yaml
  ```

Make sure the YAML file:

* Uses correct **camera intrinsics**
* Has correct **baseline**
* Matches the IR resolution (`848x480`)
* Uses correct **ROS topic names**

---

## 7. Expected Output

If everything is working correctly:

* ORB-SLAM3 window opens
* Stereo features are detected
* Map initialization completes after camera motion
* Tracking status changes to **OK**

---




---


