
# ORB-SLAM3 with Intel RealSense D435i (Stereo Infra) on ROS 2

This guide explains how to run **ORB-SLAM3 in stereo mode** using the **infrared stereo cameras** of the **Intel RealSense D435i**, connected via **USB 3.0**, using **ROS 2**.

The setup disables depth and RGB streams and uses only **infra1 + infra2**, which is the recommended configuration for stereo ORB-SLAM.



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
If the camera shows up as USB 2.0, ORB-SLAM will likely fail due to bandwidth limitations.

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
 this works, the camera pipeline is correct.


## 0. Build ORB-SLAM3 (ROS 2)

This section explains how to build **ORB-SLAM3 with ROS 2 support** before running the stereo pipeline.

---

### 0.1 Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  libeigen3-dev \
  libboost-all-dev \
  libgl1-mesa-dev \
  libglew-dev \
  libyaml-cpp-dev
```

---

### 0.2 Install Pangolin

ORB-SLAM3 requires **Pangolin** for visualization.

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

### 0.3 Clone ORB-SLAM3

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
```

---

### 0.4 Build ORB-SLAM3 Core

```bash
chmod +x build.sh
./build.sh
```

This builds:

* ORB-SLAM3 core libraries
* Vocabulary tools
* Example binaries

Make sure **no errors** occur here before continuing.

---

### 0.5 Build ROS 2 Wrapper

ORB-SLAM3 includes a ROS 2 interface in the `Examples/ROS2` folder.

```bash
cd ~/ORB_SLAM3/Examples/ROS2
colcon build --symlink-install
```

After a successful build, source the workspace:

```bash
source install/setup.bash

### 0.6 Verify ORB-SLAM3 ROS 2 Nodes

```bash
ros2 pkg list | grep orbslam3
```

You should see:

```
orbslam3
```

---

## Build Order Summary

```text
1. Install dependencies
2. Build Pangolin
3. Build ORB-SLAM3 core
4. Build ROS 2 wrapper
5. Source workspace
```

---

## Common Build Issues

### Pangolin not found

```text
Pangolin_DIR not found
```

Fix:

```bash
sudo ldconfig
```

---

### OpenCV version mismatch

* Use system OpenCV (`libopencv-dev`)
* Avoid mixing conda OpenCV

---

### Jetson users

* Build with fewer cores:

```bash
make -j2
```

* Ensure swap is enabled


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

# Fake IMU + EKF Fusion + Motion-Compensated Point Cloud (ROS 2)

## Node Graph

```
/slam_pose            → visual SLAM pose (30 FPS)
/fake_imu             → synthetic IMU from SLAM pose
/ekf_fusion           → EKF predict + update
/pointcloud_mapper    → motion-compensated point cloud
```

---

## 1. Fake IMU Node

**Node**

```
fake_imu
```

**Subscribe**

```
/slam/pose   (geometry_msgs/PoseStamped)
```

**Publish**

```
/imu/data_raw   (sensor_msgs/Imu)
```

**Logic**

```cpp
onPose(pose_k, t_k)
{
    buffer.push(pose_k)

    if buffer has pose_{k-1}, pose_k, pose_{k+1}:

        dt = t_k - t_{k-1}

        // Gyroscope
        q_k    = pose_k.orientation
        q_prev = pose_{k-1}.orientation

        dq = q_k * inverse(q_prev)
        omega = (2.0 / dt) * logQuaternion(dq)

        // Accelerometer
        p_prev = pose_{k-1}.position
        p_k    = pose_k.position
        p_next = pose_{k+1}.position

        a_world = (p_next - 2*p_k + p_prev) / (dt*dt)
        R = rotationMatrix(q_k)
        a_imu = transpose(R) * (a_world - gravity)

        publish Imu:
            angular_velocity = omega
            linear_acceleration = a_imu
            header.stamp = t_k
}
```

---

## 2. EKF Fusion Node

**Node**

```
ekf_fusion
```

**Subscribe**

```
/imu/data_raw   (sensor_msgs/Imu)
/slam/pose      (geometry_msgs/PoseStamped)
```

**Publish**

```
/fused/odometry (nav_msgs/Odometry)
/tf             (world → imu_link)
```

**State**

```cpp
x = {
    position p (3),
    velocity v (3),
    orientation q (4),
    accel_bias ba (3),
    gyro_bias bg (3)
}

P = 15x15 covariance
```

---

### EKF Prediction (IMU)

```cpp
onImu(imu, dt)
{
    omega = imu.angular_velocity - bg
    accel = imu.linear_acceleration - ba

    q = q * Exp(omega * dt)

    v = v + (R(q) * accel + gravity) * dt
    p = p + v * dt + 0.5 * (R(q) * accel + gravity) * dt * dt

    P = F * P * F.transpose() + Q
}
```

---

### EKF Update (SLAM Pose)

```cpp
onSlamPose(pose)
{
    z_p = pose.position
    z_q = pose.orientation

    r_p = z_p - p
    r_q = quaternionError(z_q, q)

    r = [r_p, r_q]

    K = P * H.transpose() * inverse(H * P * H.transpose() + R)

    x = x + K * r
    P = (I - K * H) * P

    normalize(q)

    publish fused odometry
}
```

---

## 3. Motion-Compensated Point Cloud Node

**Node**

```
pointcloud_mapper
```

**Subscribe**

```
/camera/depth/points  (sensor_msgs/PointCloud2)
/fused/odometry       (nav_msgs/Odometry)
```

**Publish**

```
/map/points           (sensor_msgs/PointCloud2)
```

**Logic**

```cpp
onPointCloud(cloud)
{
    pose = latest_fused_pose()

    for point p_cam in cloud:
        p_world = pose.R * p_cam + pose.t
        global_cloud.add(p_world)

    voxelFilter(global_cloud)
    publish global_cloud
}
```

---

## 4. TF Frames

```
world
 └── imu_link
      └── camera_link
```

---

## 5. Execution Order

```
1. SLAM node
2. fake_imu
3. ekf_fusion
4. pointcloud_mapper
```

---

## Notes

* SLAM runs at 30 FPS
* Synthetic IMU uses SLAM timestamps
* EKF runs predict + update at same rate
* Intended for validation and research only






---


