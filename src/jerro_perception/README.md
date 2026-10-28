# Jerro Perception (`jerro_perception`)

A ROS 2 perception package designed for real-time mobile robot sensor processing, featuring OpenCV-based camera edge detection, 2D LiDAR processing, and **3D volumetric point cloud scanning via a servo-actuated LiDAR mount and Action Server coordination**.


## Architecture & Node Details

The package integrates high-frequency sensor streams using optimized ROS2 QoS profiles and robust exception handling:

### 1. Camera Processor Node (`camera_processor`)
Subscribes to raw camera imagery and extracts spatial features for visual navigation.
- **Subscribed Topics**:
  - **`/image_raw`** (`sensor_msgs/msg/Image`): Raw RGB camera frames (leveraging `sensor_data` Best-Effort QoS).
- **Published Topics**:
  - **`/image_canny`** (`sensor_msgs/msg/Image`): Single-channel (`mono8`) edge-detected image output processed via OpenCV Canny filters.
- **Parameters**:
  - `canny_lower_threshold` (default: `50`): Lower hysteresis threshold for edge linking.
  - `canny_upper_threshold` (default: `150`): Upper hysteresis threshold for edge detection.

### 2. LiDAR Processor Node (`lidar_processor`)
Processes 2D laser scans from the Slamtec S2 LiDAR and translates them into Cartesian point clouds.
- **Subscribed Topics**:
  - **`/scan`** (`sensor_msgs/msg/LaserScan`): Polar coordinate distance measurements (leveraging `sensor_data` Best-Effort QoS).
- **Published Topics**:
  - **`/points`** (`sensor_msgs/msg/PointCloud2`): Transformed 3D point cloud data ($x, y, z$ Cartesian coordinates) ready for RViz visualization and spatial mapping.

### 3. LiDAR Processor Node (`lidar_processor_node`)
Processes 2D laser scans and coordinates a servo sweep to assemble full 3D point clouds.
- **Subscribed Topics**:
  - **`/scan`** (`sensor_msgs/msg/LaserScan`): Polar coordinate distance measurements (using `sensor_data` Best-Effort QoS).
- **Published Topics**:
  - **`/points`** (`sensor_msgs/msg/PointCloud2`): Real-time 2D-to-3D Cartesian point cloud.
  - **`perception/lidar_pc2`** (`sensor_msgs/msg/PointCloud2`): Assembled 3D volumetric point cloud published upon completion of an action scan.
- **Action Server**:
  - **`/perception/start_scan`** (`jerro_msgs/action/StartScan`): Coordinates the servo sweep sequence, manages real-time feedback progress, and triggers point cloud assembly.
- **Service Client**:
  - **`/set_servo_pos`** (`jerro_msgs/srv/SetServoPos`): Commands angular positioning of the LiDAR mounting servo.
 
 
---

## Prerequisites & Dependencies

This package depends on hardware drivers and utility packages provided by Lab-CORO and Slamtec:

- **SLLidar ROS 2 Driver (`sllidar_ros2`)**:
   Clone and build the official S2 LiDAR driver in your workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone [https://github.com/Slamtec/sllidar_ros2.git](https://github.com/Slamtec/sllidar_ros2.git)
   ```
---

**Execution Workflow**

To launch the complete perception pipeline (camera stream, Canny edge detection, LiDAR processing, and S2 driver) with a single command:

Terminal 1: Run the Perception Launch File **ros2 launch jerro_perception perception_launch.py**
   
Terminal 2: Launch jerro-description **ros2 launch jerro_description jerro_description.launch.py**

Terminal 3: Run the jerro_driver to move the servo: **ros2 run jerro_drivers servomotor*

Terminal 4: Trigger a 3D Volumetric Scan by by sending a goal **ros2 action send_goal /perception/start_scan jerro_msgs/action/StartScan "{start: true}" --feedback**

- RViz Visualization:

Connect via RViz2, set the Fixed Frame to `base_link`, add an Image display panel for `/image_canny`, and add a `PointCloud2` display panel for both `/points` and `perception/lidar_pc2`.

Have fun.



