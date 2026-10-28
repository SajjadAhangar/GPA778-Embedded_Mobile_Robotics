# Jerro Perception (`jerro_perception`)

A ROS 2 perception package designed for real-time mobile robot sensor processing, featuring OpenCV-based camera edge detection and 2D-to-3D LiDAR point cloud transformation on the Jerro platform.

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

Terminal 2: Inspect active topics and data publishing rates: **ros2 topic list**, **ros2 topic hz /image_canny**, and **ros2 topic hz /points**

- Connect via RViz to add an Image display panel for `/image_canny` and a `PointCloud2` display panel for `/points`.

Have fun.
