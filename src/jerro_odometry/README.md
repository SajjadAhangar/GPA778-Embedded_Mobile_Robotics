# Jerro Odometry (`jerro_odometrie`)

A ROS 2 package designed for real-time dead-reckoning odometry calculation and Coordinate Frame (TF) broadcasting for the Jerro differential drive mobile robot platform.

## Architecture & Node Details

The core node (`odometry`) computes the robot's pose in continuous 2D space from incremental wheel encoder ticks and broadcasts the live spatial transformation between the global world reference frame (`odom`) and the robot chassis (`base_link`).

### Subscribed Topics
- **`/encoder_a`** (`std_msgs/msg/Int32`): Raw encoder tick count from the left wheel.
- **`/encoder_b`** (`std_msgs/msg/Int32`): Raw encoder tick count from the right wheel.

### Broadcasted Transforms
- **`odom`** $\rightarrow$ **`base_link`** (`geometry_msgs/msg/TransformStamped`): Dynamic transform calculated via differential drive kinematics using robot geometry parameters ($L = 0.1655\,\text{m}$, $r = 0.035\,\text{m}$, `pulses_per_revolution` $= 1943$).

---

## Prerequisites & Dependencies

This package relies on official hardware driver and message packages provided by Lab-CORO, as well as core ROS 2 transform libraries (`tf2_ros`, `rclpy`, `scipy`, `numpy`):

**Jerro Repositories (`jerro`)**:
   Clone the necessary driver and message packages into your workspace's `src` directory:
   ```bash
   git clone [https://github.com/Lab-CORO/jerro.git](https://github.com/Lab-CORO/jerro.git)
   ```
   
**Execution Workflow & Test Procedure**
To verify that your localization `/odometry` node is working correctly, place the robot on the floor, aligned with a straight line, in order to verify that the calculated distance corresponds to the actual distance traveled.
To operate the robot live, you must run five separate terminals concurrently. Open five terminal windows or tabs and execute the following steps in order (the result can be observed in RViz2 on your laptop terminal by adding a TF display):
- Terminal 1: Motor Control System
    ```bash
    cd ~/ros2_ws
    
    colcon build
    
    source install/setup.bash
    
    ros2 launch jerro_drivers motor_control.launch.py
    ```

- Terminal 2: Odometry Node
    ```bash
    cd ~/ros2_ws && source install/setup.bash

    ros2 run jerro_odometrie odometrie
    ```

- Terminal 3: Navigation Package Node
    ```bash
    cd ~/ros2_ws && source install/setup.bash

    ros2 run jerro_navigation twist
    ```

- Terminal 4: Teleoperation
    ```bash
    cd ~/ros2_ws && source install/setup.bash

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    
- Terminal 5: TF Description
    ```bash
    cd ~/ros2_ws && source install/setup.bash

    ros2 launch jerro_description jerro_description.launch.py
    ```

Control the robot using the `teleop_twist_keyboard` user interface.

Have fun.
