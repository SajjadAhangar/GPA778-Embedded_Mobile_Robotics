# Jerro Navigation (`jerro_navigation`)

A ROS 2 package designed for differential drive mobile robot control, kinematic translation, and wheel encoder feedback processing on the Jerro platform.

## Architecture & Node Details

The core node (`twist`) bridges high-level velocity commands with low-level hardware interfaces:

### Subscribed Topics
- **`/cmd_vel`** (`geometry_msgs/msg/Twist`): Incoming linear and angular velocity commands (typically driven by teleoperation).
- **`/encoder_a`** (`std_msgs/msg/Int32`): Raw encoder ticks from the left wheel.
- **`/encoder_b`** (`std_msgs/msg/Int32`): Raw encoder ticks from the right wheel.

### Published Topics
- **`/motor/set_speed`** (`jerro_msgs/msg/MotorSpeed`): Computed left and right wheel tick speeds translated via differential drive kinematics using robot geometry parameters ($L = 0.175\,\text{m}$, $r = 0.035\,\text{m}$).

---

## Prerequisites & Dependencies

This package relies on official hardware driver and message packages provided by Lab-CORO:

**Jerro Repositories (`jerro` & `jerro_msgs`)**:
   Clone the necessary driver and message packages into your workspace's `src` directory:
   ```bash
   git clone [https://github.com/Lab-CORO/jerro](https://github.com/Lab-CORO/jerro)
   ```
   
**Execution Workflow**

To operate the robot live, you must run three separate terminals concurrently. Open three terminal windows or tabs and execute the following steps in order:

Terminal 1: Launch Hardware Drivers **ros2 launch jerro_drivers motor_control.launch.py**

Terminal 2: Run twist node **ros2 run jerro_navigation twist**

Terminal 3: Run Keyboard Teleoperation **ros2 run teleop_twist_keyboard teleop_twist_keyboard**

Have fun.
