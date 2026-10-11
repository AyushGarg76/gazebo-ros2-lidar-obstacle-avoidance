# Gazebo Fortress + ROS 2 LiDAR Obstacle Avoidance

A differential-drive robot simulation built with **Gazebo Fortress** and **ROS 2 Humble**.

The robot uses a simulated 360° LiDAR sensor and a ROS 2 C++ controller to detect nearby obstacles, stop forward motion, and turn toward the side with more open space.

> This project demonstrates reactive obstacle avoidance. It does not currently implement SLAM or global path planning.

## Features

- Custom differential-drive robot and obstacle arena defined in SDF.
- Simulated 360° GPU LiDAR publishing on `/lidar`.
- ROS 2 C++ controller using `sensor_msgs/msg/LaserScan`.
- Velocity commands published through `geometry_msgs/msg/Twist`.
- Forward obstacle detection within ±30°.
- Stop/resume hysteresis using 1.0 m and 1.2 m thresholds.
- Reactive turning toward the side with the greater average measured range.
- Watchdog that publishes zero velocity if LiDAR data has not arrived or becomes stale.
- Launch file that starts Gazebo, ROS-Gazebo bridges, and the controller.

## Architecture

```text
Gazebo Fortress
(lidar_world.sdf)
       |
       | LiDAR scan
       v
  ros_gz_bridge
       |
       | /lidar
       v
lidar_avoidance_node
     (ROS 2 C++)
       |
       | /cmd_vel
       v
  ros_gz_bridge
       |
       v
Gazebo differential-drive system
       |
       v
    Robot motion
```

## Technology Stack

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress
- C++17
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `ros_gz_sim`
- `ros_gz_bridge`
- SDF

## Repository Structure

```text
robotics_ws/
├── src/
│   ├── lidar_world.sdf
│   ├── lidar_avoidance_ros/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch/
│   │   │   └── lidar_avoidance.launch.py
│   │   └── src/
│   │       └── lidar_avoidance_node.cpp
│   └── lidar_controller/
│       ├── CMakeLists.txt
│       └── lidar_node.cpp
└── .gitignore
```

The `lidar_avoidance_ros` package contains the ROS 2 controller and launch file used in the main workflow.

The `lidar_controller` directory contains a separate standalone Gazebo Transport C++ controller example.

## How It Works

1. The simulated LiDAR publishes scans on `/lidar`.
2. `ros_gz_bridge` forwards the scans to ROS 2.
3. The C++ node examines the forward sector between -30° and +30°.
4. If the minimum valid range falls below 1.0 m, the controller enters obstacle-avoidance mode.
5. It compares average measured ranges in the left and right sectors.
6. The robot stops moving forward and turns toward the side with the greater average range.
7. Forward movement resumes when the minimum forward range exceeds 1.2 m.
8. If scans are unavailable or become stale, the watchdog publishes a zero-velocity command.

### Controller Parameters

| Parameter | Value | Purpose |
|---|---:|---|
| Stop distance | 1.0 m | Enter obstacle-avoidance mode |
| Resume distance | 1.2 m | Resume forward movement |
| Forward velocity command | 0.4 m/s | Move forward when the path is clear |
| Angular velocity command | 0.5 rad/s | Turn while avoiding obstacles |
| Scan timeout | 1.0 s | Detect stale LiDAR data |
| Watchdog interval | 500 ms | Check scan availability |

The separate stop and resume thresholds provide hysteresis, reducing rapid switching when an obstacle is near the stopping threshold.

These are commanded velocities and distances; actual simulated motion depends on the robot model and physics.

## Prerequisites

The project was developed for:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress
- `ros_gz_sim`
- `ros_gz_bridge`
- `colcon` and the ROS 2 C++ build toolchain

Source ROS 2 in your terminal:

```bash
source /opt/ros/humble/setup.bash
```

Make sure Gazebo Fortress and the ROS-Gazebo integration packages are installed and working before launching the simulation.

## Build and Run

### 1. Clone the Repository

The repository contains the project files under `src/`. To recreate the workspace layout expected by the current launch file:

```bash
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws/src

git clone https://github.com/AyushGarg76/gazebo-ros2-lidar-obstacle-avoidance.git project_checkout
```

Move the cloned project files into the workspace's `src/` directory, or use an equivalent directory layout that places the ROS package at:

```text
~/robotics_ws/src/lidar_avoidance_ros
```

The world file should be located at:

```text
~/robotics_ws/src/lidar_world.sdf
```

**Note:** The current launch file uses an absolute path to the world file. If your workspace is in a different location, update that path in the launch file.

### 2. Build the ROS 2 Package

From the workspace root:

```bash
cd ~/robotics_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select lidar_avoidance_ros

source install/setup.bash
```

### 3. Launch the Simulation

```bash
ros2 launch lidar_avoidance_ros lidar_avoidance.launch.py
```

The launch file starts:

- Gazebo Fortress with `lidar_world.sdf`
- The LiDAR bridge from Gazebo to ROS 2
- The velocity-command bridge from ROS 2 to Gazebo
- The `lidar_avoidance_node` C++ controller

## Useful ROS 2 Commands

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/robotics_ws/install/setup.bash
```

List active topics:

```bash
ros2 topic list
```

Inspect LiDAR scans:

```bash
ros2 topic echo /lidar
```

Inspect velocity commands:

```bash
ros2 topic echo /cmd_vel
```

List running nodes:

```bash
ros2 node list
```

## Limitations and Future Improvements

- The controller is reactive and does not build a map or plan a global route.
- Average range is a simple heuristic and may not always identify the safest path.
- The watchdog handles missing or stale scan data but does not make this a safety-certified controller.
- The world-file path in the launch file should be made configurable.
- The current implementation has not been presented as a physical-robot deployment.

Potential next steps:

- Make the launch file portable by removing the hard-coded world path.
- Add odometry and TF visualization.
- Visualize LiDAR scans in RViz.
- Integrate SLAM Toolbox for mapping.
- Explore Nav2 for localization and navigation.

## License

The ROS 2 package declares Apache-2.0 in its `package.xml`. Add a repository-level `LICENSE` file if you intend to distribute the full repository under that license.
