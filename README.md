## Experimental SLAM and Nav2 Navigation

In addition to reactive LiDAR obstacle avoidance, the project has been extended with an experimental mapping, localization, and navigation workflow using ROS 2 Humble, Gazebo Fortress, SLAM Toolbox, and Nav2.

**Status: Navigation is still under development.** A global path was observed in RViz after sending a 2D Nav Goal, but reliable end-to-end navigation has not yet been verified.

### Implemented and Tested Components

- Simulated LiDAR data bridged to ROS 2 on `/lidar`.
- Simulated odometry available on `/model/my_robot/odometry`.
- A saved occupancy-grid map at `maps/obstacle_world.pgm`, with metadata in `maps/obstacle_world.yaml`.
- SLAM Toolbox used to generate a map during mapping experiments.
- Nav2 Map Server configured to load the saved map.
- AMCL configured for localization using the saved map and LiDAR data.
- Nav2 planner and controller servers launched with a custom parameter file.
- RViz used to visualize the map and set navigation goals.
- A global path observed after sending a 2D Nav Goal.

These results demonstrate progress toward navigation, but they do not yet establish reliable autonomous goal execution.

### Navigation Architecture

The intended navigation pipeline is:

```text
Saved Occupancy Grid Map
          |
          v
      Map Server
          |
          v
         /map
          |
          v
   AMCL Localization <----- LiDAR (/lidar)
          |
          v
       TF: map
          |
          v
   Planner Server
          |
          v
      Global Plan
          |
          v
  Controller Server
          |
          v
     /cmd_vel_nav
          |
          v
   Velocity Smoother
          |
          v
       /cmd_vel
          |
          v
    Gazebo Command Bridge
          |
          v
     Robot Motion
```

The robot's odometry is provided through `/model/my_robot/odometry`, using the frames `my_robot/odom` and `my_robot/chassis`.

### Coordinate Frames

The following frame configuration was used during the experiment:

| Frame | Purpose |
|---|---|
| `map` | Global map reference frame |
| `my_robot/odom` | Local odometry frame |
| `my_robot/chassis` | Robot base frame |
| `my_robot/chassis/lidar` | LiDAR sensor frame |

The expected transform chain is:

```text
map
 |
 v
my_robot/odom
 |
 v
my_robot/chassis
 |
 v
my_robot/chassis/lidar
```

AMCL is responsible for publishing the `map` to odometry transform, while the simulation provides the odometry transform and the static LiDAR transform.

### Configuration Fixes

The following issues were encountered and corrected during Nav2 setup:

**1. Planner plugin identifier**

The initial planner configuration used:

```yaml
plugin: "nav2_navfn_planner::NavfnPlanner"
```

This caused a plugin-loading error in the tested ROS 2 Humble installation. The configuration that loaded successfully was:

```yaml
plugin: "nav2_navfn_planner/NavfnPlanner"
```

**2. Costmap parameter types**

The controller server initially crashed because the costmap `height` parameter was provided as an integer while the node expected a floating-point value.

Use floating-point values for dimensions when required by the declared parameter types:

```yaml
width: 4.0
height: 4.0
resolution: 0.05
```

**3. Simulation time**

The simulation publishes `/clock`. Nav2 nodes must use simulation time when running against this clock:

```yaml
use_sim_time: true
```

**4. Robot frame names**

The robot does not use the default `base_link` frame in this experiment. The configured base frame is:

```yaml
robot_base_frame: my_robot/chassis
```

The odometry frame is:

```yaml
odom_frame_id: my_robot/odom
```

Keep frame names consistent across AMCL, the costmaps, the behavior server, the controller, and RViz.

### Experimental Map Files

The saved map consists of two files:

```text
maps/
├── obstacle_world.pgm
└── obstacle_world.yaml
```

The YAML file references the image and defines map resolution, origin, and occupancy thresholds.

Example metadata:

```yaml
image: obstacle_world.pgm
mode: trinary
resolution: 0.05
origin: [-1.95, -2.37, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

The saved map was successfully loaded through Nav2 Map Server and was visible in RViz.

### Nav2 Launch

The navigation servers were started using the custom parameter file:

```bash
source /opt/ros/humble/setup.bash
source ~/robotics_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/ayush/robotics_ws/config/nav2_params.yaml
```

This command assumes that the simulation, ROS-Gazebo bridges, map server, AMCL, and RViz have been started separately.

The parameter file must exist at the specified path. Update it if the workspace is located elsewhere.

### Useful Navigation Diagnostics

Check the navigation lifecycle states:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

Check the navigation action and global plan:

```bash
ros2 action info /navigate_to_pose
ros2 topic echo /plan --once
```

Check odometry and localization:

```bash
ros2 topic hz /model/my_robot/odometry
ros2 topic echo /amcl_pose --once
```

Inspect the velocity command pipeline while a goal is actively executing:

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

Check the transforms:

```bash
ros2 run tf2_ros tf2_echo map my_robot/odom
ros2 run tf2_ros tf2_echo my_robot/odom my_robot/chassis
```

Run commands in separate terminals when necessary. Compare the controller output, velocity smoother output, Gazebo command bridge, and odometry during the same navigation attempt.

### Known Issues

The Nav2 experiment still requires further debugging:

- A global path can appear in RViz without the robot reliably following it.
- `/cmd_vel_nav` has produced brief angular velocity commands followed by zero commands.
- `/cmd_vel` has shown zero commands during some observations.
- Intermittent TF timestamp and connectivity warnings were observed.
- Costmap warnings indicated that the robot or sensor origin was outside the costmap bounds during startup.
- The ROS command bridge is subscribed to `/cmd_vel`, but command delivery to the Gazebo drive system during an active Nav2 goal has not been conclusively verified.

These observations have not yet established a single root cause.

### Next Steps

1. Verify the TF tree during a fresh simulation run.
2. Verify that the map, global costmap, and local costmap are correctly aligned.
3. Trace a non-zero velocity command from the Nav2 controller through the velocity smoother and Gazebo bridge.
4. Confirm that odometry changes while a navigation goal is being executed.
5. Test navigation to multiple reachable goals in the simulated world.
6. Document a successful end-to-end navigation run once it is repeatable.