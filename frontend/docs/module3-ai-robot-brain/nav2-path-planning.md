---
sidebar_position: 4
title: Nav2 (Path Planning for Bipedal Humanoid Movement)
---

# Nav2 (Path Planning for Bipedal Humanoid Movement)

## Introduction

**Nav2** (Navigation2) is the next-generation navigation framework for ROS 2. While originally designed for wheeled robots, Nav2 can be adapted for humanoid robots with careful consideration of bipedal locomotion constraints.

**Key Differences for Humanoids:**
- **Non-holonomic constraints**: Cannot move sideways without turning
- **Step planning**: Discrete foot placements vs continuous wheel motion
- **Balance considerations**: Center of mass must stay within support polygon
- **Slower speeds**: Typical humanoid walk: 0.5-1.5 m/s vs wheeled robots: 1-3 m/s
- **Higher turning costs**: Rotating requires coordinated leg movements

This chapter covers how to configure Nav2 for humanoid robots and integrate gait planning with path planning.

## Nav2 Architecture

### Core Components

```
Goal → Planner Server → Path → Controller Server → Cmd_Vel
         ↓                        ↓
    Global Costmap          Local Costmap
         ↓                        ↓
    Static Map +            Sensor Data
    Inflation               (LiDAR, Camera)
```

**Nav2 Behavior Tree:**
```
NavigateToPose
├── ComputePathToPose (global planner)
├── FollowPath (controller)
├── ClearCostmap (recovery)
└── Spin (recovery)
```

### Installation

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3-*  # Useful examples
```

## Configuring Nav2 for Humanoid Robots

### Robot Footprint

Unlike circular robots, humanoids have a rectangular footprint that changes during walking.

```yaml
# config/humanoid_footprint.yaml
robot_base_frame: "base_link"

# Conservative footprint (shoulders width x forward reach)
footprint: "[[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]"

# Alternative: circular approximation (radius in meters)
# robot_radius: 0.35
```

### Controller Parameters

Humanoids need slower, smoother commands than wheeled robots.

```yaml
# config/controller.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Hz (higher for stability)
    min_x_velocity_threshold: 0.01  # m/s
    min_y_velocity_threshold: 0.0   # No lateral movement
    min_theta_velocity_threshold: 0.01  # rad/s

    # DWB Controller (Dynamic Window Approach - Bipedal)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Velocity limits (conservative for humanoids)
      min_vel_x: 0.0
      max_vel_x: 0.8  # Slow walk
      min_vel_y: 0.0  # No strafing
      max_vel_y: 0.0
      max_vel_theta: 0.5  # 30 deg/s turning

      # Acceleration limits (smooth starts/stops)
      acc_lim_x: 0.3  # m/s²
      acc_lim_y: 0.0
      acc_lim_theta: 0.5  # rad/s²

      # Deceleration limits (prevent falling forward)
      decel_lim_x: -0.5  # Stronger braking
      decel_lim_theta: -1.0

      # Trajectory generation
      vx_samples: 10  # Test 10 forward velocities
      vy_samples: 1   # No lateral motion
      vtheta_samples: 20  # Test 20 rotational velocities

      # Simulation time (predict N seconds ahead)
      sim_time: 2.0  # Longer for slow humanoids

      # Critics (cost functions)
      critics: [
        "RotateToGoal",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist"
      ]

      # Humanoid-specific tuning
      BaseObstacle.scale: 0.02  # Obstacle avoidance weight
      PathAlign.scale: 32.0     # Stay on path (important for stability)
      PathDist.scale: 32.0      # Progress along path
      GoalAlign.scale: 24.0     # Final orientation
      GoalDist.scale: 24.0      # Reach goal

      # Path following tolerance
      xy_goal_tolerance: 0.1  # 10cm
      yaw_goal_tolerance: 0.15  # ~9 degrees
```

### Costmap Configuration

```yaml
# config/costmap_common.yaml
costmap_common:
  ros__parameters:
    footprint: "[[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]"
    robot_radius: 0.35  # Backup if footprint fails

    # Obstacle layer (from sensors)
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "scan"

      scan:
        topic: /scan
        sensor_frame: lidar_link
        observation_persistence: 0.0  # Seconds to keep old data
        expected_update_rate: 5.0  # Hz
        data_type: "LaserScan"
        min_obstacle_height: 0.0  # Ground level
        max_obstacle_height: 2.0  # Ceiling height
        inf_is_valid: false
        marking: true  # Add obstacles
        clearing: true  # Remove obstacles when not seen
        raytrace_range: 10.0  # Clear out to 10m
        obstacle_range: 8.0   # Detect obstacles to 8m

    # Inflation layer (safety margin)
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.55  # Robot radius + safety margin
      cost_scaling_factor: 3.0  # How fast cost increases

    # Static map layer
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true
      enabled: true

    always_send_full_costmap: true
```

### Global Costmap (Long-range planning)

```yaml
# config/global_costmap.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0  # Hz (slow updates for efficiency)
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link

      resolution: 0.05  # 5cm grid cells
      width: 100  # 100 cells = 5m
      height: 100
      origin_x: -2.5
      origin_y: -2.5

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### Local Costmap (Short-range obstacle avoidance)

```yaml
# config/local_costmap.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0  # Hz (faster for dynamic obstacles)
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link

      resolution: 0.025  # 2.5cm grid (finer than global)
      width: 5  # 5 cells = 12.5cm
      height: 5
      origin_x: -1.25  # Center robot
      origin_y: -1.25

      rolling_window: true  # Move with robot
      plugins: ["obstacle_layer", "inflation_layer"]
```

## Path Planners for Humanoids

### NavFn vs Smac Planner

| Planner | Algorithm | Best For | Humanoid Suitability |
|---------|-----------|----------|---------------------|
| **NavFn** | Dijkstra / A* | Simple environments | ✓ Good (fast, smooth) |
| **SmacPlanner2D** | Hybrid A* | Complex environments | ✓✓ Better (considers orientation) |
| **SmacPlannerHybrid** | State Lattice | Parking-like | ✗ Overkill (car-like motion) |

**Recommended: SmacPlanner2D** for humanoids.

```yaml
# config/planner_server.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Hz
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"

      # Grid search parameters
      tolerance: 0.25  # Goal tolerance (meters)
      downsample_costmap: false  # Use full resolution
      downsampling_factor: 1

      # A* tuning
      allow_unknown: true  # Plan through unexplored areas
      max_iterations: 1000000
      max_planning_time: 5.0  # seconds

      # Smoothing (critical for humanoids)
      smoother:
        max_iterations: 1000
        w_smooth: 0.3  # Smoothness weight
        w_data: 0.2    # Fidelity to original path
        tolerance: 1e-10
```

## Gait Integration: From Path to Footsteps

Nav2 produces smooth paths, but humanoids walk in discrete steps. We need a **footstep planner**.

### Footstep Planner Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Subscribe to Nav2 path
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        # Publish footstep sequence
        self.footstep_pub = self.create_publisher(
            Path,  # Each pose is a foot placement
            '/footsteps',
            10
        )

        # Humanoid parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # lateral foot separation
        self.step_height = 0.05  # foot clearance

    def path_callback(self, path_msg):
        """Convert continuous path to discrete footsteps"""

        if len(path_msg.poses) < 2:
            return

        footsteps = []
        current_foot = 'left'  # Start with left foot

        # Sample path at step_length intervals
        for i in range(0, len(path_msg.poses), 1):
            pose = path_msg.poses[i].pose

            # Alternate feet
            if current_foot == 'left':
                lateral_offset = self.step_width / 2
                current_foot = 'right'
            else:
                lateral_offset = -self.step_width / 2
                current_foot = 'left'

            # Create footstep pose
            footstep = PoseStamped()
            footstep.header = path_msg.header
            footstep.pose.position.x = pose.position.x
            footstep.pose.position.y = pose.position.y + lateral_offset
            footstep.pose.position.z = self.step_height
            footstep.pose.orientation = pose.orientation

            footsteps.append(footstep)

        # Publish footstep plan
        footstep_path = Path()
        footstep_path.header = path_msg.header
        footstep_path.poses = footsteps

        self.footstep_pub.publish(footstep_path)

        self.get_logger().info(f'Generated {len(footsteps)} footsteps')

def main():
    rclpy.init()
    node = FootstepPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Behavior Trees for Complex Navigation

Nav2 uses behavior trees to handle failures gracefully.

### Custom Humanoid Behaviors

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')

        # Action client for Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

    def navigate_to_goal(self, x, y, theta):
        """Send navigation goal with humanoid-specific behaviors"""

        goal_msg = NavigateToPose.Goal()

        # Set target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (convert theta to quaternion)
        goal_msg.pose.pose.orientation.z = np.sin(theta / 2)
        goal_msg.pose.pose.orientation.w = np.cos(theta / 2)

        # Behavior tree configuration
        goal_msg.behavior_tree = ''  # Use default, or custom XML

        # Wait for action server
        self.nav_client.wait_for_server()

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result:
            self.get_logger().info('Goal reached!')
            return True
        else:
            self.get_logger().warn('Navigation failed')
            return False

def main():
    rclpy.init()
    navigator = HumanoidNavigator()

    # Example: navigate to (2.0, 1.5) facing 45 degrees
    navigator.navigate_to_goal(2.0, 1.5, np.pi/4)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File

```python
# launch/humanoid_nav2.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_humanoid_robot')

    return LaunchDescription([
        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[os.path.join(pkg_dir, 'config', 'controller.yaml')],
            remappings=[('/cmd_vel', '/humanoid/cmd_vel')]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[os.path.join(pkg_dir, 'config', 'planner_server.yaml')]
        ),

        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=[os.path.join(pkg_dir, 'config', 'behavior.yaml')]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')]
        ),

        # Footstep Planner
        Node(
            package='my_humanoid_robot',
            executable='footstep_planner',
        ),
    ])
```

## Testing and Visualization

```bash
# Launch Nav2
ros2 launch my_humanoid_robot humanoid_nav2.launch.py

# Open RViz2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# Send a test goal via command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# Monitor performance
ros2 topic hz /plan  # Path generation rate
ros2 topic echo /cmd_vel  # Velocity commands
```

## Handling Stairs and Uneven Terrain

Nav2 assumes flat ground. For stairs, we need custom planning.

```python
class StairClimbingBehavior(Node):
    def __init__(self):
        super().__init__('stair_climbing_behavior')

        # Detect stairs in costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.detect_stairs,
            10
        )

    def detect_stairs(self, costmap_msg):
        """Identify stair-like obstacles"""

        # Look for vertical edges in costmap
        # If edge height > 0.1m and < 0.2m → stairs
        # If edge height > 0.2m → wall (avoid)

        # Simplified detection:
        # Count cells with high cost in vertical line

        pass  # Implement stair detection

    def plan_stair_climb(self, stair_location):
        """Generate footsteps for stair climbing"""

        # 1. Align robot perpendicular to stairs
        # 2. Lift leading foot to step height
        # 3. Shift weight, bring trailing foot
        # 4. Repeat for N steps

        footsteps = []
        step_height = 0.18  # Standard stair: 18cm
        step_depth = 0.28   # Tread: 28cm

        for step_idx in range(num_stairs):
            # Leading foot
            footsteps.append({
                'foot': 'left' if step_idx % 2 == 0 else 'right',
                'x': stair_location.x + step_idx * step_depth,
                'y': stair_location.y,
                'z': step_idx * step_height
            })

        return footsteps
```

## Summary

Nav2 provides a robust navigation framework that can be adapted for humanoid robots:

**Key Adaptations:**
1. **Slower velocities**: 0.5-1.5 m/s walking speed
2. **No lateral motion**: Set `max_vel_y = 0`
3. **Longer sim time**: 2-3 seconds for trajectory prediction
4. **Higher path alignment**: Humanoids cannot deviate easily
5. **Footstep planning**: Convert smooth paths to discrete steps

**Best Practices:**
- Use SmacPlanner2D for better orientation handling
- Inflate obstacles generously (humanoids are less agile)
- Add recovery behaviors for falls
- Integrate gait controller for stable walking
- Save maps for faster startup

**Performance Tips:**
- Local costmap: 5-10 Hz update
- Global costmap: 1 Hz update
- Controller: 20 Hz (balance-critical)
- Path planning: On-demand (expensive)

With proper tuning, Nav2 enables humanoid robots to navigate autonomously while respecting the constraints of bipedal locomotion.
