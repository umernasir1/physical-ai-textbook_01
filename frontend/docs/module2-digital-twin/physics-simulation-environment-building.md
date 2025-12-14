---
sidebar_position: 1
title: Physics Simulation and Environment Building
---

# Physics Simulation and Environment Building

## Introduction to Digital Twins

A **Digital Twin** is a virtual replica of a physical system that accurately simulates its behavior in the real world. For robotics, digital twins enable:

- **Safe Testing**: Experiment with robot behaviors without risking hardware damage
- **Rapid Prototyping**: Iterate on designs quickly before physical manufacturing
- **Training AI**: Generate synthetic data for machine learning models
- **Sim-to-Real Transfer**: Develop control algorithms that work in both simulation and reality

In this chapter, we'll explore **Gazebo**, the industry-standard physics simulator for ROS 2, and learn how to create realistic environments for humanoid robot testing.

## Why Gazebo for Physical AI?

**Gazebo** is an open-source 3D robotics simulator that provides:

- **Accurate Physics**: ODE, Bullet, Simbody, and DART physics engines
- **Rich Sensor Support**: Cameras, LiDAR, IMUs, force/torque sensors
- **ROS 2 Integration**: Seamless communication with robot control systems
- **Plugin Architecture**: Extensible for custom behaviors
- **Distributed Simulation**: Run physics on different machines

**Industry Adoption:**
- NASA uses Gazebo for space robotics
- DARPA Robotics Challenge simulations
- Autonomous vehicle testing (CARLA is built on Unreal, but Gazebo remains dominant for robots)

## Getting Started with Gazebo

### Installation (Ubuntu 22.04)

```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version

# Launch Gazebo
gazebo
```

### Gazebo Architecture

Gazebo consists of two main components:

1. **gzserver**: Physics simulation and sensor generation (headless)
2. **gzclient**: GUI for visualization and interaction

```bash
# Run server only (for training, no GUI overhead)
gzserver worlds/my_world.world

# Run client (connect to existing server)
gzclient
```

## Understanding Physics Engines

Gazebo supports multiple physics engines. Each has trade-offs:

| Engine | Strengths | Weaknesses | Best For |
|--------|-----------|------------|----------|
| **ODE** (Default) | Fast, stable, widely used | Less accurate contacts | General robotics |
| **Bullet** | Realistic collisions, open-source | Slower than ODE | Manipulation tasks |
| **DART** | Accurate dynamics, constraint handling | Steep learning curve | Bipedal locomotion |
| **Simbody** | High-fidelity biomechanics | Computationally expensive | Human motion simulation |

### Choosing a Physics Engine

For humanoid robots, **DART** is recommended because:
- Handles complex multi-body dynamics (many joints)
- Stable contact dynamics for foot-ground interaction
- Accurate constraint resolution for balance control

```xml
<world name="humanoid_world">
  <physics name="dart_physics" type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

## Building Your First World File

World files (`.world`) define the simulation environment. They use **SDF (Simulation Description Format)**.

### Basic World Structure

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_world">

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Physics Engine Configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

  </world>
</sdf>
```

### Adding a Ground Plane with Friction

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>    <!-- Coefficient of friction -->
            <mu2>1.0</mu2>
          </ode>
        </friction>
        <contact>
          <ode>
            <kp>1000000.0</kp>  <!-- Contact stiffness -->
            <kd>100.0</kd>       <!-- Contact damping -->
          </ode>
        </contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Simulating Gravity and Physical Forces

### Adjusting Gravity

For testing balance algorithms, you can modify gravity:

```xml
<!-- Earth gravity -->
<gravity>0 0 -9.81</gravity>

<!-- Moon gravity (1/6th of Earth) -->
<gravity>0 0 -1.625</gravity>

<!-- Mars gravity -->
<gravity>0 0 -3.71</gravity>

<!-- Zero gravity (space simulation) -->
<gravity>0 0 0</gravity>
```

### Wind Forces

```xml
<plugin name="wind" filename="libgazebo_ros_wind.so">
  <frame_id>world</frame_id>
  <wind_force_mean>5.0 0.0 0.0</wind_force_mean>  <!-- 5 m/s wind in X direction -->
  <wind_force_variance>1.0</wind_force_variance>
</plugin>
```

## Collision Detection and Response

### Collision Geometry Types

```xml
<!-- Box Collision -->
<collision name="box_collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
</collision>

<!-- Sphere Collision -->
<collision name="sphere_collision">
  <geometry>
    <sphere>
      <radius>0.5</radius>
    </sphere>
  </geometry>
</collision>

<!-- Cylinder Collision -->
<collision name="cylinder_collision">
  <geometry>
    <cylinder>
      <radius>0.3</radius>
      <length>1.0</length>
    </cylinder>
  </geometry>
</collision>

<!-- Mesh Collision (for complex shapes) -->
<collision name="mesh_collision">
  <geometry>
    <mesh>
      <uri>model://my_model/meshes/collision.stl</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</collision>
```

### Contact Properties for Humanoid Feet

Critical for stable bipedal locomotion:

```xml
<surface>
  <friction>
    <ode>
      <mu>1.5</mu>     <!-- High friction prevents slipping -->
      <mu2>1.5</mu2>
      <slip1>0.0</slip1>
      <slip2>0.0</slip2>
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>10000000.0</kp>  <!-- Very stiff contact (floor is hard) -->
      <kd>100.0</kd>        <!-- Damping prevents bouncing -->
      <max_vel>0.01</max_vel>
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
  <bounce>
    <restitution_coefficient>0.0</restitution_coefficient>  <!-- No bounce -->
  </bounce>
</surface>
```

## Building Complex Environments

### Creating an Indoor Navigation Scene

```xml
<world name="indoor_lab">

  <!-- Room Walls -->
  <model name="wall_north">
    <static>true</static>
    <pose>0 5 1.25 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>10 0.2 2.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>10 0.2 2.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
        </material>
      </visual>
    </link>
  </model>

  <!-- Repeat for south, east, west walls -->

  <!-- Obstacles -->
  <model name="table">
    <static>true</static>
    <pose>2 2 0.4 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>1.5 0.8 0.8</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1.5 0.8 0.8</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
        </material>
      </visual>
    </link>
  </model>

  <!-- Chair -->
  <include>
    <uri>model://chair</uri>
    <pose>2 3 0 0 0 1.57</pose>
  </include>

</world>
```

### Adding Dynamic Objects

Objects that the humanoid can interact with:

```xml
<model name="soda_can">
  <pose>1 1 0.6 0 0 0</pose>  <!-- On table -->
  <link name="link">
    <inertial>
      <mass>0.390</mass>  <!-- 390g full soda can -->
      <inertia>
        <ixx>0.00058</ixx>
        <iyy>0.00058</iyy>
        <izz>0.00019</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.031</radius>
          <length>0.115</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.031</radius>
          <length>0.115</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>  <!-- Red can -->
      </material>
    </visual>
  </link>
</model>
```

## Spawning Robots in Gazebo

### Method 1: Include in World File

```xml
<world name="my_world">
  <include>
    <uri>model://my_humanoid</uri>
    <name>robot1</name>
    <pose>0 0 1.0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  </include>
</world>
```

### Method 2: Spawn via ROS 2 Service

```bash
# Spawn URDF model
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file ~/robot.urdf -x 0 -y 0 -z 1.0
```

```python
# Python script to spawn robot
import rclpy
from gazebo_msgs.srv import SpawnEntity

def spawn_robot():
    node = rclpy.create_node('robot_spawner')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for spawn service...')

    # Read URDF file
    with open('/path/to/robot.urdf', 'r') as f:
        robot_desc = f.read()

    request = SpawnEntity.Request()
    request.name = 'my_humanoid'
    request.xml = robot_desc
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 1.0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()

if __name__ == '__main__':
    rclpy.init()
    spawn_robot()
    rclpy.shutdown()
```

## Gazebo Plugins for Robot Control

### Joint Position Controller Plugin

```xml
<gazebo>
  <plugin name="gazebo_ros_joint_state_publisher"
          filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=joint_states</remapping>
    </ros>
    <update_rate>100</update_rate>
    <joint_name>left_hip_pitch</joint_name>
    <joint_name>left_knee</joint_name>
    <joint_name>right_hip_pitch</joint_name>
    <joint_name>right_knee</joint_name>
  </plugin>
</gazebo>
```

### Joint Effort Controller

```xml
<gazebo>
  <plugin name="gazebo_ros_joint_effort"
          filename="libgazebo_ros_joint_effort.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/command:=joint_effort</remapping>
    </ros>
    <joint_name>left_knee</joint_name>
  </plugin>
</gazebo>
```

## Performance Optimization

### Simulation Speed vs. Accuracy Trade-offs

```xml
<physics type="ode">
  <!-- High accuracy (slower) -->
  <max_step_size>0.0001</max_step_size>  <!-- 10 kHz -->
  <real_time_factor>0.5</real_time_factor>  <!-- Runs at 0.5x real-time -->

  <!-- Balanced -->
  <max_step_size>0.001</max_step_size>  <!-- 1 kHz -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Fast (less accurate) -->
  <max_step_size>0.01</max_step_size>  <!-- 100 Hz -->
  <real_time_factor>2.0</real_time_factor>  <!-- Runs at 2x real-time -->
</physics>
```

### Collision Optimization

```xml
<!-- Use simple collision shapes -->
<collision name="simplified">
  <geometry>
    <cylinder radius="0.05" length="0.4"/>  <!-- Fast -->
  </geometry>
</collision>

<!-- Instead of complex meshes -->
<collision name="detailed">
  <geometry>
    <mesh>
      <uri>detailed_mesh.stl</uri>  <!-- Slow -->
    </mesh>
  </geometry>
</collision>
```

### Level of Detail (LOD)

For large environments, use simplified visuals at a distance:

```xml
<visual name="far_view">
  <geometry>
    <box size="1 1 1"/>  <!-- Simple box for distant viewing -->
  </geometry>
</visual>
```

## Creating Custom Gazebo Models

### Directory Structure

```
my_humanoid_gazebo/
├── model.config
├── model.sdf
└── meshes/
    ├── torso.stl
    ├── head.dae
    └── leg.obj
```

### model.config

```xml
<?xml version="1.0"?>
<model>
  <name>My Humanoid</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>

  <description>
    A humanoid robot for Physical AI research
  </description>
</model>
```

### model.sdf (Simplified)

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_humanoid">
    <static>false</static>

    <link name="torso">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>30.0</mass>
        <inertia>
          <ixx>2.5</ixx>
          <iyy>2.5</iyy>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://my_humanoid/meshes/torso.stl</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.4 0.6</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Add more links and joints here -->

  </model>
</sdf>
```

### Installing Custom Models

```bash
# Copy to Gazebo model path
mkdir -p ~/.gazebo/models
cp -r my_humanoid_gazebo ~/.gazebo/models/

# Or set environment variable
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/my_models
```

## Debugging Simulation Issues

### Common Problems and Solutions

**Problem: Robot falls through ground**
```xml
<!-- Solution: Increase contact stiffness -->
<surface>
  <contact>
    <ode>
      <kp>10000000.0</kp>
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
</surface>
```

**Problem: Robot explodes or shakes violently**
```xml
<!-- Solution: Reduce step size, increase damping -->
<physics>
  <max_step_size>0.0001</max_step_size>
</physics>

<joint name="knee">
  <dynamics>
    <damping>10.0</damping>  <!-- Increase damping -->
  </dynamics>
</joint>
```

**Problem: Simulation runs too slowly**
```bash
# Check real-time factor
gz stats

# Enable GPU acceleration (if available)
export GAZEBO_USE_GPU=1
```

### Visualization Tools

```bash
# View joint states
ros2 topic echo /joint_states

# Monitor contact forces
gz topic -e /gazebo/default/physics/contacts

# Check model tree
gz model -m my_robot -i
```

## Practical Exercise: Building a Humanoid Test Arena

Let's create a complete testing environment:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="humanoid_arena">

    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Flat walking area -->
    <model name="flat_surface">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.5</mu>
                <mu2>1.5</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Ramp for testing climbing -->
    <model name="ramp">
      <static>true</static>
      <pose>5 0 0.25 0 0.2 0</pose>  <!-- 11.5 degree slope -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 2 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 2 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Stairs -->
    <model name="stairs">
      <static>true</static>
      <pose>-3 0 0 0 0 0</pose>
      <link name="step1">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 2 0.2</size>
            </box>
          </geometry>
        </visual>
      </link>
      <link name="step2">
        <pose>0.3 0 0.3 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 2 0.2</size>
            </box>
          </geometry>
        </visual>
      </link>
      <!-- Add more steps -->
    </model>

  </world>
</sdf>
```

## Learning Outcomes

After completing this chapter, you should be able to:

- Understand the role of digital twins in robotics development
- Set up and configure Gazebo simulation environments
- Choose appropriate physics engines for different robot types
- Create realistic world files with proper gravity, friction, and collision properties
- Build complex environments with obstacles and interactive objects
- Spawn and control robots in simulation
- Optimize simulation performance for training and testing
- Debug common simulation issues

## Key Takeaways

- Digital twins enable safe, rapid prototyping of Physical AI systems
- Gazebo provides accurate physics simulation with multiple engine options
- DART engine is best for humanoid bipedal locomotion
- Proper contact properties (friction, stiffness, damping) are critical for realistic simulation
- Collision geometry should be simplified for performance
- World files use SDF format to define complete simulation environments
- Gazebo plugins enable ROS 2 integration for robot control

---

**Next**: We'll explore high-fidelity rendering and human-robot interaction using Unity for more realistic visualizations.
