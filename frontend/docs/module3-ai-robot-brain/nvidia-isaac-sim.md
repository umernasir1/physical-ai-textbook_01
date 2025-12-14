---
sidebar_position: 2
title: NVIDIA Isaac Sim
---

# NVIDIA Isaac Sim

## Introduction to Photorealistic Robot Simulation

NVIDIA Isaac Sim is a revolutionary robot simulation platform built on NVIDIA Omniverse. Unlike traditional simulators (Gazebo, PyBullet), Isaac Sim provides photorealistic rendering, physically accurate simulations, and seamless integration with AI training workflows—all running on RTX GPUs for real-time performance.

**Why Isaac Sim is Different:**

- **Ray-Traced Rendering**: Photorealistic lighting, reflections, and shadows using RTX technology
- **PhysX 5 Engine**: Highly accurate rigid body dynamics, soft body physics, and collision detection
- **USD Foundation**: Universal Scene Description (USD) enables asset sharing across film, gaming, and robotics
- **AI-Ready**: Native integration with synthetic data generation and reinforcement learning
- **ROS 2 Bridge**: Seamless connection to ROS 2 ecosystem for real robot deployment

## Getting Started with Isaac Sim

### System Requirements

**Minimum Specifications:**
- GPU: NVIDIA RTX 2070 or higher (8GB VRAM)
- CPU: Intel Core i7 or AMD Ryzen 7
- RAM: 32GB
- OS: Ubuntu 20.04/22.04 or Windows 10/11
- Storage: 50GB SSD space

**Recommended Specifications:**
- GPU: NVIDIA RTX 4080 or higher (12GB+ VRAM)
- CPU: Intel Core i9 13th Gen or AMD Ryzen 9
- RAM: 64GB DDR5
- OS: Ubuntu 22.04 LTS (native ROS 2 support)

### Installation

```bash
# Download and install Omniverse Launcher
# Visit: https://www.nvidia.com/en-us/omniverse/download/

# Install Isaac Sim from Omniverse Launcher (select latest version)
# Typical path: ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Set environment variables
echo 'export ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1"' >> ~/.bashrc
echo 'export PYTHONPATH="${ISAAC_SIM_PATH}/exts/omni.isaac.python_app/pip_prebundle:${PYTHONPATH}"' >> ~/.bashrc
source ~/.bashrc

# Verify installation
cd $ISAAC_SIM_PATH
./isaac-sim.sh
```

### First Simulation: Hello World

```python
# hello_isaac.py
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create simulation world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Add a dynamic cube
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="my_cube",
    position=[0, 0, 1.0],  # 1 meter above ground
    size=0.2,  # 20cm cube
    color=[1.0, 0.0, 0.0]  # Red
)

world.scene.add(cube)

# Reset simulation
world.reset()

# Run simulation for 1000 steps
for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
```

Run the script:
```bash
cd $ISAAC_SIM_PATH
./python.sh /path/to/hello_isaac.py
```

## Understanding the Isaac Sim Architecture

### USD (Universal Scene Description)

Isaac Sim uses USD as its foundation—a file format developed by Pixar for complex 3D scenes.

**Key USD Concepts:**

1. **Prims (Primitives)**: Basic building blocks of a scene (meshes, lights, cameras)
2. **Properties**: Attributes of prims (position, rotation, color)
3. **Hierarchy**: Prims organized in a tree structure (like a file system)
4. **Layers**: Non-destructive editing through layer composition

```python
from pxr import Usd, UsdGeom, Gf

# Access the USD stage
stage = omni.usd.get_context().get_stage()

# Create a sphere prim
sphere_path = "/World/Sphere"
sphere_geom = UsdGeom.Sphere.Define(stage, sphere_path)

# Set properties
sphere_geom.GetRadiusAttr().Set(0.5)
sphere_geom.AddTranslateOp().Set(Gf.Vec3f(1.0, 0.0, 0.5))

# Set color (requires material binding in practice)
color_attr = sphere_geom.GetDisplayColorAttr()
color_attr.Set([(0.0, 1.0, 0.0)])  # Green
```

### Physics Simulation with PhysX 5

Isaac Sim integrates NVIDIA PhysX 5 for realistic physics:

```python
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.physics import simulate_async

# Create a rigid body with physics
from pxr import UsdPhysics, PhysxSchema

# Define rigid body
rigid_body = UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())

# Add collision shape
collision_api = UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())

# Set physics properties
mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
mass_api.GetMassAttr().Set(1.0)  # 1 kg

# Set friction and restitution
material = UsdPhysics.MaterialAPI.Apply(sphere_geom.GetPrim())
material.CreateStaticFrictionAttr().Set(0.5)
material.CreateDynamicFrictionAttr().Set(0.4)
material.CreateRestitutionAttr().Set(0.6)  # Bounciness
```

## Loading and Simulating Robots

### Importing Robot Models

Isaac Sim supports multiple robot formats:

**1. URDF Import:**
```python
from omni.isaac.urdf import _urdf

# Import URDF (converted to USD internally)
urdf_path = "/path/to/robot.urdf"
usd_path = "/World/Robot"

# Import configuration
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = True  # For collision meshes
import_config.import_inertia_tensor = True
import_config.fix_base = False  # True for fixed-base robots

# Import robot
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path=usd_path
)
```

**2. Loading Pre-built Assets:**
```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load from Isaac Sim asset library
robot_asset = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Unitree/G1/g1.usd"

add_reference_to_stage(
    usd_path=robot_asset,
    prim_path="/World/Humanoid"
)
```

### Articulation Controller

Control robot joints using the Articulation API:

```python
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

# Create articulation controller
robot = Articulation(prim_path="/World/Humanoid")
robot.initialize()

# Get joint information
joint_names = robot.dof_names
num_dof = robot.num_dof
print(f"Robot has {num_dof} degrees of freedom: {joint_names}")

# Set joint positions (position control)
target_positions = [0.0] * num_dof  # All joints to 0
target_positions[0] = 0.5  # First joint to 0.5 radians

action = ArticulationAction(joint_positions=target_positions)
robot.apply_action(action)

# Set joint velocities (velocity control)
target_velocities = [0.0] * num_dof
target_velocities[1] = 1.0  # Second joint at 1 rad/s

action = ArticulationAction(joint_velocities=target_velocities)
robot.apply_action(action)

# Set joint efforts (torque control)
target_efforts = [0.0] * num_dof
target_efforts[2] = 10.0  # Apply 10 Nm torque to third joint

action = ArticulationAction(joint_efforts=target_efforts)
robot.apply_action(action)
```

## Sensor Simulation

Isaac Sim provides accurate sensor models essential for perception testing.

### RGB Camera

```python
from omni.isaac.sensor import Camera
import numpy as np

# Create RGB camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 1.5]),
    frequency=30,  # 30 Hz
    resolution=(1280, 720),
    orientation=np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
)

# Initialize camera
camera.initialize()

# Capture frame
camera.add_distance_to_camera_to_frame()
frame_data = camera.get_current_frame()

rgb_image = frame_data["rgba"][:, :, :3]  # Remove alpha channel
print(f"Captured image shape: {rgb_image.shape}")
```

### Depth Camera

```python
# Enable depth output
camera.add_depth_to_frame()

# Capture depth
frame_data = camera.get_current_frame()
depth_image = frame_data["depth"]

# Depth is in meters
print(f"Min depth: {depth_image.min()}, Max depth: {depth_image.max()}")
```

### LiDAR Simulation

```python
from omni.isaac.range_sensor import _range_sensor

# Create LiDAR sensor
lidar_config = _range_sensor.LidarSensorConfig()
lidar_config.min_range = 0.1  # meters
lidar_config.max_range = 100.0
lidar_config.horizontal_fov = 360.0  # degrees
lidar_config.vertical_fov = 30.0
lidar_config.horizontal_resolution = 0.4  # degrees
lidar_config.vertical_resolution = 1.0
lidar_config.rotation_rate = 20.0  # Hz

# Create LiDAR
result, lidar = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path="/World/Lidar",
    parent="/World/Robot",
    config=lidar_config,
    translation=(0, 0, 0.5)
)

# Read LiDAR data
lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
depth_data = lidar_interface.get_linear_depth_data("/World/Lidar")
azimuth_data = lidar_interface.get_azimuth_data("/World/Lidar")
elevation_data = lidar_interface.get_elevation_data("/World/Lidar")
```

### IMU (Inertial Measurement Unit)

```python
from omni.isaac.sensor import IMUSensor

# Create IMU
imu = IMUSensor(
    prim_path="/World/Robot/IMU",
    translation=np.array([0, 0, 0.1]),  # Mounted on robot
    frequency=100  # 100 Hz
)

imu.initialize()

# Read IMU data
imu_reading = imu.get_current_frame()

linear_acceleration = imu_reading["lin_acc"]  # m/s^2
angular_velocity = imu_reading["ang_vel"]  # rad/s
orientation = imu_reading["orientation"]  # Quaternion

print(f"Acceleration: {linear_acceleration}")
print(f"Angular velocity: {angular_velocity}")
```

## Environment Building

### Creating a Warehouse Scene

```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim

world = World()

# Add warehouse floor
floor_prim = create_prim(
    prim_path="/World/Floor",
    prim_type="Cube",
    position=[0, 0, -0.05],
    scale=[20, 20, 0.1],
    semantic_label="floor"
)

# Add shelves (repeated)
for i in range(5):
    shelf_path = f"/World/Shelf_{i}"
    add_reference_to_stage(
        usd_path="omniverse://localhost/NVIDIA/Assets/Industrial/Shelves/shelf_unit.usd",
        prim_path=shelf_path
    )
    # Position shelves
    create_prim(shelf_path).GetAttribute("xformOp:translate").Set((i * 2.0, 0, 0))

# Add boxes on shelves
for i in range(20):
    box = DynamicCuboid(
        prim_path=f"/World/Box_{i}",
        position=[
            np.random.uniform(-5, 5),
            np.random.uniform(-5, 5),
            np.random.uniform(0.5, 2.0)
        ],
        size=0.2,
        color=np.random.rand(3)
    )
    world.scene.add(box)

# Add lighting
from pxr import UsdLux
dome_light = UsdLux.DomeLight.Define(world.stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1000.0)
```

### Randomized Environments for Training

```python
import random

class EnvironmentRandomizer:
    """Randomize environments for robust AI training"""
    def __init__(self, world):
        self.world = world

    def randomize_lighting(self):
        """Randomize scene lighting"""
        stage = self.world.stage
        dome_light = UsdLux.DomeLight.Get(stage, "/World/DomeLight")

        # Random intensity
        intensity = random.uniform(500, 2000)
        dome_light.GetIntensityAttr().Set(intensity)

        # Random color temperature
        temperature = random.uniform(3000, 7000)
        dome_light.GetColorTemperatureAttr().Set(temperature)

    def randomize_object_poses(self):
        """Randomize object positions"""
        for i in range(20):
            box_path = f"/World/Box_{i}"
            prim = self.world.stage.GetPrimAtPath(box_path)

            new_position = (
                random.uniform(-5, 5),
                random.uniform(-5, 5),
                random.uniform(0.5, 2.0)
            )

            prim.GetAttribute("xformOp:translate").Set(new_position)

    def randomize_materials(self):
        """Randomize object materials and textures"""
        from omni.isaac.core.materials import PreviewSurface

        for i in range(20):
            box_path = f"/World/Box_{i}"
            material = PreviewSurface(prim_path=f"{box_path}/Material")

            # Random color
            color = (random.random(), random.random(), random.random())
            material.set_color(color)

            # Random metallic/roughness
            material.set_metallic(random.uniform(0, 1))
            material.set_roughness(random.uniform(0, 1))
```

## Integration with ROS 2

Isaac Sim provides a native ROS 2 bridge for seamless integration:

```python
# Enable ROS 2 bridge
import omni.isaac.ros2_bridge as ros2_bridge

# Start ROS 2 bridge
ros2_bridge.enable_ros2()

# Publish camera data to ROS 2
camera_helper = ros2_bridge.create_camera_publisher(
    camera_prim_path="/World/Camera",
    topic_name="/camera/image_raw",
    message_type="sensor_msgs/Image",
    publish_rate=30
)

# Publish LiDAR data
lidar_helper = ros2_bridge.create_lidar_publisher(
    lidar_prim_path="/World/Lidar",
    topic_name="/scan",
    message_type="sensor_msgs/LaserScan"
)

# Subscribe to velocity commands
def cmd_vel_callback(linear_x, linear_y, angular_z):
    # Apply velocities to robot
    robot.set_linear_velocity([linear_x, linear_y, 0])
    robot.set_angular_velocity([0, 0, angular_z])

cmd_vel_sub = ros2_bridge.create_subscriber(
    topic_name="/cmd_vel",
    message_type="geometry_msgs/Twist",
    callback=cmd_vel_callback
)
```

## Reinforcement Learning with Isaac Sim

Train humanoid control policies using RL:

```python
from omni.isaac.gym.vec_env import VecEnvBase
import torch

class HumanoidWalkEnv(VecEnvBase):
    """Custom RL environment for humanoid walking"""
    def __init__(self, num_envs=4096):
        self.num_envs = num_envs
        super().__init__(num_envs=num_envs)

    def get_observations(self):
        """Return current observations"""
        # Joint positions, velocities
        joint_pos = self.robot.get_joint_positions()
        joint_vel = self.robot.get_joint_velocities()

        # Base orientation (from IMU)
        base_orientation = self.imu.get_current_frame()["orientation"]

        obs = torch.cat([joint_pos, joint_vel, base_orientation], dim=-1)
        return obs

    def calculate_reward(self):
        """Compute reward signal"""
        # Reward for forward movement
        velocity = self.robot.get_linear_velocity()
        forward_reward = velocity[0]

        # Penalty for falling
        height = self.robot.get_world_pose()[0][2]
        fall_penalty = -10.0 if height < 0.3 else 0.0

        # Penalty for high energy consumption
        joint_efforts = self.robot.get_measured_joint_efforts()
        energy_penalty = -0.001 * torch.sum(torch.abs(joint_efforts))

        return forward_reward + fall_penalty + energy_penalty

    def step(self, actions):
        """Execute actions and return results"""
        # Apply actions (joint targets)
        self.robot.apply_action(ArticulationAction(joint_positions=actions))

        # Simulate
        self.world.step()

        # Get observations
        obs = self.get_observations()

        # Calculate rewards
        rewards = self.calculate_reward()

        # Check for episode termination
        dones = self.robot.get_world_pose()[0][:, 2] < 0.2  # Fallen

        return obs, rewards, dones, {}
```

## Best Practices for Isaac Sim

1. **Performance Optimization**
   - Use headless mode for batch data generation
   - Reduce scene complexity when possible
   - Enable GPU acceleration for physics

2. **Asset Management**
   - Use USD references instead of duplicating assets
   - Organize scenes with clear hierarchy
   - Leverage Omniverse Nucleus for team collaboration

3. **Debugging**
   - Use visual debugger for physics issues
   - Enable collision visualization
   - Log joint states and sensor data

4. **Sim-to-Real**
   - Add realistic sensor noise
   - Implement domain randomization
   - Validate with real-world data early

## Conclusion

NVIDIA Isaac Sim transforms robot development by providing:
- Photorealistic simulation for accurate perception training
- Hardware-accelerated physics for real-time performance
- Seamless ROS 2 integration for deployment
- Native support for reinforcement learning workflows

**Next**: We'll explore Isaac ROS—hardware-accelerated perception pipelines for VSLAM and navigation on Jetson devices.

---

**Key Takeaways:**
- Isaac Sim provides photorealistic rendering and accurate physics simulation
- USD enables powerful asset management and scene composition
- Sensor simulation (cameras, LiDAR, IMU) generates realistic training data
- ROS 2 bridge enables seamless integration with robot software stacks
- Built-in RL support accelerates policy training for locomotion and manipulation
