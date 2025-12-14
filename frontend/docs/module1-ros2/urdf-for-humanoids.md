---
sidebar_position: 4
title: Understanding URDF for Humanoids
---

# Understanding URDF for Humanoids

## Introduction to URDF

**URDF (Unified Robot Description Format)** is an XML-based format for describing robot structures in ROS. It defines the kinematic and dynamic properties of a robot, including its links (rigid bodies), joints, sensors, and visual/collision geometries.

For humanoid robots, URDF becomes critical because:
- Humanoids have complex kinematic chains (legs, arms, spine, head)
- Accurate mass distribution affects balance and locomotion
- Joint limits and dynamics must match real hardware
- Collision geometries prevent self-intersection during motion planning

## URDF Structure: Core Components

### 1. Links: The Rigid Bodies

A **link** represents a rigid body in the robot structure. Each link has:
- **Visual geometry**: What you see in simulators
- **Collision geometry**: For physics calculations (usually simplified)
- **Inertial properties**: Mass, center of mass, inertia tensor

```xml
<link name="torso">
  <!-- Visual representation -->
  <visual>
    <geometry>
      <box size="0.3 0.4 0.6"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision representation (simplified) -->
  <collision>
    <geometry>
      <box size="0.3 0.4 0.6"/>
    </geometry>
  </collision>

  <!-- Physical properties -->
  <inertial>
    <mass value="30.0"/>
    <origin xyz="0 0 0.3"/>
    <inertia ixx="2.5" ixy="0.0" ixz="0.0"
             iyy="2.5" iyz="0.0" izz="0.5"/>
  </inertial>
</link>
```

### 2. Joints: Connecting Links

**Joints** define how links move relative to each other. Key joint types for humanoids:

**Revolute (Rotational):**
```xml
<joint name="left_hip_pitch" type="revolute">
  <parent link="pelvis"/>
  <child link="left_thigh"/>
  <origin xyz="0 0.1 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="1.0" friction="0.5"/>
</joint>
```

**Continuous (Unlimited rotation):**
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base"/>
  <child link="wheel"/>
  <axis xyz="0 0 1"/>
</joint>
```

**Fixed (No movement):**
```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
</joint>
```

**Prismatic (Linear motion):**
```xml
<joint name="telescope_extension" type="prismatic">
  <parent link="base"/>
  <child link="extension"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="50" velocity="0.1"/>
</joint>
```

## Humanoid-Specific Kinematic Chains

### Lower Body Structure

A typical humanoid lower body includes:
- **Pelvis** (base link)
- **Hip joints** (3 DOF per leg: yaw, roll, pitch)
- **Thigh links**
- **Knee joints** (1 DOF: pitch)
- **Shin links**
- **Ankle joints** (2 DOF: pitch, roll)
- **Feet links**

```xml
<!-- Left Leg Chain -->
<link name="pelvis"/>

<!-- Hip Yaw -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="pelvis"/>
  <child link="left_hip_yaw_link"/>
  <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.785" upper="0.785" effort="150" velocity="2.0"/>
</joint>

<link name="left_hip_yaw_link">
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<!-- Hip Roll -->
<joint name="left_hip_roll" type="revolute">
  <parent link="left_hip_yaw_link"/>
  <child link="left_hip_roll_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.524" upper="0.524" effort="150" velocity="2.0"/>
</joint>

<link name="left_hip_roll_link">
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<!-- Hip Pitch -->
<joint name="left_hip_pitch" type="revolute">
  <parent link="left_hip_roll_link"/>
  <child link="left_thigh"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="200" velocity="2.5"/>
</joint>

<link name="left_thigh">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.4" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="8.0"/>
    <origin xyz="0 0 -0.2"/>
    <inertia ixx="0.11" ixy="0" ixz="0" iyy="0.11" iyz="0" izz="0.02"/>
  </inertial>
</link>

<!-- Knee -->
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.36" effort="200" velocity="2.5"/>
</joint>

<link name="left_shin">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.4" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="6.0"/>
    <origin xyz="0 0 -0.2"/>
    <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.015"/>
  </inertial>
</link>

<!-- Ankle Pitch -->
<joint name="left_ankle_pitch" type="revolute">
  <parent link="left_shin"/>
  <child link="left_ankle_pitch_link"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.785" upper="0.785" effort="100" velocity="2.0"/>
</joint>

<link name="left_ankle_pitch_link">
  <inertial>
    <mass value="0.3"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
  </inertial>
</link>

<!-- Ankle Roll -->
<joint name="left_ankle_roll" type="revolute">
  <parent link="left_ankle_pitch_link"/>
  <child link="left_foot"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.524" upper="0.524" effort="100" velocity="2.0"/>
</joint>

<link name="left_foot">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
    <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
    <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <origin xyz="0.05 0 -0.025"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

## Calculating Inertial Properties

Accurate inertial properties are critical for realistic simulation and control.

### Mass Distribution Guidelines (70kg humanoid):

| Body Part | Approximate Mass | Percentage |
|-----------|------------------|------------|
| Head | 5.0 kg | 7% |
| Torso | 30.0 kg | 43% |
| Upper Arms (each) | 2.5 kg | 3.5% each |
| Forearms (each) | 1.5 kg | 2.1% each |
| Hands (each) | 0.5 kg | 0.7% each |
| Thighs (each) | 8.0 kg | 11.4% each |
| Shins (each) | 6.0 kg | 8.6% each |
| Feet (each) | 1.5 kg | 2.1% each |

### Inertia Tensor Calculation

For simple geometric shapes:

**Cylinder (along z-axis):**
```
Ixx = Iyy = (1/12) * m * (3r² + h²)
Izz = (1/2) * m * r²
```

**Box:**
```
Ixx = (1/12) * m * (h² + d²)
Iyy = (1/12) * m * (w² + d²)
Izz = (1/12) * m * (w² + h²)
```

**Sphere:**
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

### Python Helper for Inertia Calculation

```python
import numpy as np

def cylinder_inertia(mass, radius, height):
    """Calculate inertia tensor for a cylinder"""
    ixx = iyy = (1/12) * mass * (3 * radius**2 + height**2)
    izz = 0.5 * mass * radius**2
    return {'ixx': ixx, 'iyy': iyy, 'izz': izz}

def box_inertia(mass, width, height, depth):
    """Calculate inertia tensor for a box"""
    ixx = (1/12) * mass * (height**2 + depth**2)
    iyy = (1/12) * mass * (width**2 + depth**2)
    izz = (1/12) * mass * (width**2 + height**2)
    return {'ixx': ixx, 'iyy': iyy, 'izz': izz}

# Example: Calculate thigh inertia
thigh_mass = 8.0  # kg
thigh_radius = 0.05  # m
thigh_length = 0.4  # m

inertia = cylinder_inertia(thigh_mass, thigh_radius, thigh_length)
print(f"Thigh inertia: {inertia}")
# Output: {'ixx': 0.107, 'iyy': 0.107, 'izz': 0.01}
```

## Advanced URDF Features for Humanoids

### 1. Using Xacro for Modularity

**Xacro** (XML Macros) allows you to create reusable URDF components.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Define properties -->
  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="thigh_radius" value="0.05"/>
  <xacro:property name="thigh_mass" value="8.0"/>

  <!-- Macro for creating a leg -->
  <xacro:macro name="leg" params="prefix reflect">

    <!-- Hip Yaw Joint -->
    <joint name="${prefix}_hip_yaw" type="revolute">
      <parent link="pelvis"/>
      <child link="${prefix}_hip_yaw_link"/>
      <origin xyz="0 ${reflect * 0.1} -0.1" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.785" upper="0.785" effort="150" velocity="2.0"/>
    </joint>

    <link name="${prefix}_hip_yaw_link">
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <!-- Additional joints and links... -->

  </xacro:macro>

  <!-- Base link -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.3 0.4 0.2"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Instantiate both legs -->
  <xacro:leg prefix="left" reflect="1"/>
  <xacro:leg prefix="right" reflect="-1"/>

</robot>
```

### 2. Adding Sensors to URDF

```xml
<!-- IMU Sensor -->
<link name="imu_link"/>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>

<!-- Camera Sensor -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
        <remapping>camera_info:=camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Best Practices for Humanoid URDF

### 1. Joint Limit Safety
Always define realistic joint limits based on human biomechanics:

```xml
<!-- Knee can only bend forward (0 to ~135°) -->
<limit lower="0" upper="2.356" effort="200" velocity="2.5"/>

<!-- Elbow bends backward (0 to ~150°) -->
<limit lower="-2.618" upper="0" effort="100" velocity="3.0"/>
```

### 2. Collision Geometry Simplification
Use simplified collision shapes for performance:

```xml
<visual>
  <geometry>
    <mesh filename="package://humanoid_description/meshes/thigh_detailed.stl"/>
  </geometry>
</visual>

<collision>
  <!-- Simplified cylinder for collision detection -->
  <geometry>
    <cylinder length="0.4" radius="0.06"/>
  </geometry>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
</collision>
```

### 3. Gazebo-Specific Tags
Add Gazebo properties for realistic simulation:

```xml
<gazebo reference="left_foot">
  <mu1>1.0</mu1>  <!-- Friction coefficient 1 -->
  <mu2>1.0</mu2>  <!-- Friction coefficient 2 -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>  <!-- Contact damping -->
  <material>Gazebo/Grey</material>
</gazebo>
```

## Validating Your URDF

### Command-Line Validation

```bash
# Check URDF syntax
check_urdf my_humanoid.urdf

# Convert Xacro to URDF
xacro my_humanoid.xacro > my_humanoid.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=my_humanoid.urdf
```

### Python Validation Script

```python
#!/usr/bin/env python3
import xml.etree.ElementTree as ET

def validate_urdf(urdf_path):
    """Basic URDF validation"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links = [link.get('name') for link in root.findall('link')]
    joints = root.findall('joint')

    print(f"Found {len(links)} links")
    print(f"Found {len(joints)} joints")

    # Check joint parent-child relationships
    for joint in joints:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')

        if parent not in links:
            print(f"ERROR: Joint {joint.get('name')} has invalid parent: {parent}")
        if child not in links:
            print(f"ERROR: Joint {joint.get('name')} has invalid child: {child}")

    print("Validation complete!")

if __name__ == "__main__":
    validate_urdf("my_humanoid.urdf")
```

## Complete Minimal Humanoid Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Pelvis) -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.3 0.4 0.2"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="spine" type="revolute">
    <parent link="pelvis"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.3" upper="0.3" effort="50" velocity="1.0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.4 0.6"/>
      </geometry>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.4 0.6"/>
      </geometry>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="30.0"/>
      <origin xyz="0 0 0.3"/>
      <inertia ixx="2.5" ixy="0" ixz="0" iyy="2.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.045" ixy="0" ixz="0" iyy="0.045" iyz="0" izz="0.045"/>
    </inertial>
  </link>

</robot>
```

## Learning Outcomes

After completing this chapter, you should be able to:

- Understand the structure and purpose of URDF files
- Define links with appropriate visual, collision, and inertial properties
- Create joints with correct axes, limits, and dynamics
- Build complete kinematic chains for humanoid robots
- Calculate inertial properties for robot components
- Use Xacro for creating modular, reusable robot descriptions
- Validate and visualize URDF models
- Add sensors to robot descriptions

## Key Takeaways

- URDF is the standard format for robot description in ROS
- Accurate inertial properties are critical for simulation and control
- Humanoid robots require complex kinematic chains with many degrees of freedom
- Xacro enables modularity and reduces repetition in URDF files
- Simplified collision geometries improve simulation performance
- Always validate your URDF before deploying to simulation or hardware

---

**Next**: We'll explore physics simulation and environment building in Gazebo, where your URDF models come to life.
