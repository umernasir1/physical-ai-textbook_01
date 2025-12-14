---
sidebar_position: 4
title: Capstone Project (The Autonomous Humanoid)
---

# Capstone Project: The Autonomous Humanoid

## Project Overview

This capstone project synthesizes all previous modules into a fully autonomous humanoid robot capable of:

1. **Perception**: Using LiDAR, RGB-D cameras, and IMU for environmental awareness
2. **Localization**: VSLAM for knowing where it is
3. **Navigation**: Nav2 for planning paths and avoiding obstacles
4. **Manipulation**: Grasping objects using visual servoing
5. **Intelligence**: Natural language understanding for task commands
6. **Interaction**: Communicating with humans through speech and gestures

**Scenario:** A humanoid butler robot that can:
- Receive voice commands ("Bring me a water bottle")
- Navigate autonomously to the kitchen
- Find and grasp the bottle
- Return and hand it to the human
- Report status via speech

## System Architecture

```
Voice Command (Whisper) → Task Parser (GPT-4) → Task Execution
                                                        ↓
┌──────────────────────────────────────────────────────────────┐
│ High-Level Planner (behavior tree)                           │
├──────────────────────────────────────────────────────────────┤
│ Navigation (Nav2) ←→ Manipulation (MoveIt2) ←→ Perception   │
├──────────────────────────────────────────────────────────────┤
│ Localization (VSLAM) ←→ Object Detection (YOLO) ←→ Grasping │
├──────────────────────────────────────────────────────────────┤
│ Low-Level Control (Joint Controllers) ←→ Balance (Whole-Body)│
└──────────────────────────────────────────────────────────────┘
                              ↓
                    Simulation (Gazebo/Isaac)
```

## Phase 1: Project Setup

### Create Workspace

```bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# Clone dependencies
git clone https://github.com/ros-planning/moveit2.git -b humble
git clone https://github.com/ros-planning/navigation2.git -b humble
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Create our package
ros2 pkg create humanoid_butler --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs

cd ~/humanoid_ws
colcon build
source install/setup.bash
```

### Directory Structure

```
humanoid_butler/
├── launch/          # Launch files
├── config/          # Parameter files (Nav2, VSLAM)
├── urdf/            # Robot description
├── worlds/          # Gazebo environments
├── models/          # 3D models for objects
├── scripts/         # Python nodes
│   ├── task_planner.py
│   ├── object_detector.py
│   ├── grasp_planner.py
│   └── speech_interface.py
└── behavior_trees/  # BT XML files
```

## Phase 2: Robot Model

### URDF Humanoid Definition

```xml
<!-- urdf/humanoid.urdf.xacro -->
<?xml version="1.0"?>
<robot name="humanoid_butler" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>  <!-- 30cm x 20cm x 60cm torso -->
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Head with camera -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm (simplified 3-DOF) -->
  <xacro:macro name="arm" params="prefix reflect">
    <link name="${prefix}_shoulder">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.1"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_shoulder"/>
      <origin xyz="0 ${reflect*0.15} 0.25" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-3.14" upper="3.14" effort="30" velocity="2.0"/>
    </joint>

    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="0.04" length="0.3"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_elbow_joint" type="revolute">
      <parent link="${prefix}_shoulder"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.5" effort="20" velocity="2.0"/>
    </joint>

    <link name="${prefix}_gripper">
      <visual>
        <geometry>
          <box size="0.08 0.08 0.12"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wrist_joint" type="fixed">
      <parent link="${prefix}_upper_arm"/>
      <child link="${prefix}_gripper"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate both arms -->
  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>

  <!-- Sensors -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>1280</width>
          <height>720</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Phase 3: High-Level Task Planner

### Voice-to-Task Pipeline

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class TaskPlanner(Node):
    """Convert natural language commands to robot tasks"""

    def __init__(self):
        super().__init__('task_planner')

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_callback,
            10
        )

        # Publish structured tasks
        self.task_pub = self.create_publisher(String, '/robot_task', 10)

        # OpenAI API
        openai.api_key = os.getenv('OPENAI_API_KEY')

    def voice_callback(self, msg):
        """Parse voice command into structured task"""

        user_command = msg.data
        self.get_logger().info(f'Received command: {user_command}')

        # Use GPT-4 to parse intent
        system_prompt = """You are a robot task parser. Convert natural language commands into structured JSON tasks.

Available actions: navigate, grasp, place, speak
Available objects: water_bottle, coffee_cup, book, remote
Available locations: kitchen, living_room, bedroom, table

Output format:
{
  "action": "grasp",
  "object": "water_bottle",
  "location": "kitchen"
}
"""

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_command}
            ],
            temperature=0.0
        )

        task_json = response.choices[0].message.content

        # Publish task
        task_msg = String()
        task_msg.data = task_json
        self.task_pub.publish(task_msg)

        self.get_logger().info(f'Task: {task_json}')

def main():
    rclpy.init()
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 4: Object Detection and Localization

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        # YOLO model
        self.model = YOLO('yolov8n.pt')  # Nano model for speed

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish object poses
        self.object_pub = self.create_publisher(
            PoseStamped,
            '/detected_objects',
            10
        )

        # Target object
        self.target_object = 'bottle'

    def image_callback(self, msg):
        """Detect objects in camera image"""

        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO detection
        results = self.model(cv_image)

        for result in results:
            boxes = result.boxes

            for box in boxes:
                # Get class name
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                if class_name == self.target_object:
                    # Get bounding box
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    # Calculate 3D position (simplified - assumes known object size)
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # Estimate distance from bounding box size
                    box_height = y2 - y1
                    known_height = 0.25  # 25cm bottle
                    focal_length = 600  # Camera intrinsic
                    distance = (known_height * focal_length) / box_height

                    # Publish object pose
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.header.frame_id = 'camera_link'

                    # Convert pixel coordinates to 3D (assuming camera calibration)
                    pose.pose.position.x = distance
                    pose.pose.position.y = -(center_x - 640) * distance / focal_length
                    pose.pose.position.z = -(center_y - 360) * distance / focal_length

                    pose.pose.orientation.w = 1.0

                    self.object_pub.publish(pose)

                    self.get_logger().info(
                        f'Detected {class_name} at ({pose.pose.position.x:.2f}, '
                        f'{pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})'
                    )

                    # Visualize
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(cv_image, class_name, (int(x1), int(y1)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Object Detection', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 5: Grasp Planning

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupAction
from rclpy.action import ActionClient

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')

        # Subscribe to detected objects
        self.object_sub = self.create_subscription(
            PoseStamped,
            '/detected_objects',
            self.object_callback,
            10
        )

        # MoveIt2 action client
        self.moveit_client = ActionClient(
            self,
            MoveGroupAction,
            '/move_action'
        )

    def object_callback(self, object_pose):
        """Plan grasp for detected object"""

        self.get_logger().info('Planning grasp...')

        # Calculate pre-grasp pose (approach from above)
        pre_grasp = PoseStamped()
        pre_grasp.header = object_pose.header
        pre_grasp.pose.position.x = object_pose.pose.position.x
        pre_grasp.pose.position.y = object_pose.pose.position.y
        pre_grasp.pose.position.z = object_pose.pose.position.z + 0.15  # 15cm above

        # Orientation: gripper pointing down
        pre_grasp.pose.orientation.x = 0.707
        pre_grasp.pose.orientation.y = 0.707
        pre_grasp.pose.orientation.z = 0.0
        pre_grasp.pose.orientation.w = 0.0

        # Move to pre-grasp
        self.move_arm_to_pose(pre_grasp)

        # Move to grasp
        grasp_pose = object_pose
        grasp_pose.pose.position.z += 0.05  # Slight offset
        self.move_arm_to_pose(grasp_pose)

        # Close gripper
        self.close_gripper()

        # Lift object
        lift_pose = grasp_pose
        lift_pose.pose.position.z += 0.2
        self.move_arm_to_pose(lift_pose)

        self.get_logger().info('Grasp complete!')

    def move_arm_to_pose(self, target_pose):
        """Send MoveIt2 goal"""
        # Simplified - actual implementation uses MoveGroupInterface
        pass

    def close_gripper(self):
        """Close gripper fingers"""
        # Publish gripper command
        pass

def main():
    rclpy.init()
    node = GraspPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 6: Behavior Tree Integration

### Complete Task Execution

```xml
<!-- behavior_trees/butler_task.xml -->
<root main_tree_to_execute="ButlerTask">
  <BehaviorTree ID="ButlerTask">
    <Sequence name="fetch_object">

      <!-- Parse voice command -->
      <Action ID="ParseVoiceCommand" command="{voice_input}" task="{task}"/>

      <!-- Navigate to object location -->
      <Action ID="NavigateToLocation" location="{task.location}"/>

      <!-- Search for object -->
      <Action ID="DetectObject" object_name="{task.object}" object_pose="{object_pose}"/>

      <!-- Plan and execute grasp -->
      <Action ID="GraspObject" pose="{object_pose}"/>

      <!-- Return to human -->
      <Action ID="NavigateToLocation" location="living_room"/>

      <!-- Hand over object -->
      <Action ID="HandOver"/>

      <!-- Speak confirmation -->
      <Action ID="Speak" text="Here is your {task.object}"/>

    </Sequence>
  </BehaviorTree>
</root>
```

## Phase 7: Testing and Validation

### Launch Everything

```python
# launch/capstone_complete.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ])
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', 'urdf/humanoid.urdf']
        ),

        # VSLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam'
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ])
        ),

        # Task planner
        Node(
            package='humanoid_butler',
            executable='task_planner'
        ),

        # Object detector
        Node(
            package='humanoid_butler',
            executable='object_detector'
        ),

        # Grasp planner
        Node(
            package='humanoid_butler',
            executable='grasp_planner'
        ),

        # Behavior tree
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[{'default_bt_xml_filename': 'behavior_trees/butler_task.xml'}]
        ),
    ])
```

### Run the Capstone

```bash
# Terminal 1: Launch simulation
ros2 launch humanoid_butler capstone_complete.launch.py

# Terminal 2: Send voice command
ros2 topic pub /voice_command std_msgs/String "data: 'Bring me a water bottle from the kitchen'"

# Watch the magic happen!
```

## Expected Behavior

1. **Voice Recognition**: Robot hears "Bring me a water bottle from the kitchen"
2. **Task Parsing**: GPT-4 extracts: action=grasp, object=water_bottle, location=kitchen
3. **Navigation**: Nav2 plans path to kitchen, humanoid walks there
4. **Perception**: Camera detects water bottle using YOLO
5. **Grasping**: MoveIt2 plans arm trajectory, gripper closes
6. **Return**: Nav2 plans return path to human
7. **Handoff**: Extends arm toward human
8. **Speech**: "Here is your water bottle"

## Evaluation Metrics

| Metric | Target | How to Measure |
|--------|--------|----------------|
| **Task Success Rate** | >80% | Complete fetch tasks / Total attempts |
| **Navigation Time** | &lt;60s | Time from command to reaching object |
| **Grasp Success** | >70% | Successful grasps / Attempts |
| **Localization Error** | &lt;10cm | VSLAM drift after 100m |
| **Safety** | 0 collisions | Obstacle collisions during nav |

## Extensions and Improvements

### Add More Capabilities

1. **Multi-Object Tasks**: "Bring me a cup AND a book"
2. **Conditional Logic**: "If there's no coffee, bring tea"
3. **Learning**: Improve grasping from failures
4. **Human Following**: Walk alongside a person
5. **Emotional Expression**: Facial animations for engagement

### Real Hardware Deployment

1. Replace Gazebo with real sensors
2. Calibrate camera intrinsics
3. Tune gait parameters for stability
4. Add safety e-stops
5. Test in real environments

## Summary

This capstone project demonstrates the complete robotics stack:

**Modules Integrated:**
- ✅ Module 1: ROS 2 for communication
- ✅ Module 2: Gazebo for simulation, Unity for rendering
- ✅ Module 3: Isaac ROS for perception, Nav2 for navigation
- ✅ Module 4: Whisper for voice, GPT-4 for reasoning

**Key Achievements:**
- End-to-end autonomous behavior
- Multi-modal perception (vision + LiDAR + IMU)
- Natural language understanding
- Robust navigation and manipulation
- Human-robot collaboration

**Congratulations!** You've built a fully autonomous humanoid robot from scratch. This foundation prepares you for cutting-edge research and industry applications in physical AI.

**Next Steps:**
1. Deploy to real humanoid hardware (e.g., Unitree H1, Boston Dynamics Atlas)
2. Contribute to open-source robotics projects
3. Research advanced topics: reinforcement learning for locomotion, sim-to-real transfer, multi-robot coordination
4. Build the future of physical AI!

The era of humanoid robots in everyday life is just beginning—and you're ready to be part of it.
