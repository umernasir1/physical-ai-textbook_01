---
sidebar_position: 3
title: Bridging Python Agents to ROS Controllers using rclpy
---

# Bridging Python Agents to ROS Controllers using rclpy

## Introduction

In the age of AI-powered robotics, the ability to bridge intelligent software agents with physical robot controllers is essential. ROS 2 provides `rclpy`—a Python client library that enables seamless integration between AI models (like those built with TensorFlow, PyTorch, or OpenAI APIs) and robotic hardware.

This chapter covers how to:
- Set up `rclpy` for Python-based ROS 2 development
- Create nodes that integrate AI decision-making with robot control
- Design publisher-subscriber patterns for AI-robot communication
- Implement service-based AI inference for robotics

## Why Python for Robotics?

Python has become the lingua franca of AI and machine learning:

**Advantages:**
- Rich ecosystem of AI/ML libraries (TensorFlow, PyTorch, scikit-learn)
- Rapid prototyping and development
- Easy integration with cloud APIs (OpenAI, Google Cloud AI)
- Extensive data processing tools (NumPy, Pandas)

**Challenges:**
- Performance limitations compared to C++
- Global Interpreter Lock (GIL) restricts multithreading
- Not suitable for real-time critical control loops

**Best Practice:** Use Python for high-level planning and AI inference, C++ for low-level motor control and sensor processing.

## Setting Up rclpy

### Installation

```bash
# Install ROS 2 (Ubuntu 22.04 - Humble)
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
python3 -c "import rclpy; print('rclpy ready!')"
```

### Creating a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Building Your First AI-Enabled ROS 2 Node

### Example: AI Decision Node

Let's create a node that uses AI to make navigation decisions based on sensor data.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AINavigationNode(Node):
    def __init__(self):
        super().__init__('ai_navigation_node')

        # Create publisher for robot velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create subscriber for laser scan data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.get_logger().info('AI Navigation Node initialized')

    def laser_callback(self, msg):
        """Process laser scan data and make navigation decisions"""
        # Extract distances from laser scan
        ranges = np.array(msg.ranges)

        # Simple AI logic: avoid obstacles
        front_distance = np.mean(ranges[len(ranges)//2 - 10: len(ranges)//2 + 10])
        left_distance = np.mean(ranges[0:len(ranges)//4])
        right_distance = np.mean(ranges[3*len(ranges)//4:])

        # Create velocity command
        cmd = Twist()

        if front_distance < 0.5:  # Obstacle ahead
            # Turn toward more open space
            if left_distance > right_distance:
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.angular.z = -0.5  # Turn right
            cmd.linear.x = 0.0
        else:
            # Path is clear, move forward
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        # Publish command
        self.velocity_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AINavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components Explained

**1. Node Initialization:**
```python
super().__init__('ai_navigation_node')
```
Every ROS 2 node needs a unique name.

**2. Publisher Creation:**
```python
self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
```
- `Twist`: Message type for velocity commands
- `/cmd_vel`: Topic name (standard for robot velocity)
- `10`: Queue size for message buffering

**3. Subscriber Creation:**
```python
self.laser_subscriber = self.create_subscription(
    LaserScan, '/scan', self.laser_callback, 10
)
```
The callback function is called whenever new sensor data arrives.

## Integrating Deep Learning Models

### Example: Vision-Based Object Detection

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Load pretrained model
        self.model = fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()

        # OpenCV bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Object Detection Node ready')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Prepare image for model
        image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).float() / 255.0
        image_tensor = image_tensor.unsqueeze(0)

        # Run inference
        with torch.no_grad():
            predictions = self.model(image_tensor)

        # Process detections
        boxes = predictions[0]['boxes'].cpu().numpy()
        labels = predictions[0]['labels'].cpu().numpy()
        scores = predictions[0]['scores'].cpu().numpy()

        # Log detected objects
        for box, label, score in zip(boxes, labels, scores):
            if score > 0.7:  # Confidence threshold
                self.get_logger().info(f'Detected: Class {label}, Score: {score:.2f}')
```

## Services for AI Inference

For computationally expensive AI operations, use ROS 2 services instead of continuous callbacks.

### Creating an AI Inference Service

```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Replace with custom srv
import rclpy

class AIInferenceService(Node):
    def __init__(self):
        super().__init__('ai_inference_service')

        self.srv = self.create_service(
            AddTwoInts,  # Use custom service type in practice
            'ai_inference',
            self.inference_callback
        )

        # Load your AI model here
        # self.model = load_model('path/to/model.pth')

    def inference_callback(self, request, response):
        # Run AI inference
        self.get_logger().info(f'Inference request received: {request.a}, {request.b}')

        # Placeholder for actual AI computation
        response.sum = request.a + request.b

        return response
```

### Calling the Service from a Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AIClient(Node):
    def __init__(self):
        super().__init__('ai_client')
        self.client = self.create_client(AddTwoInts, 'ai_inference')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.send_request(5, 7)

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        result = future.result()
        self.get_logger().info(f'AI Inference Result: {result.sum}')
```

## Best Practices for AI-ROS Integration

### 1. Separate Computation from Communication
- Keep AI inference in separate threads/processes
- Use ROS 2's executor for efficient callback management

### 2. Handle Timing Constraints
```python
# Create a timer for periodic AI updates
self.timer = self.create_timer(0.1, self.ai_update_callback)  # 10 Hz
```

### 3. Optimize Performance
- Use `torch.jit` or ONNX for model optimization
- Leverage GPU acceleration when available
- Consider model quantization for edge deployment

### 4. Error Handling
```python
def safe_inference(self, data):
    try:
        result = self.model(data)
        return result
    except Exception as e:
        self.get_logger().error(f'Inference failed: {str(e)}')
        return None
```

## Real-World Example: Humanoid Grasping with AI

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class GraspingNode(Node):
    def __init__(self):
        super().__init__('grasping_ai_node')

        # Subscribe to hand camera
        self.camera_sub = self.create_subscription(
            Image,
            '/hand_camera/image',
            self.detect_grasp_pose,
            10
        )

        # Publish grasp commands
        self.grasp_pub = self.create_publisher(
            Float32MultiArray,
            '/grasp_pose',
            10
        )

    def detect_grasp_pose(self, msg):
        # Run AI model to detect optimal grasp pose
        # (Using a pretrained grasp detection network)

        # Convert image
        image = self.bridge.imgmsg_to_cv2(msg)

        # Run grasp detection AI
        grasp_poses = self.grasp_model.predict(image)

        # Publish best grasp
        if len(grasp_poses) > 0:
            best_grasp = grasp_poses[0]  # Highest confidence

            msg = Float32MultiArray()
            msg.data = [
                best_grasp.x, best_grasp.y, best_grasp.z,
                best_grasp.roll, best_grasp.pitch, best_grasp.yaw
            ]
            self.grasp_pub.publish(msg)
```

## Conclusion

Bridging Python-based AI agents with ROS 2 using `rclpy` unlocks powerful capabilities:
- Integrate state-of-the-art AI models into robotic systems
- Leverage Python's rich AI ecosystem while maintaining real-time performance
- Create modular, maintainable robot software architectures

**Next Steps:**
- Explore URDF for describing robot structures
- Learn about ROS 2 actions for long-running AI tasks
- Study parameter management for AI model configuration
