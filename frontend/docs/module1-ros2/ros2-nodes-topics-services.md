---
sidebar_position: 2
title: ROS 2 Nodes, Topics, and Services
---

# ROS 2 Nodes, Topics, and Services

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is an open-source middleware framework for building robot applications. It provides the communication infrastructure that allows different parts of a robot system to work together seamlessly.

### Why ROS 2?

- **Industry Standard**: Used by companies like Boston Dynamics, NASA, and Tesla
- **Language Agnostic**: Write nodes in Python, C++, or other languages
- **Distributed Architecture**: Run components on multiple machines
- **Real-Time Capable**: Deterministic communication for time-critical tasks
- **Cross-Platform**: Works on Linux, Windows, and macOS

## Core Concepts: The Building Blocks

### 1. Nodes: Independent Processes

A **node** is an independent process that performs a specific computation. Think of nodes as microservices in a robotic system.

**Example Nodes:**
- `camera_driver`: Captures images from a camera
- `object_detector`: Identifies objects in images
- `motion_planner`: Plans robot movements
- `motor_controller`: Sends commands to motors

**Key Characteristics:**
- Each node runs independently
- Nodes can be started, stopped, or restarted without affecting others
- One node crash won't bring down the entire system
- Nodes communicate via topics, services, and actions

### 2. Topics: Publish-Subscribe Communication

**Topics** are named channels for one-way data streams. Nodes **publish** messages to topics, and other nodes **subscribe** to receive those messages.

**Use Case**: Streaming sensor data (camera frames, LIDAR scans, IMU readings)

```python
# Publisher Example (Python)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        msg = Image()
        # Fill in image data
        self.publisher.publish(msg)
        self.get_logger().info('Published image')
```

```python
# Subscriber Example (Python)
class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        # Process the image
```

**Topic Characteristics:**
- **Asynchronous**: Publishers don't wait for subscribers
- **Many-to-Many**: Multiple publishers and subscribers per topic
- **Fire-and-Forget**: No acknowledgment of receipt
- **Best for**: High-frequency data streams

### 3. Services: Request-Response Communication

**Services** provide synchronous request-response communication. A client sends a request to a service, and waits for a response.

**Use Case**: Triggering actions or querying information (e.g., "Start recording", "Get current pose")

```python
# Service Server Example
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

**Service Characteristics:**
- **Synchronous**: Client waits for response
- **One-to-One**: One service server per service name
- **Guaranteed Response**: Server must respond (or timeout occurs)
- **Best for**: Infrequent, critical operations

## Practical Exercise: Building Your First ROS 2 System

Let's create a simple "talker-listener" system:

```python
# Talker (Publisher)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello from ROS 2! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1
```

## Key Takeaways

- **Nodes** are independent processes that perform specific tasks
- **Topics** enable asynchronous many-to-many communication (best for streaming data)
- **Services** provide synchronous request-response communication (best for commands)
- ROS 2 architecture promotes modularity, reusability, and fault tolerance
- Quality of Service (QoS) policies allow fine-tuning of communication behavior

---

**Next**: We'll learn how to bridge Python AI agents to ROS 2 controllers using `rclpy`.
