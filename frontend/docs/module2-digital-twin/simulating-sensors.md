---
sidebar_position: 3
title: Simulating Sensors (LIDAR, Depth Cameras, IMUs)
---

# Simulating Sensors (LIDAR, Depth Cameras, IMUs)

## Introduction

Autonomous humanoid robots rely on sensors to perceive their environment. Before deploying expensive hardware, simulation allows us to test sensor configurations, develop perception algorithms, and generate training data. This chapter covers the three essential sensor types for humanoid robotics:

1. **LiDAR** (Light Detection and Ranging): 3D mapping and obstacle detection
2. **Depth Cameras** (RGB-D): Close-range manipulation and object recognition
3. **IMU** (Inertial Measurement Unit): Balance and orientation tracking

## Why Sensor Simulation Matters

**Benefits of Simulated Sensors:**

- **Cost Savings**: Test configurations before purchasing hardware
- **Safety**: Develop algorithms without risking real robots
- **Data Generation**: Create labeled datasets for machine learning
- **Sim-to-Real Transfer**: Algorithms trained in simulation work on real robots
- **Rapid Iteration**: Change sensor placements instantly

**Challenges:**

- **Sensor Noise Modeling**: Real sensors have noise; simulations must replicate it
- **Performance**: Ray-tracing LiDAR with millions of points is computationally expensive
- **Accuracy**: Physics engines approximate reality

## LiDAR Simulation

### Understanding LiDAR Technology

LiDAR measures distances by timing laser pulses reflected from surfaces. Common types:

| Type | Range | Points/Second | Use Case |
|------|-------|---------------|----------|
| **2D LiDAR** | 30m | 5,000-15,000 | Floor-level obstacle detection |
| **3D LiDAR** | 100m+ | 300,000-2M | Full 3D mapping (Velodyne, Ouster) |
| **Solid-State LiDAR** | 200m | 1M+ | Automotive (no moving parts) |

For humanoid robots, **3D LiDAR** mounted at head height provides:
- 360° environmental awareness
- Stair/obstacle detection for navigation
- Human detection and tracking

### Simulating LiDAR in Gazebo

Gazebo's `gpu_ray` sensor efficiently simulates LiDAR using GPU ray-tracing.

**Example: Velodyne VLP-16 LiDAR Simulation**

```xml
<!-- Add to your robot's URDF/SDF file -->
<gazebo reference="lidar_link">
  <sensor name="velodyne" type="gpu_ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>

    <ray>
      <!-- 16 vertical lasers (VLP-16 spec) -->
      <scan>
        <horizontal>
          <samples>1800</samples>  <!-- 0.2° resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180° -->
          <max_angle>3.14159</max_angle>   <!-- +180° -->
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2617</min_angle>  <!-- -15° -->
          <max_angle>0.2617</max_angle>   <!-- +15° -->
        </vertical>
      </scan>

      <!-- Range specifications -->
      <range>
        <min>0.3</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>

      <!-- Simulate sensor noise -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>  <!-- 2cm standard deviation -->
      </noise>
    </ray>

    <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_velodyne_laser.so">
      <topicName>/velodyne_points</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### Processing LiDAR Data in ROS 2

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to LiDAR point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10
        )

        # Publisher for processed obstacles
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/obstacles',
            10
        )

    def lidar_callback(self, msg):
        # Convert ROS PointCloud2 to numpy array
        points = self.pointcloud2_to_array(msg)

        # Filter ground plane (assume flat floor at z=0)
        non_ground = points[points[:, 2] > 0.1]  # Points above 10cm

        # Detect obstacles within 2 meters
        distances = np.linalg.norm(non_ground[:, :2], axis=1)
        obstacles = non_ground[distances < 2.0]

        # Cluster obstacles (simple spatial clustering)
        clusters = self.dbscan_clustering(obstacles)

        self.get_logger().info(f'Detected {len(clusters)} obstacles')

        # Publish obstacle point cloud
        obstacle_msg = self.array_to_pointcloud2(obstacles, msg.header)
        self.obstacle_pub.publish(obstacle_msg)

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        points_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list)

    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2 message"""
        return pc2.create_cloud_xyz32(header, points)

    def dbscan_clustering(self, points, eps=0.3, min_samples=10):
        """Simple DBSCAN clustering for obstacle grouping"""
        from sklearn.cluster import DBSCAN

        if len(points) == 0:
            return []

        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points[:, :2])
        labels = clustering.labels_

        # Group points by cluster
        clusters = []
        for label in set(labels):
            if label == -1:  # Noise
                continue
            cluster_points = points[labels == label]
            clusters.append(cluster_points)

        return clusters

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visualizing LiDAR Data in RViz2

```bash
# Launch RViz2
ros2 run rviz2 rviz2

# Add PointCloud2 display
# Topic: /velodyne_points
# Fixed Frame: base_link
# Color Transformer: AxisColor (colors by height)
```

## Depth Camera Simulation (RGB-D)

### RGB-D Camera Technology

Depth cameras provide both color images and depth information. Technologies:

| Technology | Example | Range | Indoor/Outdoor |
|------------|---------|-------|----------------|
| **Structured Light** | Kinect v1 | 0.5-4m | Indoor only |
| **Time-of-Flight** | Kinect Azure | 0.25-5m | Indoor/Outdoor |
| **Stereo Vision** | Intel RealSense | 0.1-10m | Both |
| **LiDAR-based** | iPad Pro | 0-5m | Both |

**Humanoid Use Cases:**
- Object grasping (see depth for grip planning)
- Facial recognition (RGB + depth for 3D face models)
- Gesture detection (track hand depth for interaction)

### Simulating RGB-D in Gazebo

```xml
<!-- Intel RealSense D435 simulation -->
<gazebo reference="camera_link">
  <!-- RGB Camera -->
  <sensor name="realsense_color" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.211</horizontal_fov>  <!-- 69.4° -->
      <image>
        <width>1920</width>
        <height>1080</height>
        <format>RGB8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/realsense</namespace>
        <remapping>image_raw:=color/image_raw</remapping>
        <remapping>camera_info:=color/camera_info</remapping>
      </ros>
      <camera_name>color</camera_name>
      <frame_name>camera_color_optical_frame</frame_name>
    </plugin>
  </sensor>

  <!-- Depth Camera -->
  <sensor name="realsense_depth" type="depth">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.487</horizontal_fov>  <!-- 85.2° -->
      <image>
        <width>1280</width>
        <height>720</height>
        <format>R_FLOAT32</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="depth_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/realsense</namespace>
        <remapping>image_raw:=depth/image_raw</remapping>
        <remapping>camera_info:=depth/camera_info</remapping>
      </ros>
      <camera_name>depth</camera_name>
      <frame_name>camera_depth_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Object Detection with RGB-D

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class RGBDObjectDetector(Node):
    def __init__(self):
        super().__init__('rgbd_object_detector')

        self.bridge = CvBridge()

        # Subscribe to RGB and Depth images
        self.rgb_sub = self.create_subscription(
            Image,
            '/realsense/color/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/realsense/depth/image_raw',
            self.depth_callback,
            10
        )

        self.latest_rgb = None
        self.latest_depth = None

        # Timer for synchronized processing
        self.timer = self.create_timer(0.1, self.process_rgbd)

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def process_rgbd(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        # Detect objects by color (example: red objects)
        hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)

        # Red color range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 500:  # Filter small noise
                continue

            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)

            # Get depth at object center
            center_x, center_y = x + w//2, y + h//2
            depth = self.latest_depth[center_y, center_x]

            # Draw on image
            cv2.rectangle(self.latest_rgb, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(
                self.latest_rgb,
                f'{depth:.2f}m',
                (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            self.get_logger().info(f'Red object detected at {depth:.2f}m')

        # Display result
        cv2.imshow('Object Detection', self.latest_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RGBDObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3D Point Cloud from RGB-D

Convert depth images to 3D point clouds for manipulation planning:

```python
import numpy as np

def depth_to_pointcloud(depth_image, camera_info):
    """
    Convert depth image to 3D point cloud

    Args:
        depth_image: HxW numpy array of depth values (meters)
        camera_info: CameraInfo message with intrinsics

    Returns:
        Nx3 numpy array of 3D points
    """
    h, w = depth_image.shape

    # Camera intrinsic matrix
    fx = camera_info.k[0]  # Focal length x
    fy = camera_info.k[4]  # Focal length y
    cx = camera_info.k[2]  # Principal point x
    cy = camera_info.k[5]  # Principal point y

    # Create meshgrid of pixel coordinates
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Convert to 3D coordinates
    z = depth_image
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Stack into Nx3 array
    points = np.stack([x, y, z], axis=-1)
    points = points.reshape(-1, 3)

    # Remove invalid points (zero depth)
    valid = points[:, 2] > 0
    points = points[valid]

    return points

# Example usage
camera_info = get_camera_info()  # From /realsense/depth/camera_info
points_3d = depth_to_pointcloud(depth_image, camera_info)
```

## IMU Simulation (Inertial Measurement Unit)

### IMU Fundamentals

IMUs measure:
1. **Linear Acceleration** (accelerometer): 3-axis, includes gravity
2. **Angular Velocity** (gyroscope): 3-axis rotation rates
3. **Magnetic Field** (magnetometer): 3-axis compass heading

**Critical for Humanoid Robots:**
- **Balance Control**: Detect falls before they happen
- **Gait Stabilization**: Adjust walking pattern based on tilt
- **Pose Estimation**: Combine with joint encoders for full-body state

### Simulating IMU in Gazebo

```xml
<!-- IMU sensor in robot torso -->
<gazebo reference="torso_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>

    <imu>
      <!-- Noise parameters based on real IMU specs (e.g., MPU-6050) -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>  <!-- 0.009 rad/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- 0.017 m/s² -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Fall Detection with IMU

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class FallDetector(Node):
    def __init__(self):
        super().__init__('fall_detector')

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Thresholds for fall detection
        self.tilt_threshold = 30.0  # degrees
        self.acceleration_threshold = 20.0  # m/s² (high impact)

    def imu_callback(self, msg):
        # Extract orientation (quaternion to Euler)
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # Check if robot is tilted beyond safe angle
        tilt_angle = np.degrees(max(abs(roll), abs(pitch)))

        if tilt_angle > self.tilt_threshold:
            self.get_logger().warn(f'Tilt detected: {tilt_angle:.1f}°')
            self.execute_fall_protection()

        # Check for high impact (already fallen)
        accel = msg.linear_acceleration
        accel_magnitude = np.sqrt(accel.x**2 + accel.y**2 + accel.z**2)

        if accel_magnitude > self.acceleration_threshold:
            self.get_logger().error('High impact detected! Robot may have fallen.')
            self.execute_emergency_stop()

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def execute_fall_protection(self):
        """Protective motions when fall is imminent"""
        self.get_logger().info('Executing fall protection sequence...')
        # - Extend arms to catch fall
        # - Bend knees to lower center of mass
        # - Activate emergency motors to stiffen joints

    def execute_emergency_stop(self):
        """Cut power to motors after fall"""
        self.get_logger().info('Emergency stop activated')
        # - Stop all motor commands
        # - Log fall event
        # - Request human assistance

def main(args=None):
    rclpy.init(args=args)
    node = FallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Sensor Fusion: Complementary Filter

Combine accelerometer and gyroscope data for accurate orientation:

```python
class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter')

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Filter parameters
        self.alpha = 0.98  # Weight for gyroscope (fast but drifts)
        self.beta = 0.02   # Weight for accelerometer (slow but accurate)

        # State
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Gyroscope-based estimate (integrate angular velocity)
        gyro_roll = self.roll + msg.angular_velocity.x * dt
        gyro_pitch = self.pitch + msg.angular_velocity.y * dt

        # Accelerometer-based estimate
        accel_roll = np.arctan2(msg.linear_acceleration.y, msg.linear_acceleration.z)
        accel_pitch = np.arctan2(-msg.linear_acceleration.x,
                                  np.sqrt(msg.linear_acceleration.y**2 +
                                          msg.linear_acceleration.z**2))

        # Complementary filter (weighted average)
        self.roll = self.alpha * gyro_roll + self.beta * accel_roll
        self.pitch = self.alpha * gyro_pitch + self.beta * accel_pitch

        self.get_logger().info(
            f'Orientation - Roll: {np.degrees(self.roll):.1f}°, '
            f'Pitch: {np.degrees(self.pitch):.1f}°'
        )
```

## Multi-Sensor Fusion

### Kalman Filter for Sensor Integration

Combine LiDAR, IMU, and wheel odometry for robust localization:

```python
import numpy as np

class ExtendedKalmanFilter:
    def __init__(self):
        # State vector: [x, y, theta, v_x, v_y, omega]
        self.state = np.zeros(6)

        # Covariance matrix (uncertainty)
        self.P = np.eye(6) * 0.1

        # Process noise
        self.Q = np.eye(6) * 0.01

        # Measurement noise
        self.R_imu = np.eye(3) * 0.1     # IMU: [theta, omega, accel]
        self.R_odom = np.eye(3) * 0.05   # Odometry: [x, y, theta]
        self.R_lidar = np.eye(2) * 0.02  # LiDAR landmarks: [x, y]

    def predict(self, dt):
        """Predict next state based on motion model"""
        x, y, theta, vx, vy, omega = self.state

        # State transition (constant velocity model)
        self.state[0] += vx * dt  # x
        self.state[1] += vy * dt  # y
        self.state[2] += omega * dt  # theta

        # Jacobian of motion model
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_imu(self, imu_data):
        """Update with IMU measurement"""
        # Measurement: [theta, omega, accel]
        z = np.array([imu_data.orientation_z, imu_data.angular_velocity_z, imu_data.linear_acceleration_x])

        # Measurement model
        H = np.array([
            [0, 0, 1, 0, 0, 0],  # theta
            [0, 0, 0, 0, 0, 1],  # omega
            [0, 0, 0, 1, 0, 0]   # accel → vx
        ])

        # Kalman gain
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        innovation = z - H @ self.state
        self.state += K @ innovation

        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_lidar(self, landmark_x, landmark_y):
        """Update with LiDAR landmark detection"""
        z = np.array([landmark_x, landmark_y])

        H = np.array([
            [1, 0, 0, 0, 0, 0],  # x
            [0, 1, 0, 0, 0, 0]   # y
        ])

        S = H @ self.P @ H.T + self.R_lidar
        K = self.P @ H.T @ np.linalg.inv(S)

        innovation = z - H @ self.state
        self.state += K @ innovation
        self.P = (np.eye(6) - K @ H) @ self.P
```

## Performance Optimization

### Reducing Sensor Computational Load

```python
class SensorOptimizer(Node):
    def __init__(self):
        super().__init__('sensor_optimizer')

        # Adaptive update rates based on robot state
        self.robot_state = 'stationary'  # or 'walking', 'running'

    def adjust_sensor_rates(self):
        if self.robot_state == 'stationary':
            # Low update rates when not moving
            self.set_lidar_rate(5)  # 5 Hz
            self.set_camera_rate(10)  # 10 Hz
            self.set_imu_rate(50)  # 50 Hz

        elif self.robot_state == 'walking':
            # Medium rates for walking
            self.set_lidar_rate(10)
            self.set_camera_rate(15)
            self.set_imu_rate(100)

        elif self.robot_state == 'running':
            # High rates for dynamic motion
            self.set_lidar_rate(20)
            self.set_camera_rate(30)
            self.set_imu_rate(200)
```

## Generating Synthetic Training Data

### Automated Data Collection in Simulation

```python
class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        self.rgb_sub = self.create_subscription(Image, '/camera/rgb', self.save_rgb, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth', self.save_depth, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar', self.save_lidar, 10)

        self.dataset_path = '/home/user/datasets/humanoid_sim/'
        self.frame_count = 0

    def save_rgb(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        filename = f'{self.dataset_path}/rgb/frame_{self.frame_count:06d}.png'
        cv2.imwrite(filename, rgb)

    def save_depth(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        filename = f'{self.dataset_path}/depth/frame_{self.frame_count:06d}.npy'
        np.save(filename, depth)

    def save_lidar(self, msg):
        points = self.pointcloud2_to_array(msg)
        filename = f'{self.dataset_path}/lidar/frame_{self.frame_count:06d}.npy'
        np.save(filename, points)

        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Collected {self.frame_count} frames')
```

## Summary

Sensor simulation is essential for developing robust humanoid robots:

**Key Takeaways:**

1. **LiDAR**: Use `gpu_ray` in Gazebo for efficient 3D mapping
2. **RGB-D Cameras**: Combine color and depth for object manipulation
3. **IMU**: Critical for balance and fall detection
4. **Sensor Fusion**: Combine multiple sensors with Kalman filtering
5. **Synthetic Data**: Generate training datasets in simulation

**Best Practices:**

- Always add realistic noise to simulated sensors
- Tune noise parameters to match real hardware specs
- Use sensor fusion instead of relying on single sensors
- Optimize update rates based on robot state
- Validate sim-to-real transfer with real hardware

By mastering sensor simulation, you can develop perception systems that work reliably when deployed on physical humanoid robots.
