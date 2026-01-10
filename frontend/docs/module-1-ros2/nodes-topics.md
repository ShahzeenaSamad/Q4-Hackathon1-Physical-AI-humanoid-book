# Chapter 3: Nodes and Topics

**Module**: Module 1 - ROS 2 Fundamentals
**Estimated Time**: 2.5 hours (Reading: 45 min, Hands-on: 105 min)
**Prerequisites**: Chapter 2 (ROS 2 Architecture)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** custom ROS 2 nodes using the `rclpy` library.
2. **Design** a Publisher-Subscriber communication pattern for robotic sensor data.
3. **Analyze** message flow and node connectivity using `rqt_graph`.
4. **Utilize** standard message interfaces for common robotic data types.

**Bloom's Taxonomy Levels**: Apply (implement, design, create), Analyze (analyze), Remember (utilize)

---

## Introduction

In Chapter 2, we learned that ROS 2 is a distributed system of small, modular programs called **Nodes**. Now, we dive into the most fundamental way these nodes communicate: **Topics**.

Think of a "Topic" as a dedicated radio channel. One node (the **Publisher**) broadcasts a message on that channel, and any number of other nodes (the **Subscribers**) can tune in to listen. This decoupled nature is what allows roboticists to build complex "nervous systems" where components can be added or replaced without breaking the rest of the robot.

**Topics covered in this chapter**:
- The Python Node lifecycle using `rclpy`
- Implementing Publishers and Subscribers
- Understanding Message Types (msg)
- Debugging with `ros2 topic` and `rqt_graph`

**Why this matters**: This is the core mechanism for streaming data—from camera frames and LiDAR point clouds to motor feedback. Mastering topics is the first step toward building any autonomous robot behavior.

**Example Use Case**: A humanoid robot's IMU (Inertial Measurement Unit) publishes orientation data on a topic called `/imu/data`. Simultaneously, a balance controller node and a logging node both subscribe to that same topic—one to keep the robot upright, and one to record data for analysis.

---

## Core Content

### Section 1: The Python Node Lifecycle

In ROS 2 Python (`rclpy`), we design nodes using Object-Oriented Programming (OOP) by inheriting from the `Node` class. This provides a clean structure for managing timers, publishers, and subscribers.

#### Anatomy of a ROS 2 Python Node

```python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('my_custom_node')

        # Log an info message to the console
        self.get_logger().info('Node has been initialized')

def main(args=None):
    # 1. Initialize rclpy
    rclpy.init(args=args)

    # 2. Instantiate our node class
    node = MyCustomNode()

    # 3. Spin the node (keep it alive to process callbacks)
    rclpy.spin(node)

    # 4. Shutdown cleanly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Section 2: Publishers and Subscribers

#### 1. The Publisher
A Publisher is responsible for sending data. In Python, it requires three things:
- **Message Type**: The structure of the data (e.g., `String`, `Image`).
- **Topic Name**: The name of the channel.
- **Queue Size**: The number of messages to buffer if the receiver is slow (usually 10).

#### 2. The Subscriber
A Subscriber listens for data and triggers a **Callback Function** whenever a new message arrives.

---

### Section 3: Standard Messages (std_msgs)
Every topic must have a defined type. ROS 2 provides standard types in the `std_msgs` package:
- `String`: For text
- `Int32` / `Float64`: For numbers
- `Bool`: For True/False flags

More complex types exist in `sensor_msgs` (Images, LaserScans) and `geometry_msgs` (Twist, Pose).

---

## Hands-On Exercise

### Exercise 3: Creating a Robot Heartbeat System

**Objective**: Create a publisher-subscriber system where one node signals its "heartbeat" (status) and another node monitors it.

**Estimated Time**: 105 minutes

#### Step 1: Create the Package
First, create a workspace and a new Python package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python heartbeat_system --dependencies rclpy std_msgs
```

#### Step 2: Write the Heartbeat Publisher
Create `heartbeat_system/heartbeat_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        # Create publisher: (Type, Topic Name, Queue Size)
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)

        # Set a timer to publish every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Heartbeat Status: OK (Index: {self.count})'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HeartbeatPublisher())
    rclpy.shutdown()
```

#### Step 3: Write the Monitor Subscriber
Create `heartbeat_system/monitor_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MonitorSubscriber(Node):
    def __init__(self):
        super().__init__('monitor_subscriber')
        # Create subscription: (Type, Topic Name, Callback, Queue Size)
        self.subscription = self.create_subscription(
            String, 'heartbeat', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Monitor received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MonitorSubscriber())
    rclpy.shutdown()
```

#### Step 4: Build and Verify
1.  Update `setup.py` entry points.
2.  `colcon build` and `source install/setup.bash`.
3.  Run both nodes and use `rqt_graph` to see the connection.

---

## Summary

**Key Concepts**:
1.  **Publisher-Subscriber**: A one-way, asynchronous communication pattern.
2.  **rclpy**: The Python library for creating ROS 2 nodes.
3.  **Spinning**: The process that keeps nodes alive to handle incoming data or timer events.

**Skills Acquired**:
- Writing inheritance-based ROS 2 nodes in Python.
- Creating timers for periodic data publishing.
- Using callbacks to process incoming topic data.

---

## Assessment Questions

**Question 1** (Difficulty: Easy)
Which `rclpy` function is required to keep a node running and processing events?
- A) `node.start()`
- B) `rclpy.spin(node)`
- C) `node.loop()`

**Question 2** (Difficulty: Medium)
Explain what happens if a node publishes to a topic but no nodes are currently subscribed.

**Question 3** (Difficulty: Hard)
If you need to send orientation data from a sensor at 100Hz, would you use a Topic or a Service? Justify your choice based on the characteristics of the data.

---

## Next Chapter

Continue to **[Chapter 4: Services and Actions](/docs/module-1-ros2/services-actions)**.

---

**Revision History**:
- **Version 1.0** (2025-12-31): Initial release.
