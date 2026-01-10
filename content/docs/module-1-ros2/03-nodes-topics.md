# Chapter 3: ROS 2 Nodes and Topics

**Module**: Module 1: ROS 2 Fundamentals
**Estimated Time**: 2.5 hours (Reading: 45 min, Hands-on: 105 min)
**Prerequisites**: Chapter 2: Introduction to ROS 2

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** custom ROS 2 nodes using the `rclpy` library.
2. **Design** a Publisher-Subscriber communication pattern for robotic sensor data.
3. **Analyze** message flow and node connectivity using `rqt_graph`.
4. **Create** custom message interfaces for specialized robotic data.

---

## Introduction

In Chapter 2, we learned that ROS 2 is a federated system of small programs called **Nodes**. But how do these nodes actually talk to each other? The most fundamental way is through **Topics**.

Think of a "Topic" as a radio frequency. One node (the **Publisher**) broadcasts a message on that frequency, and any number of other nodes (the **Subscribers**) can tune in to listen.

**Topics covered in this chapter**:
- The Node lifecycle in Python
- Topic communication (Publishers and Subscribers)
- Standard vs. Custom Messages
- Debugging with `rqt_graph` and `ros2 topic`

**Why this matters**: This is the "nervous system" of your robot. Without topics, your robot's camera wouldn't be able to tell the motors to stop when it sees an obstacle. Mastering topics is the first step toward building autonomous systems.

**Example Use Case**: A humanoid robot's IMU (Inertial Measurement Unit) publishes orientation data on a topic called `/imu_data`. The balance controller node subscribes to this topic to adjust the robot's ankles and stay upright.

---

## Core Content

### Section 1: The Python Node Lifecycle

In ROS 2 Python (`rclpy`), we create nodes by inheriting from the `Node` class. This allows us to use timers, publishers, and subscribers in an object-oriented way.

#### Example 3.1: A Basic ROS 2 Node (Template)

```python
# Environment: Ubuntu 22.04 + ROS 2 Humble + Python 3.10
# File: basic_node.py

import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node') # Initialize node with name
        self.get_logger().info('Simple Node has been started!')

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2
    node = SimpleNode()
    rclpy.spin(node) # Keep node running
    rclpy.shutdown() # Cleanup on exit

if __name__ == '__main__':
    main()
```

---

### Section 2: Publishers and Subscribers

Topics use a **many-to-many** communication model. It is anonymous: the publisher doesn't know who is listening, and the subscriber doesn't know who is talking.

#### Key Points
- **Unidirectional**: Topics are meant for a stream of data in one direction.
- **Asynchronous**: The sender doesn't wait for a response.
- **Data Types**: Every topic must have a specific message type (e.g., `String`, `Int32`, `Image`).

#### Visual Aid

![Figure 3.1: Publisher-Subscriber Pattern](../../../static/img/module-1/pub-sub-diagram.png)

*Figure 3.1: A camera node publishing data that is consumed by both a remote viewer and a local obstacle avoidance node.*

---

## Hands-On Exercise

### Exercise 3: Building a Robot Heartbeat System

In this exercise, you will create two nodes: a **Heartbeat Publisher** that broadcasts the robot's status and a **Status Monitor** that listens and logs the data.

**Objective**: Master the creation of publishers and subscribers in Python.

**Estimated Time**: 105 minutes

#### Step 1: Create a Package

Navigate to your workspace and create a new Python package.

```bash
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python robot_status_pkg --dependencies rclpy std_msgs
```

#### Step 2: Write the Heartbeat Publisher

Create `robot_status_pkg/heartbeat_publisher.py`:

```python
# File: heartbeat_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_heartbeat', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot Active: Heartbeat {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

#### Step 3: Write the Status Monitor (Subscriber)

Create `robot_status_pkg/status_monitor.py`:

```python
# File: status_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        self.subscription = self.create_subscription(
            String,
            'robot_heartbeat',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

#### Step 4: Build and Run

1. Add entry points to `setup.py`.
2. Build the workspace.
3. Run both nodes in separate terminals and check `rqt_graph`.

```bash
colcon build --packages-select robot_status_pkg
source install/setup.bash
# Terminal 1:
ros2 run robot_status_pkg heartbeat_cmd
# Terminal 2:
ros2 run robot_status_pkg monitor_cmd
```

**Success Criteria**:
- [ ] Publisher logs "Publishing: Heartbeat X" every second.
- [ ] Subscriber logs "I heard: Heartbeat X" instantly.
- [ ] `rqt_graph` shows a line connecting the two nodes.

---

## Summary

**Key Concepts Covered**:
1. **Nodes**: Independent computational processes.
2. **Topics**: Named channels for data broadcast.
3. **rqt_graph**: The "X-ray" for your robot's communication.

**Skills Acquired**:
- Writing `rclpy` class-based nodes.
- Setting up a Publisher with a Timer.
- Setting up a Subscriber with a Callback.

**Connection to Next Chapter**: Now that we can stream data, what if we need a direct response? In **Chapter 4: ROS 2 Services and Actions**, we'll learn about Request-Response and long-running feedback loops.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which CLI command shows the list of active topics?
A) `ros2 node list`
B) `ros2 topic list`
C) `ros2 message list`

**Question 2** (Difficulty: Medium)
Explain why the "Queuesize" (the `10` in our publisher/subscriber setup) is important for a robot with a slow internet connection.

**Question 3** (Difficulty: Hard)
If two nodes publish on the same topic `/sensor_data`, what will a subscriber to that topic see?

---

## Next Chapter

Continue to **[Chapter 4: ROS 2 Services and Actions](./04-services-actions.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
