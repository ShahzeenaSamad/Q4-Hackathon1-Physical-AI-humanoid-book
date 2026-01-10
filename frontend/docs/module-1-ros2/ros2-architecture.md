# Chapter 2: ROS 2 Architecture and Core Concepts

**Module**: Module 1 - ROS 2 Fundamentals
**Estimated Time**: 2.0 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Chapter 1 (Introduction to Physical AI), ROS 2 Humble installed, basic understanding of distributed systems

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** the architectural differences between ROS 1 and ROS 2, including DDS middleware
2. **Identify** the core components of a ROS 2 system: nodes, topics, services, actions, and parameters
3. **Analyze** the ROS 2 computation graph to understand node communication patterns
4. **Create** multi-node ROS 2 systems with publisher-subscriber communication
5. **Evaluate** quality of service (QoS) policies for different robotics applications

**Bloom's Taxonomy Levels**: Understand (explain), Remember (identify), Analyze, Apply (create), Evaluate

---

## Introduction

The Robot Operating System 2 (ROS 2) is built on a fundamentally different architecture than its predecessor ROS 1. Understanding this architecture is crucial for building reliable, real-time robotic systems. Unlike ROS 1's centralized master node, ROS 2 uses a peer-to-peer discovery mechanism powered by Data Distribution Service (DDS), enabling truly distributed systems that can operate across multiple machines, survive network partitions, and support real-time control.

This chapter explores ROS 2's layered architecture, from the DDS middleware layer through the client libraries (rclpy/rclcpp) to the high-level computation graph of nodes and topics. You'll learn how nodes discover each other, how messages flow through the system, and how quality of service policies ensure reliable communication in challenging robotic environments.

**Topics covered in this chapter**:
- ROS 1 vs ROS 2 architectural comparison
- DDS middleware and its role in distributed robotics
- The ROS 2 computation graph (nodes, topics, services, actions)
- Quality of Service (QoS) policies for reliability
- Introspection tools for debugging ROS 2 systems

**Why this matters**: Choosing between pub-sub topics vs request-response services affects system responsiveness. QoS policies determine whether sensor data survives network hiccups or gets dropped. Understanding the architecture helps you design systems that are both performant and maintainable—critical when debugging a 20-node autonomous robot in production.

**Example Use Case**: A self-driving car has 50+ ROS 2 nodes: camera drivers, LiDAR processing, localization, path planning, and motor control. Each must communicate with precise timing and reliability guarantees. The architecture you learn here is what makes that coordination possible.

---

## Core Content

### Section 1: The ROS 2 Layered Architecture

ROS 2 is organized into several layers, allowing it to be hardware-agnostic and language-flexible:

1.  **OS Layer**: Typically Ubuntu Linux, but ROS 2 also supports Windows 10/11, macOS, and Real-Time Operating Systems (RTOS).
2.  **DDS (Data Distribution Service)**: The middleware layer that provides discovery, serialization, and transport. This replaces the `roscore` (master) from ROS 1.
3.  **RMW (ROS Middleware Interface)**: A thin abstraction layer that allows different DDS implementations (e.g., eProsima Fast DDS, Cyclone DDS) to be swapped out without changing your code.
4.  **RCL (ROS Client Library)**: The core C-based logic for timers, logging, and communication.
5.  **rclpy / rclcpp**: Language-specific libraries that we use to write our robot applications.

#### Key Concept: Discovery
In ROS 2, there is no central server. When a node starts, it broadcasts its presence on the network. Other nodes listen for these broadcasts and automatically establish connections. This is called **Discovery**.

---

### Section 2: The Computation Graph

The "Graph" is the network of ROS 2 elements processing data at any given time.

#### 1. Nodes
A **Node** is a single-purpose process. A complex robot might have hundreds of nodes working together. Reducing code into small nodes increases modularity and fault tolerance.

#### 2. Topics (Publish/Subscribe)
**Topics** are used for continuous streams of data (e.g., sensor data, motor commands).
- **Many-to-Many**: Multiple nodes can publish or subscribe to the same topic.
- **Asynchronous**: The publisher sends data and doesn't wait for the subscriber to receive it.

#### 3. Services (Request/Response)
**Services** are for "call and response" interactions (e.g., triggering a camera capture, calculating an IK solution).
- **Synchronous/Asynchronous**: The client sends a request and waits for a response from the server.
- **One-to-One**: A service has exactly one server, though multiple clients can call it.

#### 4. Actions (Goal/Feedback/Result)
**Actions** are for long-running tasks (e.g., "navigate to the kitchen", "pick up the box").
- **Feedback**: Provides continuous updates while the task is running.
- **Progressive**: Can be canceled or preempted.

---

### Section 3: Quality of Service (QoS)

ROS 2 allows you to tune how data is handled. This is critical for physical AI where network reliability varies.

| Policy | Description | Use Case |
| :--- | :--- | :--- |
| **Reliability** | **Reliable**: Re-send if lost / **Best Effort**: Send once | Control commands vs. high-bandwidth Video |
| **Durability** | **Volatile**: No history / **Transient Local**: Late joiners get last msg | Sensor data vs. Map data |
| **History** | **Keep Last N**: Only newest / **Keep All**: Buffer everything | Real-time control vs. Logging |

---

## Hands-On Exercise

### Exercise 2: Introspecting the Computation Graph

**Objective**: Run multiple nodes and use command-line tools to visualize and debug the communication graph.

**Estimated Time**: 75 minutes

#### Step 1: Launch Turtlesim
Open a terminal and run the simulator:
```bash
ros2 run turtlesim turtlesim_node
```

#### Step 2: Use Introspection Tools
Open a second terminal and explore the running system:

```bash
# List all active nodes
ros2 node list

# List all active topics
ros2 topic list

# See the type of message on a topic
ros2 topic info /turtle1/cmd_vel

# See the data flowing in real-time
ros2 topic echo /turtle1/pose
```

#### Step 3: Run the Teleop Node
Open a third terminal to control the turtle:
```bash
ros2 run turtlesim turtle_teleop_key
```

#### Step 4: Visualize with RQT Graph
ROS 2 includes a powerful tool called `rqt_graph` to see the live network:
```bash
rqt_graph
```
In the window that opens, you will see the `/teleop_turtle` node publishing to the `/turtlesim` node via the `/turtle1/cmd_vel` topic.

---

## Summary

**Key Concepts Covered**:
1.  **DDS Middleware**: Enables decentralized, peer-to-peer discovery and communication.
2.  **Computation Graph**: The map of how nodes, topics, and services interact.
3.  **QoS Policies**: Configuration that balances reliability and performance for different data types.
4.  **Introspection**: Tools like `ros2 node`, `ros2 topic`, and `rqt_graph` are essential for debugging physical AI systems.

**Skills Acquired**:
- Navigating the ROS 2 command-line interface.
- Analyzing real-time data flows between nodes.
- Visualizing complex multi-node systems.

**Connection to Next Chapter**: Now that we understand the overall architecture, **Chapter 3: Nodes and Topics** will dive deep into writing the Python code to create your own custom publishers and subscribers.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
In ROS 2, what replaces the ROS 1 "Master" (`roscore`)?
- A) The Global Launcher
- B) DDS Discovery
- C) The Primary Node
- D) Systemd

**Question 2** (Difficulty: Medium)
When should you use a **Service** instead of a **Topic**?
- A) When sending a continuous video stream.
- B) When you need a one-time response to a specific request.
- C) When multiple nodes need to hear the same data.
- D) When the data is low-priority.

**Question 3** (Difficulty: Hard)
Explain a scenario where **Best Effort** reliability is better than **Reliable** for a physical AI robot.

---

## Next Chapter

Continue to **[Chapter 3: Nodes and Topics](/docs/module-1-ros2/nodes-topics)**.

---

**Revision History**:
- **Version 1.0** (2025-12-31): Initial release.
