# Chapter 2: Introduction to ROS 2

**Module**: Module 1: ROS 2 Fundamentals
**Estimated Time**: 2.5 hours (Reading: 60 min, Hands-on: 90 min)
**Prerequisites**: Chapter 1: Introduction to Physical AI

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** the federated architecture of ROS 2 and its core design goals.
2. **Install** ROS 2 Humble Hawksbill on Ubuntu 22.04.
3. **Configure** a ROS 2 workspace and manage packages using `colcon`.
4. **Utilize** command-line tools like `ros2 run`, `ros2 node list`, and `ros2 topic list`.

---

## Introduction

In the previous chapter, we explored the concept of Physical AI. To build such systems, we need a robust communication layer that connects sensors (eyes), controllers (brain), and actuators (muscles). This is where **ROS 2 (Robot Operating System 2)** comes in.

Despite its name, ROS 2 is not a traditional operating system like Windows or Linux. It is a **middleware** framework—a set of software libraries and tools that help you build robot applications.

**Topics covered in this chapter**:
- The "Why" of ROS 2: Federated systems and DDS
- Installation walkthrough for ROS 2 Humble
- The `colcon` build system and workspace layout
- Running your first ROS 2 nodes

**Why this matters**: ROS 2 is the industry standard for robotics. Companies like Tesla, NVIDIA, and Boston Dynamics use ROS 2 (or custom versions of it) because it allows them to build complex systems where different developers can work on different parts of the robot independently.

**Example Use Case**: In a humanoid robot, one node might be responsible for processing camera images to detect obstacles, while another node calculates the balancing torques for the legs. ROS 2 allows these two nodes to talk to each other reliably and with low latency.

---

## Core Content

### Section 1: ROS 2 Architecture

ROS 2 is designed to be a distributed, peer-to-peer system. Instead of one massive program, a robot is composed of many small programs called **Nodes**.

#### Key Points
- **Nodes**: A process that performs computation. Each node should do one thing (e.g., one node for the camera, one for the lidar, one for the motor).
- **DDS (Data Distribution Service)**: The underlying communication protocol that handles discovery, serialization, and transport within ROS 2.
- **RCL (ROS Client Library)**: The interface that allows us to write code in different languages (`rclpy` for Python, `rclcpp` for C++).

---

### Section 2: Workspaces and Colcon

A **Workspace** is a directory where you develop your ROS 2 packages. We use the `colcon` tool to build these packages.

The standard layout for a workspace is:
```text
dev_ws/
  ├── src/                # Your source code / packages
  ├── build/              # Intermediate build files
  ├── install/            # Where build artifacts are installed
  └── log/                # Build logs
```

---

## Hands-On Exercise

### Exercise 2: Installing and Running ROS 2

**Objective**: Install ROS 2 Humble and run the `turtlesim` demo to verify the installation.

**Estimated Time**: 90 minutes

#### Step 1: Set Locale

Make sure you have a locale that supports `UTF-8`.

```bash
# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### Step 2: Setup Sources

Add the ROS 2 GPG key and repository.

```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Step 3: Install ROS 2 Packages

We will install the Desktop version, which includes GUI tools like RViz.

```bash
# Update and install ROS 2 Humble Desktop
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

#### Step 4: Environment Setup

You must "source" the installation to use ROS 2 commands.

```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify sourcing (add to .bashrc for convenience)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Step 5: Test Turtlesim

Open two terminals and run the turtle simulator.

**Terminal 1**:
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2**:
```bash
ros2 run turtlesim turtle_teleop_key
```

Use the arrow keys in Terminal 2 to move the turtle in Terminal 1!

**Success Criteria**:
- [ ] ROS 2 Humble installed successfully
- [ ] `turtlesim_node` opens a blue window with a turtle
- [ ] `turtle_teleop_key` allows moving the turtle via keyboard

---

## Summary

**Key Concepts Covered**:
1. **Middleware**: ROS 2 acts as the communication glue for robotics.
2. **Nodes**: The building blocks of a robotic system.
3. **Sourcing**: Making the OS aware of the ROS 2 software.

**Skills Acquired**:
- Installing a specific ROS 2 distribution.
- Managing environment variables for ROS 2.
- Running and interacting with nodes from the command line.

**Connection to Next Chapter**: Now that we have ROS 2 running, we'll learn how these nodes actually communicate in **Chapter 3: ROS 2 Nodes and Topics**.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Is ROS 2 an Operating System like Ubuntu?
A) Yes, it replaces Linux on robots.
B) No, it is a middleware framework that runs on top of an OS.

**Question 2** (Difficulty: Medium)
Why should we divide robot software into many small nodes instead of one large program?

**Question 3** (Difficulty: Hard)
What is the purpose of the `source` command when working with ROS 2?

---

## Next Chapter

Continue to **[Chapter 3: ROS 2 Nodes and Topics](./03-nodes-topics.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
