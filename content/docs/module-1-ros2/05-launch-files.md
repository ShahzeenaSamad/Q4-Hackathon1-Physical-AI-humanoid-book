# Chapter 5: Launch Files and Parameters

**Module**: Module 1: ROS 2 Fundamentals
**Estimated Time**: 1.5 hours (Reading: 45 min, Hands-on: 45 min)
**Prerequisites**: Chapter 4: ROS 2 Services and Actions

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Write** Python-based launch files to automate the startup of multiple nodes.
2. **Utilize** ROS 2 Parameters to configure node behavior dynamically without re-compiling.
3. **Organize** complex system architectures using Namespaces and Remapping.
4. **Debug** system startup issues using `ros2 launch` CLI.

---

## Introduction

As your robot becomes more complex, manually starting each node in a new terminal becomes impossible. A humanoid robot might have 50+ nodes running simultaneously! **Launch Files** provide a way to start everything with a single command.

Furthermore, you often need to tweak how a node works (e.g., changing the speed limit of a motor). **Parameters** allow you to change these values without digging into the source code.

**Topics covered in this chapter**:
- The structure of a Python Launch File
- Managing Node Parameters (YAML and CLI)
- Namespaces and Topic Remapping
- Composing complex launches from smaller files

**Why this matters**: Launch files and parameters are the "configuration layer" of your robot. They allow you to reuse the same code for different robots by simply changing a configuration file.

**Example Use Case**: You have a `camera_driver` node. Using parameters, you can tell the node which camera port to open (`/dev/video0` vs `/dev/video1`) and what resolution to use, all from a Launch file.

---

## Core Content

### Section 1: ROS 2 Parameters

Parameters are configuration values stored within a node. Think of them as "settings."

#### Key Points
- **Node-Specific**: Each node manages its own parameters.
- **Dynamic**: Parameters can be changed while the node is running.
- **YAML Format**: Complex configurations are usually stored in `.yaml` files.

#### Example 5.1: Accessing Parameters in Python

```python
# Environment: Ubuntu 22.04 + ROS 2 Humble
class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        # Declare a parameter with a default value
        self.declare_parameter('my_param', 'default_value')

        # Get the parameter value
        my_param = self.get_parameter('my_param').get_parameter_value().string_value
        self.get_logger().info(f'Parameter value: {my_param}')
```

---

### Section 2: Python Launch Files

In ROS 2, launch files are written in Python. This gives you the power to use logic (if/else statements, loops) to decide which nodes should start.

#### Key Points
- **Node Action**: Used to define which node to start, from which package.
- **LaunchDescription**: The top-level container for all launch actions.
- **Remapping**: Changing a topic name for a specific node (e.g., from `/cmd_vel` to `/teleop_vel`).

---

## Hands-On Exercise

### Exercise 5: Orchestrating the Heartbeat System

**Objective**: Create a launch file that starts both the Heartbeat Publisher and Status Monitor from Chapter 3, while passing a custom label parameter.

**Estimated Time**: 45 minutes

#### Step 1: Create the Launch Directory

In your package, create a `launch` folder.

```bash
cd ~/dev_ws/src/robot_status_pkg
mkdir launch
```

#### Step 2: Write the Launch File

Create `launch/robot_system_launch.py`:

```python
# File: robot_system_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_status_pkg',
            executable='heartbeat_cmd',
            name='publisher_node',
            parameters=[{'heartbeat_label': 'PHYSICAL_AI_LAB'}]
        ),
        Node(
            package='robot_status_pkg',
            executable='monitor_cmd',
            name='subscriber_node',
            remappings=[('robot_heartbeat', 'filtered_heartbeat')]
        )
    ])
```

#### Step 3: Run the Launch

```bash
# General syntax: ros2 launch <package_name> <launch_file_name>
ros2 launch robot_status_pkg robot_system_launch.py
```

**Success Criteria**:
- [ ] Both nodes start with one command.
- [ ] Nodes appear in `rqt_graph`.
- [ ] Publisher uses the custom label from the parameter.

---

## Summary

**Key Concepts Covered**:
1. **Launch Files**: Automation for starting robot systems.
2. **Parameters**: External configuration for nodes.
3. **Remapping**: Redirecting data flow without changing source code.

**Skills Acquired**:
- Writing basic Python launch files.
- Declaring and getting parameters in Python nodes.
- Using `ros2 param` CLI commands.

**Connection to Next Chapter**: Now that we can start complex systems, how do we describe the physical shape of the robot? In **Chapter 6: Understanding URDF**, we'll learn how to define the robot's links, joints, and sensors.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which file format is most commonly used to store batches of ROS 2 parameters?
A) JSON
B) XML
C) YAML

**Question 2** (Difficulty: Medium)
Explain the difference between a `package` name and an `executable` name in a Launch file.

**Question 3** (Difficulty: Hard)
If you have two identical robots running the same code on the same network, how can you use **Namespaces** to prevent their topics from clashing?

---

## Next Chapter

Continue to **[Chapter 6: Understanding URDF](./06-urdf-robot-description.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
