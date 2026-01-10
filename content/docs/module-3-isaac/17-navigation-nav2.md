# Chapter 17: Navigation with Nav2

**Module**: Module 3: NVIDIA Isaac Platform
**Estimated Time**: 2.5 hours (Reading: 60 min, Hands-on: 90 min)
**Prerequisites**: Chapter 16: Isaac ROS for Hardware-Accelerated Perception

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** the modular architecture of the Nav2 (Navigation 2) stack in ROS 2.
2. **Configure** costmaps and behavior trees for autonomous humanoid navigation.
3. **Integrate** hardware-accelerated perception data (VSLAM/LiDAR) into the Nav2 pipeline.
4. **Implement** path planning and obstacle avoidance algorithms in a simulated dynamic environment.

---

## Introduction

A robot that knows where it is but cannot move intelligently is a statue. To create an active agent, we need the **Nav2** stack. Nav2 is the industry-standard software for autonomous mobility in ROS 2. It handles the complex math of "How do I get from Point A to Point B without hitting a person?"

While Nav2 can run on standard CPUs, in this module we learn how to feed it high-quality, high-speed data from **Isaac ROS** and **Isaac Sim** to build navigation systems that are significantly more responsive.

**Topics covered in this chapter**:
- Nav2 Architecture: Planners, Controllers, and Recoveries
- Configuration with Behavior Trees
- Global vs. Local Costmaps
- Real-time obstacle avoidance in Isaac Sim

**Why this matters**: In humanitarian and industrial robotics, navigation must be perfect. A 500lb humanoid robot cannot make mistakes. Nav2 provides the safety "guardrails" and specialized behaviors (like backing up or spinning) that ensure your robot moves gracefully through unpredictable spaces.

**Example Use Case**: A humanoid logistics robot in a busy mall uses Nav2 to map out a clear path through a crowd. When a child runs in front of it, the local controller—fed by Isaac ROS perception—detects the child in milliseconds and halts the robot's motion immediately.

---

## Core Content

### Section 1: Nav2 Architecture

Nav2 is based on a **Modular Server** architecture. Instead of one large program, it's divided into specialized nodes:

#### Key Components
- **Planner Server**: Calculates the "Global Path" (the long-distance route on a map).
- **Controller Server**: Handles the "Local Path" (individual steering commands to avoid immediate obstacles).
- **Behavior Tree (BT) Navigator**: The brain of the stack. It uses BTs to decide when to plan, when to move, and what to do if the robot gets stuck.
- **Costmaps**: 2D or 3D grids that represent obstacles.
  - *Static layer*: The walls of the building.
  - *Obstacle layer*: Moving people, chairs, or boxes.

---

### Section 2: Behavior Trees for Navigation

Nav2 uses **Behavior Trees** instead of simple FSMs (Finite State Machines). This allows for complex navigation logic, like "Try this path, if it fails, try a different planner, if that fails, beep for help."

#### Visual Aid

![Figure 17.1: Nav2 Modular Design](../../../static/img/module-3/nav2-architecture.png)

*Figure 17.1: The relationship between sensors, costmaps, planners, and the behavior tree.*

---

## Hands-On Exercise

### Exercise 17: Navigating a Humanoid in a Warehouse Stage

**Objective**: Configure Nav2 to guide a bipedal robot through a maze of pallets in Isaac Sim.

**Estimated Time**: 90 minutes

#### Step 1: Launch the Navigation Demo

Isaac Sim provides a ready-to-run Nav2 integration demo using an RTX GPU.

```bash
# Set up ROS 2 environment first
source /opt/ros/humble/setup.bash

# Use the Isaac Sim environment loader (standard path)
./python.sh samples/ros2/navigation/warehouse_nav.py
```

1. Open Isaac Sim.
2. Select `Window > Robotics > Navigation > Warehouse Navigation`.
3. Click **Play**.

#### Step 2: Set a Goal in RViz

1. Open the pre-configured RViz session.
2. Use the **2D Nav Goal** tool in the RViz top toolbar.
3. Click and drag somewhere in the warehouse.
4. Watch the "Global Path" appear as a purple line and the robot begin its journey.

#### Step 3: Configure Parameters

Open the `nav2_params.yaml` file (provided in the demo directory) and modify the **inflation_radius**.

```yaml
# Environment: YAML configuration
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        # Increase this to keep the robot further away from obstacles
        inflation_radius: 0.75
        cost_scaling_factor: 3.0
```

#### Step 4: Add Dynamic Obstacle

While the robot is moving, drag a box (or a person asset) into its path in Isaac Sim. Observe how the local planner instantly recalculates the "wiggle" path around the new object.

**Success Criteria**:
- [ ] Robot successfully reaches the navigation goal without collision.
- [ ] Global and Local costmaps are correctly visualized in RViz.
- [ ] Robot demonstrates "obstacle avoidance" behavior when a dynamic object is added.

---

## Summary

**Key Concepts Covered**:
1. **Nav2**: The "Legs" of the ROS 2 software stack.
2. **Behavior Trees**: Managing complex, multi-step navigation tasks.
3. **Costmaps**: How the robot represents "Free" vs "Occupied" space.

**Skills Acquired**:
- Tuning navigation parameters for different robot form factors.
- Integrating high-frequency sensor data into local controllers.
- Managing autonomous mission logic using Behavior Trees.

**Connection to Next Chapter**: We've mastered perception and navigation. In **Chapter 18: Module 3 Summary**, we will bring all the Isaac tools together to build a complete autonomous agent and prepare for the final module: **Vision-Language-Action**.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which Nav2 component is responsible for the overall logic (deciding what to do next)?
A) Planner Server
B) BT Navigator
C) Controller Server

**Question 2** (Difficulty: Medium)
Explain the difference between a **Global Planner** and a **Local Controller**.

**Question 3** (Difficulty: Hard)
If a robot gets stuck in a narrow doorway, what part of the Nav2 stack triggers a "Clear Costmap" or "Spin" behavior? How do you customize this?

---

## Next Chapter

Continue to **[Chapter 18: Module 3 Summary](./18-module-summary.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
