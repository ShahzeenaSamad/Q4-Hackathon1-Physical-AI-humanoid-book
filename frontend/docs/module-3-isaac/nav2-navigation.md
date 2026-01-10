# Chapter 17: Navigation with Nav2

**Module**: Module 3 - NVIDIA Isaac Platform
**Estimated Time**: 2 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Module 1 (ROS 2), Chapter 10 (SDF and World Building)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** the Nav2 architecture (Planners, Controllers, Costmaps, Behavior Trees)
2. **Configure** costmaps to handle static and dynamic obstacles
3. **Understand** the role of Behavior Trees in complex navigation tasks
4. **Implement** path planning and following for a mobile robot
5. **Debug** navigation failures using Nav2 log messages and RViz visualization

---

## Introduction

In robotics, navigation is the ability for a robot to move from point A to point B safely and efficiently. The **Nav2** (Navigation 2) stack is the industry-standard framework for autonomous navigation in ROS 2.

Whether it's a warehouse robot moving palettes or a humanoid navigating a home, Nav2 provides the tools to plan paths, avoid obstacles, and handle unexpected situations (like a door being closed). In this chapter, we will integrate Nav2 with our simulation and perception pipelines.

---

## Core Content

### Section 1: Nav2 Architecture

Nav2 is built as a set of modular services:
- **Planner Server**: Computes the high-level path from A to B (the "Global Path").
- **Controller Server**: Commands the robot's motors to follow the path while avoiding local obstacles (the "Local Plan").
- **Costmap 2D**: A grid-based representation of the environment, where each cell has a "cost" (high cost = obstacle).
- **Behavior Tree (BT) Navigator**: Uses trees to manage high-level logic, such as "if path is blocked, try recovering, then replan."

### Section 2: Costmaps

A robot needs to know where it is safe to drive. Nav2 uses two types of costmaps:
1. **Global Costmap**: Used for long-term planning across the entire map.
2. **Local Costmap**: A small "window" around the robot used for immediate obstacle avoidance.

---

## Hands-On Exercise

### Exercise 17: Running Nav2 in Simulation

**Objective**: Navigate a robot to a goal using Nav2 and RViz.

1. Launch a simulated robot in Gazebo (e.g., TurtleBot3 or Carter).
2. Launch the Nav2 stack:
   ```bash
   ros2 launch nav2_bringup tb3_simulation_launch.py
   ```
3. Open RViz and set the **Initial Pose Estimate** (using the green arrow).
4. Select **Nav2 Goal** and click on a location in the map.
5. Observe the robot planning and moving toward the goal.

---

## Assessment Questions

1. Describe the difference between a Global Planner and a Local Controller.
2. What is a Behavior Tree, and why is it used in navigation?
3. How do "Inflation Layers" in costmaps help keep robots safe?

---

## Summary
You have successfully implemented autonomous navigation! This completes the "Body" and "Nervous System" of our robot. In the next module, we will build the "Brain"—using Vision-Language-Action (VLA) models to allow the robot to understand and execute complex, natural language instructions.
