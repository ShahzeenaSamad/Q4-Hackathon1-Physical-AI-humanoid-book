# Chapter 10: SDF and World Building

**Module**: Module 2: Gazebo & Unity Simulation
**Estimated Time**: 2.5 hours (Reading: 60 min, Hands-on: 90 min)
**Prerequisites**: Chapter 9: Introduction to Robot Simulation

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** the structure and purpose of SDF (Simulation Description Format) in robot simulation.
2. **Design** complex 3D environments using nested models, lighting, and specialized physics properties.
3. **Utilize** the Gazebo Fuel library to import community-contributed 3D models.
4. **Modify** world-level properties like gravity, atmospheric pressure, and physics step size.

---

## Introduction

In Chapter 9, we launched a pre-made world. But what if you need to test your robot in a specific kitchen, a warehouse, or even the surface of Mars? To build these digital environments, we use **SDF (Simulation Description Format)**.

While URDF was designed to describe just the robot, SDF was designed to describe everything: robots, static furniture, lighting, and the physical laws of the world itself.

**Topics covered in this chapter**:
- SDF vs URDF: Why we need both
- The Hierarchy of an SDF World
- Managing 3D Assets: Fuel and local directories
- Building your first warehouse environment

**Why this matters**: A robot's behavior is defined by its environment. A navigation algorithm that works in an empty field will fail in a crowded office. Understanding world building allows you to create "Edge Case" scenarios—like extreme shadows or narrow corridors—that stress-test your AI.

**Example Use Case**: To train a humanoid robot to assist elderly people, you build an SDF world representing a 2-bedroom apartment with realistic furniture, sliding doors, and carpeted floors to test the robot's balance and interaction.

---

## Core Content

### Section 1: SDF Structure

An SDF file is hierarchical. A `<world>` contains `<model>`s, `<light>`s, and `<physics>` settings.

#### Key Points
- **Links and Joints**: Models in SDF use links and joints just like URDF, but with more advanced features (like nested models).
- **Physics Engine**: You can tune the solver parameters (e.g., `max_step_size`) to trade off between simulation accuracy and CPU speed.

---

### Section 2: Fuel Library and Asset Management

Gazebo Fuel is an online database of thousands of high-quality 3D models. You can reference them directly in your SDF file using a URL.

#### Example 10.1: Referencing a Fuel Model (XML)

```xml
<!-- Example of including a Fuel model in a world -->
<include>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke Can
  </uri>
  <pose>0 0 0.5 0 0 0</pose>
</include>
```

---

## Hands-On Exercise

### Exercise 10: Building a Warehouse Testing Lab

**Objective**: Create a custom warehouse world using SDF, including a floor, walls, lighting, and imported obstacles.

**Estimated Time**: 90 minutes

#### Step 1: Create the World File

Create a file named `warehouse_lab.sdf`.

```xml
<!-- File: warehouse_lab.sdf -->
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="warehouse_lab">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Global Light -->
    <light type="directional" name="sun">
      <cast_shadows>true</light>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</model>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

#### Step 2: Add Obstacles from CLI

Start the simulator with your new world:

```bash
gz sim warehouse_lab.sdf
```

In a new terminal, use the `gz service` command to spawn additional models (e.g., a shelving unit) from the Fuel library into the running simulation.

#### Step 3: Tuning Physics

Modify the `max_step_size` in your XML file to `0.01` and observe how the simulation seems "jittery" or less accurate, but consumes less CPU.

**Success Criteria**:
- [ ] Customized warehouse world loads in Gazebo.
- [ ] Directional lighting creates visible shadows.
- [ ] At least one community model (Fuel) is successfully spawned.

---

## Summary

**Key Concepts Covered**:
1. **SDF (Simulation Description Format)**: The standard for environment modeling.
2. **Static vs. Dynamic Models**: Defining what moves and what stays fixed.
3. **Fuel Library**: Leveraging open-source assets for realistic worlds.

**Skills Acquired**:
- Authoring multi-component SDF worlds.
- Managing 3D lighting and physics properties.
- Integrating online asset libraries into local workflows.

**Connection to Next Chapter**: Now that our robot has a world to live in, it needs to see. In **Chapter 11: Sensor Simulation**, we'll add digital cameras, lidars, and depth sensors to our models and stream that data back to ROS 2.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which tag in an SDF file is used to indicate that a model (like a wall) cannot be moved by physical collisions?
A) `<fixed>`
B) `<static>true</static>`
C) `<collision>false</collision>`

**Question 2** (Difficulty: Medium)
What is the primary advantage of SDF over URDF when building a simulation?

**Question 3** (Difficulty: Hard)
How does reducing the `max_step_size` in the physics engine affects the trade-off between realism and computational cost?

---

## Next Chapter

Continue to **[Chapter 11: Sensor Simulation](./11-sensor-simulation.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
