# Chapter 10: SDF and World Building

**Module**: Module 2 - Gazebo & Unity Simulation
**Estimated Time**: 2 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Chapter 9 (Introduction to Robot Simulation)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Understand** the Simulation Description Format (SDF) and its structure
2. **Create** custom virtual worlds with lighting, terrain, and physics properties
3. **Populate** environments with static and dynamic models
4. **Configure** physics engine parameters for stability and accuracy
5. **Manage** resource paths and model dependencies in Gazebo

---

## Introduction

In the previous chapter, we launched an "empty" world. However, robots in the real world don't operate in voids—they navigate warehouses, climb stairs, and interact with objects.

To test a Physical AI agent, we must build a **Virtual Environment** that mimics the constraints of reality. This is done using the **Simulation Description Format (SDF)**, an XML-based format used by Gazebo to define everything in a simulation except for the robot's logic.

---

## Core Content

### Section 1: SDF vs. URDF

While **URDF** (Unified Robot Description Format) is great for defining the kinematics of a robot (joints and links), **SDF** is designed for the world itself.

- **URDF**: Best for robot structure.
- **SDF**: Best for environments, lighting, physics, and model nesting.

### Section 2: Building a Simple World

A basic SDF world file contains:
1. `<scene>`: Defines ambient lighting and shadows.
2. `<gui>`: Sets the initial camera position.
3. `<physics>`: Defines the physics engine (standard gravity is -9.8 m/s² on the Z-axis).
4. `<model>`: Placed objects like ground planes, walls, or obstacles.

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="my_custom_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
  </world>
</sdf>
```

---

## Hands-On Exercise

### Exercise 10: Creating a Warehouse World

**Objective**: Create a `.sdf` file for a simple warehouse environment.

1. Create a file named `warehouse.sdf`.
2. Add a Ground Plane and Sun using the `<include>` tag.
3. Use the Fuel model database to add "Coke Can" or "Construction Barrel" models.
4. Launch the world:
   ```bash
   gz sim warehouse.sdf
   ```

---

## Assessment Questions

1. What is the main difference between URDF and SDF?
2. Why is the `real_time_factor` important in simulation?
3. How do you include a pre-made model from an online database (like Gazebo Fuel)?

---

## Summary
You now know how to build a static environment. In the next chapter, we will add "eyes" and "ears" to our robots by simulating sensors like LiDAR and cameras within these worlds.
