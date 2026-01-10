# Chapter 6: Understanding URDF

**Module**: Module 1: ROS 2 Fundamentals
**Estimated Time**: 2.0 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Chapter 5: Launch Files and Parameters

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** the structure of a URDF (Unified Robot Description Format) file.
2. **Define** robot kinematics using Links and Joints.
3. **Utilize** Xacro to create modular and reusable robot descriptions.
4. **Visualize** and debug robot models using `rviz2`.

---

## Introduction

So far, we've focused on the "brain" (software nodes) and "nerves" (communication) of the robot. Now, we need to describe its "body." **URDF (Unified Robot Description Format)** is an XML-based format used in ROS 2 to describe the physical properties of a robot.

**Topics covered in this chapter**:
- Links vs. Joints: The building blocks of URDF
- Visual, Collision, and Inertial properties
- Using Xacro for modularity
- Inspecting your robot in RViz

**Why this matters**: ROBOT URDF is the single source of truth for the robot's physical configuration. Simulation tools (like Gazebo), visualization tools (like RViz), and motion planning algorithms (like Nav2) all read your URDF file to know how the robot looks, how it moves, and what its limits are.

**Example Use Case**: When designing a humanoid robot, the URDF defines exactly where the elbow joint is relative to the shoulder, the range of motion for the knee, and the weight of the torso.

---

## Core Content

### Section 1: Links and Joints

A robot model in URDF is a tree structure composed of **Links** connected by **Joints**.

#### Key Points
- **Links**: These are the "bones" of the robot. They represent physical pieces (e.g., forearm, wheel, base).
- **Joints**: These are the "hinges." They define how one link moves relative to another.
  - *Fixed*: No movement.
  - *Revolute*: Rotates around an axis (e.g., an elbow).
  - *Prismatic*: Slides along an axis (e.g., a linear actuator).
  - *Continuous*: Rotates indefinitely (e.g., a wheel).

#### Visual Aid

![Figure 6.1: Link and Joint Structure](../../../static/img/module-1/urdf-structure.png)

*Figure 6.1: A simplified chain of links and joints representing a robotic limb.*

---

### Section 2: Visual, Collision, and Inertial

Every **Link** in your URDF should have three main sections:
1. **Visual**: What the robot looks like in RViz (geometry, color, meshes).
2. **Collision**: The simplified shape used by the physics engine to detect if the robot hits a wall or itself.
3. **Inertial**: Physical properties like mass and inertia tensor, required for realistic simulation.

#### Example 6.1: A Simple Box Link (XML)

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </collision>
</link>
```

---

## Hands-On Exercise

### Exercise 6: Building a 2-DOF Robotic Arm

**Objective**: Create a URDF for a simple two-jointed robotic arm and visualize it in RViz.

**Estimated Time**: 75 minutes

#### Step 1: Create the Package

```bash
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python robot_description_pkg
cd robot_description_pkg
mkdir urdf
```

#### Step 2: Write the URDF File

Create `urdf/simple_arm.urdf`:

```xml
<!-- File: simple_arm.urdf -->
<robot name="simple_arm">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry><box size="0.05 0.05 0.4"/></geometry>
    </visual>
  </link>
</robot>
```

#### Step 3: Visualize in RViz

ROS 2 provides a tool called `urdf_tutorial` (or similar) to quickly preview files.

```bash
# Preview using the joint_state_publisher_gui
ros2 launch urdf_tutorial display.launch.py model:=src/robot_description_pkg/urdf/simple_arm.urdf
```

**Success Criteria**:
- [ ] RViz opens and shows the cylinder base and box arm.
- [ ] Moving the slider in the GUI rotates the arm around its base.

---

## Summary

**Key Concepts Covered**:
1. **Kinematic Chain**: The hierarchy of links and joints.
2. **Visual vs. Collision**: Differentiating between appearance and physical boundaries.
3. **RViz**: The primary visualization tool for robot state.

**Skills Acquired**:
- Writing XML-based URDF descriptions.
- Defining revolute joints with limits.
- Debugging link frames in RViz.

**Connection to Next Chapter**: Now that we have a body and software, how do we ensure our code is reliable and follows industry standards? In **Chapter 7: ROS 2 Best Practices**, we'll learn about package organization and unit testing.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which joint type should you use for a wheel that spins at a constant speed?
A) Fixed
B) Revolute
C) Continuous

**Question 2** (Difficulty: Medium)
Why should the **Collision** geometry often be simpler than the **Visual** geometry?

**Question 3** (Difficulty: Hard)
If a joint has an `<origin xyz="0 0 0.1">`, where is the child link placed relative to the parent?

---

## Next Chapter

Continue to **[Chapter 7: ROS 2 Best Practices](./07-best-practices.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
