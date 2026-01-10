# Chapter 11: Sensor Simulation

**Module**: Module 2: Gazebo & Unity Simulation
**Estimated Time**: 2.0 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Chapter 10: SDF and World Building

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Integrate** digital sensors (Camera, LiDAR, IMU) into simulated robot models using SDF.
2. **Configure** sensor parameters such as resolution, noise models, and update rates.
3. **Bridge** simulation sensor streams to ROS 2 topics using Gazebo ROS plugins.
4. **Visualize** real-time sensor data from simulation in RViz.

---

## Introduction

A robot without sensors is like a person with their eyes closed. In the world of Physical AI, sensors are the interface through which the AI perceives its environment. In this chapter, we will learn how to add digital versions of cameras and lidars to our robots.

Adding sensors in simulation involves two steps:
1. **Defining the physical sensor component** in the SDF/URDF (geometry and pose).
2. **Configuring the sensor behavior** (how it generates data and how that data reaches ROS 2).

**Topics covered in this chapter**:
- Common robotic sensors: Cameras, Depth Sensors, and LiDAR
- Gazebo Sensor Models and Noise
- The ROS 2 Bridge: Sending simulated data to your nodes
- Visualizing point clouds and camera streams

**Why this matters**: Real-world sensors are noisy. They have resolution limits, blind spots, and interference. If you train your AI on "perfect" simulated data, it will fail on a real robot. Learning to configure noise and resolution allows you to build models that are resilient to the imperfections of the real world.

**Example Use Case**: A humanoid's head contains a depth camera (like an Intel RealSense). We simulate this camera in Gazebo to feed a "Person Detection" node, allowing the robot to track and face a person standing in front of it.

---

## Core Content

### Section 1: Types of Simulated Sensors

Gazebo supports a wide variety of sensors out of the box.

#### Key Sensor Types
- **Camera**: Standard RGB images.
- **Depth Camera**: Generates a point cloud (XYZ coordinates) and a depth map.
- **LiDAR**: Uses laser beams to measure distance (typically 2D or 3D 360-degree scans).
- **IMU (Inertial Measurement Unit)**: Reports acceleration and angular velocity.

---

### Section 2: Sensor Plugins and ROS 2 Bridge

To make simulated sensor data available to ROS 2, we use **plugins**. These small pieces of code "glue" the simulator's internal data to a ROS 2 topic.

#### Example 11.1: Configuring a Camera Sensor (SDF)

```xml
<sensor name="camera" type="camera">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <!-- Plugin to bridge to ROS 2 -->
  <plugin name="gz::sim::systems::ImagePublisher" filename="gz-sim-image-publisher-system">
    <topic>camera/image_raw</topic>
  </plugin>
</sensor>
```

---

## Hands-On Exercise

### Exercise 11: Integrating a LiDAR and Visualizing in RViz

**Objective**: Add a LiDAR sensor to a simple robot model and visualize the scan data in RViz.

**Estimated Time**: 75 minutes

#### Step 1: Add LiDAR to your URDF/SDF

Open your robot description from Chapter 6 and add a laser sensor block.

#### Step 2: Launch the ROS 2 Bridge

Gazebo and ROS 2 are separate systems. We need to run a bridge node to translate the messages.

```bash
# Run the bridge for a laser scan
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

#### Step 3: Visualize in RViz

1. Start your simulation with the LiDAR-enabled robot.
2. Open RViz (`rviz2`).
3. Set the **Fixed Frame** to `base_link`.
4. Add a **LaserScan** display type and select the `/lidar` topic.

**Success Criteria**:
- [ ] Robot model in Gazebo shows a visual representation of the laser rays.
- [ ] `ros2 topic echo /lidar` shows a stream of distance values.
- [ ] RViz displays red/yellow dots representing the laser hits on obstacles.

---

## Summary

**Key Concepts Covered**:
1. **Perception**: The integration of sensor data into the simulation.
2. **Noise Models**: Simulating real-world sensor inaccuracies.
3. **The Bridge**: The communication link between Simulation and ROS 2.

**Skills Acquired**:
- Adding and configuring RGB and Depth sensors in SDF.
- Managing communication bridges between different middlewares.
- Debugging perception systems using RViz and command-line tools.

**Connection to Next Chapter**: Now that we've mastered Gazebo, we'll look at an alternative. In **Chapter 12: Unity for High-Fidelity Rendering**, we'll explore how to use the Unity game engine for even more realistic visuals and complex environments.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which tool is used to visualize point clouds and camera streams from a ROS 2 system?
A) Gazebo
B) rqt_graph
C) RViz

**Question 2** (Difficulty: Medium)
Why might you set `<visualize>true</visualize>` for a sensor during development, but `false` during final training?

**Question 3** (Difficulty: Hard)
Explain the role of the `ros_gz_bridge` in a simulation workflow. What would happen to your ROS 2 nodes if the bridge was not running?

---

## Next Chapter

Continue to **[Chapter 12: Unity for High-Fidelity Rendering](./12-unity-high-fidelity.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
