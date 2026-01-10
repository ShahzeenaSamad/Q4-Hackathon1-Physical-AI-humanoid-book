# Chapter 11: Sensor Simulation

**Module**: Module 2 - Gazebo & Unity Simulation
**Estimated Time**: 1.5 hours (Reading: 30 min, Hands-on: 60 min)
**Prerequisites**: Chapter 10 (SDF and World Building)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Simulate** common robotic sensors: LiDAR, RGB-D cameras, and IMUs
2. **Configure** sensor properties like noise, resolution, and update frequency
3. **Bridge** simulated sensor data to ROS 2 topics
4. **Visualize** sensor output in RViz for debugging
5. **Understand** the performance impact of high-resolution sensors in simulation

---

## Introduction

In Physical AI, sensors are the interface between the robot and its environment. In simulation, we don't have real photons or magnetic fields, so we use **Sensor Plugins** to mimic the behavior of real hardware.

Simulating sensors allows us to develop perception algorithms (like object detection or SLAM) without needing a battery-powered robot. We can even simulate "noisy" sensors to ensure our AI is robust to real-world imperfections.

---

## Core Content

### Section 1: Simulating LiDAR

LiDAR (Light Detection and Ranging) is used for mapping and obstacle avoidance. In Gazebo, we define a GPU-based LiDAR sensor:

```xml
<sensor name='gpu_lidar' type='gpu_lidar'>
  <pose>0 0 0.5 0 0 0</pose>
  <topic>scan</topic>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.396263</min_angle>
        <max_angle>1.396263</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <always_on>1</always_on>
  <visualize>true</visualize>
</sensor>
```

---

## Hands-On Exercise

### Exercise 11: Visualizing LiDAR in RViz

**Objective**: Connect Gazebo sensor data to ROS 2 and visualize it.

1. Launch a robot with a LiDAR sensor in Gazebo.
2. Use the **ROS 2 - Gazebo Bridge** to forward the `/scan` topic:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
   ```
3. Open a new terminal and launch RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```
4. Add a "LaserScan" display and set the "Fixed Frame" to `world` or `robot_base`.

---

## Assessment Questions

1. Why do we use "GPU-based" LiDAR in simulation?
2. How do you simulate sensor noise in an SDF file?
3. What ROS 2 message type is used for depth camera point clouds?

---

## Summary
With sensors functional, our virtual robot can now "see" its environment. In the next chapter, we'll look at Unity as an alternative for even higher fidelity visuals and complex physical interactions.
