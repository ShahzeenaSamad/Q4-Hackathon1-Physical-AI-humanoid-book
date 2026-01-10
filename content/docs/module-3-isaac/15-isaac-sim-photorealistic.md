# Chapter 15: Isaac Sim for Photorealistic Simulation

**Module**: Module 3: NVIDIA Isaac Platform
**Estimated Time**: 3.0 hours (Reading: 60 min, Hands-on: 120 min)
**Prerequisites**: Chapter 14: Introduction to NVIDIA Isaac

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Construct** photorealistic 3D environments in Isaac Sim using the USD (Universal Scene Description) framework.
2. **Utilize** Isaac Replicator to generate domain-randomized synthetic datasets for AI training.
3. **Configure** advanced sensor models with ray-tracing for high-fidelity visual and lidar data.
4. **Implement** a ROS 2 bridge between Isaac Sim and external control nodes.

---

## Introduction

In typical simulators, gravity is calculated carefully, but light is an afterthought. In **Isaac Sim**, light is treated with the same physical accuracy as gravity. By using **Ray Tracing** and **Path Tracing**, Isaac Sim simulates how light bounces off surfaces, creating realistic shadows, reflections, and glares.

This isn't just for aesthetics. Real-world robots often fail because of glare on a floor or a shadow that looks like an obstacle. Real-time photorealistic simulation prepares your robot's vision system for these exact conditions.

**Topics covered in this chapter**:
- Environment Building: From CAD to USD
- Physically Based Rendering (PBR) Materials
- Isaac Replicator: Automating synthetic data generation
- Bridging the Gap: ROS 2 connectivity in Isaac Sim

**Why this matters**: High-quality data is the lifeblood of Physical AI. Isaac Replicator allows you to generate 100,000 labeled images in an afternoon—labeled with perfect accuracy (bounding boxes, depth, segmentation)—which would take humans months to annotate manually.

**Example Use Case**: You are training a humanoid to identify surgical tools. Since surgical environments have highly reflective metallic surfaces, you use Isaac Sim's ray-tracing to simulate the specular glare on the tools, ensuring your vision model can still identify them under bright operating room lights.

---

## Core Content

### Section 1: USD Scene Composition

USD is more than just a file format; it's a composition engine. It allows multiple developers to work on different "layers" of a scene simultaneously without overwriting each other's work.

#### Key Principles
- **References**: Including one USD file inside another.
- **Layers**: Overriding properties of an object (e.g., its color or position) on a separate layer.
- **Variants**: Easy switching between different versions of an object (e.g., a "clean" vs. "dirty" version of a robot).

---

### Section 2: Isaac Replicator

Isaac Replicator is a tool for **Synthetic Data Generation (SDG)**. It uses **Domain Randomization** to automatically vary the environment (lighting, textures, positions) while collecting ground-truth data.

#### Example 15.1: A Basic Replicator Script (Python)

```python
# Environment: Isaac Sim 2023+
import omni.replicator.core as rep

with rep.new_layer():
    # 1. Add objects
    camera = rep.create.camera(position=(0, 0, 5))
    render_product = rep.create.render_product(camera, resolution=(1024, 1024))

    # 2. Randomize objects
    with rep.trigger.on_frame(num_frames=10):
        # Move the robot slightly every frame
        with rep.create.from_usd("robot.usd"):
            rep.modify.pose(position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0)))

    # 3. Setup writer (output labels + images)
    writer = rep.WriterRegistry.get_writer("BasicWriter")
    writer.initialize(output_dir="_sdg_output", rgb=True, bounding_box_2d_tight=True)
    writer.attach([render_product])
```

---

## Hands-On Exercise

### Exercise 15: Building a Photorealistic Warehouse & Generating Data

**Objective**: Create a warehouse laboratory in Isaac Sim, add PBR materials, and use Replicator to generate a small set of labeled training data.

**Estimated Time**: 120 minutes

#### Step 1: Create a New Stage

1. Open Isaac Sim.
2. Select `File > New`.
3. Add a **Physics Scene** and **Ground Plane** from the Robotics menu.

#### Step 2: Import High-Quality Assets

1. Open the **Content Browser**.
2. Navigate to `NVIDIA > Assets > Isaac > 2023.1 > Isaac > Environments > Warehouse`.
3. Drag and drop `Warehouse_With_Shelves.usd` into your stage.
4. Note the detail of the materials (rust on the shelves, scuffs on the floor). These are PBR materials.

#### Step 3: Run the SDG Script

Copy the Python snippet from Section 2 into the Isaac Sim **Script Editor** (`Window > Script Editor`). Modify the file path to point to a pallet USD in the warehouse. Click **Run**.

Alternatively, you can run replicator scripts from the command line:

```bash
# Execute a standalone replicator script
./python.sh my_replicator_script.py
```

#### Step 4: Verify Output

Navigate to the `_sdg_output` directory on your computer. You should see RGB images and corresponding JSON files containing the bounding box coordinates for the pallets.

**Success Criteria**:
- [ ] Warehouse environment renders with ray-traced shadows.
- [ ] Script generates at least 10 domain-randomized images.
- [ ] Bounding box labels correctly identify the target pallets in the output JSON.

---

## Summary

**Key Concepts Covered**:
1. **PBR Materials**: Accurate representation of light interaction with physical surfaces.
2. **Synthetic Data Generation (SDG)**: Automating the creation of training data.
3. **USD Layers**: Managing complex, multi-user robotic environments.

**Skills Acquired**:
- Compositing large-scale USD environments.
- Using Python scripting to automate data collection.
- Understanding the "Ground Truth" pipeline for robot vision.

**Connection to Next Chapter**: Perception is the first step; speed is the next. In **Chapter 16: Isaac ROS for Hardware-Accelerated Perception**, we will take the models we train on this data and run them on Jetson hardware using NVIDIA's optimized ROS 2 packages.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
What does PBR stand for in the context of simulator textures?
A) Physical Based Rendering
B) Pixel Based Rotation
C) Position Based Rays

**Question 2** (Difficulty: Medium)
Explain how Isaac Replicator can help improve a robot's ability to see in low-light conditions.

**Question 3** (Difficulty: Hard)
What are "USD Variants," and how would you use them to test a robot's performance under different environmental conditions?

---

## Next Chapter

Continue to **[Chapter 16: Isaac ROS for Hardware-Accelerated Perception](./16-isaac-ros-perception.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
