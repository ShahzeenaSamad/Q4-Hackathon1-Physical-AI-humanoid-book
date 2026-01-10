# Instructor Guide

This guide is for instructors using this textbook to teach a 13-week Physical AI and Humanoid Robotics course.

---

## 📅 Suggested Syllabus (13 Weeks)

| Week | Module | Topics | Chapter |
|------|--------|--------|---------|
| 1-2 | Module 1 | Intro to Physical AI, ROS 2 Installation | 1-2 |
| 3 | Module 1 | Nodes, Topics, Messages | 3 |
| 4 | Module 1 | Services, Actions, Launch Files | 4-5 |
| 5 | Module 1 | URDF, RViz, Best Practices | 6-8 |
| 6 | Module 2 | Simulation Basics, SDF Worlds | 9-10 |
| 7 | Module 2 | Sensors, Unity Hub, Sim-to-Real | 11-13 |
| 8-9 | Module 3 | NVIDIA Isaac Sim, Photorealistic Sim | 14-15 |
| 10 | Module 3 | Visual SLAM, Nav2, Manipulation | 16-18 |
| 11 | Module 4 | VLA Intro, Whisper Voice Control | 19-20 |
| 12 | Module 4 | Cognitive Planning, VLMs | 21-22 |
| 13 | Module 4 | Capstone Project Finalization | 23-24 |

---

## 📝 Assessment Keys (Sample)

### Chapter 1: Intro to Physical AI
1.  **Q**: Primary difference between digital and physical AI?
    **A**: Physical AI interacts with the physical world through sensors and actuators with real-time constraints and noise.
2.  **Q**: What is the "Perception-Action Loop"?
    **A**: The continuous cycle of reading sensors, processing data, and executing motor commands.

### Chapter 3: Nodes and Topics
1.  **Q**: What happens if two nodes publish to the same topic?
    **A**: Both messages are interleaved on the topic. Subscribers will receive messages from both sources.

---

## 🛠️ Lab Setup Recommendations
- **Local Workstations**: NVIDIA RTX 3060+ GPU, 16GB RAM, 100GB Free Space.
- **Alternative**: AWS RoboMaker or NVIDIA Omniverse Cloud for students without GPU access.
