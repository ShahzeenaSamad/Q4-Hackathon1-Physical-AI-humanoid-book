# Chapter 23: Capstone Project - The Autonomous Humanoid

**Module**: Module 4 - Vision-Language-Action (VLA)
**Estimated Time**: 1 Week (10-15 hours)
**Prerequisites**: Completion of all previous chapters (1-22)

---

## Learning Outcomes

By the end of this capstone project, you will be able to:

1. **Integrate** multiple robotics and AI subsystems into a unified Physical AI agent
2. **Execute** an end-to-end task pipeline: Listen -> Think -> See -> Navigate -> Manipulate
3. **Debug** cross-system issues (e.g., communication delays between LLM and ROS 2)
4. **Document** a complex software architecture for a humanoid robot
5. **Demonstrate** a fully autonomous interaction in a high-fidelity simulation

---

## The Challenge

For your final project, you will develop the "Brain" and "Body" for a simulated humanoid robot tasked with assisting in a domestic or industrial environment.

**Task Scenario**: The robot is in a room with several objects. It must respond to a natural language voice command, navigate to the correct location, identify a specific object, and perform a physical interaction.

**Example Command**: *"The office is a mess. Take this bottle and put it in the recycling bin."*

---

## System Architecture

Your solution must integrate the following components:

| Component | Technology | Role |
|-----------|------------|------|
| **Ear** | OpenAI Whisper | Convert voice to text command |
| **Mind** | GPT-4o / Claude 3.5 | Task decomposition & Cognitive Planning |
| **Eyes** | Isaac ROS / VLM | Object grounding & 3D localization |
| **Legs** | Nav2 / Isaac Sim | Autonomous navigation to the workstation |
| **Arms** | ROS 2 Control / Lula | Motion execution for grasp and drop |
| **Nervous System** | ROS 2 Humble | Inter-process communication |

---

## Project Requirements

### 1. Functional Requirements
- **FR-1**: The robot must transcribed audio with >90% accuracy in a quiet room.
- **FR-2**: The high-level plan must contain at least 4 discrete steps (Move, Find, Grasp, Drop).
- **FR-3**: The robot must navigate to within 0.5m of the target object without collision.
- **FR-4**: The final action (dropping the object) must be confirmed via verbal feedback.

### 2. Deliverables
- **Codebase**: A clean, documented ROS 2 workspace.
- **Video Demo**: A 3-minute screen recording of the simulation running the end-to-end task.
- **Architecture Diagram**: A visual map showing how data flows between your nodes.

---

## Evaluation Rubric

| Criteria | Points | Excellent (4-5) | Satisfactory (2-3) |
|----------|--------|-----------------|--------------------|
| **Integration** | 20 | All systems communicate seamlessly. | Minor manual restarts required. |
| **Robustness** | 10 | Handles one "unexpected" failure. | Fails on any variation. |
| **Documentation** | 10 | README clearly explains how to run. | Code lacks comments. |
| **Innovation** | 10 | Extraordinary feature added. | Follows basic tutorial only. |

---

## Summary
The Capstone Project is your chance to show the world what you've learned. By completing this, you move from being a student of Physical AI to a practitioner capable of building the future of humanoid robotics. Good luck!
