# Chapter 21: Cognitive Planning with LLMs

**Module**: Module 4 - Vision-Language-Action (VLA)
**Estimated Time**: 2.5 hours (Reading: 60 min, Hands-on: 90 min)
**Prerequisites**: Chapter 19 (Introduction to VLA), Module 3 (NVIDIA Isaac)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Explain** how Large Language Models (LLMs) function as "Reasoning Cores" for robots
2. **Implement** Hierarchical Task Decomposition using an LLM
3. **Design** prompts that constrain LLM output to valid robotic actions (JSON/Code)
4. **Implement** an error recovery loop where the robot reports failures and asks for new plans
5. **Analyze** the trade-offs between zero-shot planning and few-shot planning in robotics

---

## Introduction

In the previous chapter, we mapped "Move forward" to a direct motor command. But what if the user says, "Clean the kitchen counter"? This command is underspecified—the robot doesn't know where the counter is, what "clean" means for a robot, or what tools it needs.

**Cognitive Planning** is the process where a high-level goal is broken down into a sequence of executable primitives. In current Physical AI research, we use LLMs as the "Inner Monologue" that interprets these goals and manages the robot's logic.

---

## Core Content

### Section 1: Hierarchical Task Planning

Robot planning is often hierarchical:
1. **High-Level Goal**: "Make a cup of tea."
2. **Intermediate Steps (Planning)**:
   - FIND_KETTLE -> FILL_KETTLE -> BOIL_KETTLE -> FIND_MUG...
3. **Execution Primitives (Action)**:
   - MOVE_TO(X, Y) -> GRASP(Kettle) -> ACTIVATE(Tap)...

LLMs excel at the mapping between Step 1 and Step 2.

### Section 2: JSON as the "Interface Language"

To connect an LLM to a ROS 2 system, we usually ask the model to output **JSON**. This structured format can be easily parsed by our Python nodes to trigger specific actions.

Example Prompt:
*"You are a robot controller. Output ONLY a valid JSON list of actions for the goal: 'Put the apple in the basket'. Options: [MOVE_TO, GRASP, RELEASE]. Output: `[{"action": "MOVE_TO", "target": "apple"}, ...]`"*

---

## Hands-On Exercise

### Exercise 21: The "Kitchen Assistant" Planner

**Objective**: Create a Python script that uses an LLM to plan a breakfast task.

1. Set up a Python environment with the `openai` or `anthropic` SDK.
2. Define a set of "Robot Skills" (e.g., `navigate_to`, `pick_up`, `open_container`).
3. Write a script that takes a user command (e.g., "I want cereal") and sends it to the LLM with a list of available skills.
4. Parse the LLM's response and print the "Execution Plan."
5. Introduce a failure (e.g., "The milk carton is empty") and ask the LLM to Generate a "Recovery Plan."

---

## Assessment Questions

1. Why are LLMs better at task decomposition than traditional rule-based state machines?
2. What is "Prompt Injection" in a robotics context, and why is it a safety risk?
3. How do you handle an LLM generating a step that the robot doesn't have a "skill" for?

---

## Summary
The robot can now think and plan. In the next chapter, we will combine this "brain" with its "eyes" to enable **Multimodal Interaction**—allowing the robot to see the objects it is talking about.
