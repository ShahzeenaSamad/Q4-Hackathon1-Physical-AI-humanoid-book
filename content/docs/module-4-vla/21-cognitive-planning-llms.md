# Chapter 21: Cognitive Planning with LLMs

**Module**: Module 4: Vision-Language-Action (VLA)
**Estimated Time**: 2.5 hours (Reading: 60 min, Hands-on: 90 min)
**Prerequisites**: Chapter 20: Voice-to-Action with Whisper

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Design** complex robotic mission plans by leveraging the reasoning capabilities of Large Language Models (LLMs).
2. **Implement** error recovery and feedback loops that allow the robot to "re-plan" when a task fails.
3. **Utilize** LangChain agents to manage long-term memory and tool orchestration for Physical AI.
4. **Compare** zero-shot, few-shot, and fine-tuned models for specific robotic domain expertise.

---

## Introduction

In Chapter 19, we saw how a simple prompt can decompose a task. But the real world is messy. Steps fail, batteries run low, and users change their minds. To build a truly autonomous humanoid, we need **Cognitive Planning**—the ability for the robot to reason about its state, its goals, and its failures.

By using LLMs as a "Cognitive Layer," we can give robots a form of common sense. Instead of a hard-coded script, the robot has a dynamic brain that can solve problems it has never seen before.

**Topics covered in this chapter**:
- The Agentic Workflow: Loop, Observe, Act
- Error Recovery: Teaching the AI to "Debug" its own mission
- Memory Management: Keeping track of what the robot has done
- LangChain for Robotics: Building tool-using agents

**Why this matters**: Hard-coded logic cannot handle the infinite variety of the real world. A robot programmed for "Kitchen cleaning" might get stuck if it finds a new type of glass it doesn't recognize. Cognitive planning allows the robot to say, "I don't recognize this object, let me ask the user or move it carefully" rather than simply failing.

**Example Use Case**: You tell a humanoid, "Prepare the guest room." The robot discovers the room is locked. Instead of stopping, its cognitive planner reasons: "The door is locked -> I need a key -> the key is usually in the drawer -> I will go to the drawer first."

---

## Core Content

### Section 1: The OODA Loop for LLMs

OODA stands for **Observe, Orient, Decide, Act**. In our system, the LLM performs the "Decide" and "Orient" phases.

#### Key Patterns
- **Observation**: Feeding the current ROS 2 state (battery level, location, sensor data) back into the prompt.
- **Feedback Loop**: When a ROS 2 action returns `REJECTED` or `ABORTED`, the LLM receives the error message and generates a corrective action.

---

### Section 2: Agents and Tools

An **Agent** is an LLM that has access to a set of **Tools**. In robotics, our "Tools" are the ROS 2 Service and Action nodes we've built in previous chapters.

#### Tool Inventory Example
- `move_to(location)` -> `Nav2 Action`
- `check_vision(object)` -> `Isaac ROS Inference`
- `grasp(object)` -> `Joint Controller Service`

---

## Hands-On Exercise

### Exercise 21: Building a Self-Healing Robot Mission

**Objective**: Create a LangChain-based agent that plans a multi-step task and intelligently handles a simulated failure (e.g., "The door is blocked").

**Estimated Time**: 90 minutes

#### Step 1: Install LangChain and OpenAI

```bash
pip install langchain langchain-openai
```

#### Step 2: Create the Agent Node

Create `vla_pkg/cognitive_agent.py`:

```python
# Environment: Python 3.10 + LangChain
from langchain_openai import ChatOpenAI
from langchain.agents import AgentExecutor, create_openai_functions_agent
from langchain_core.prompts import ChatPromptTemplate

class RobotAgent:
    def __init__(self):
        self.llm = ChatOpenAI(model="gpt-4", temperature=0)
        # Define robot tools here (pseudo-code for demo)
        self.tools = [move_to_tool, pick_up_tool]

    def execute_mission(self, user_input):
        prompt = ChatPromptTemplate.from_messages([
            ("system", "You are a humanoid assistant. If a tool fails, reason about why and try an alternative."),
            ("human", "{input}"),
            ("placeholder", "{agent_scratchpad}"),
        ])
        # Core agent logic
        agent = create_openai_functions_agent(self.llm, self.tools, prompt)
        executor = AgentExecutor(agent=agent, tools=self.tools, verbose=True)
        return executor.invoke({"input": user_input})

# Example Mission Failure Handling:
# "The robot tried to move to the kitchen but detected a person in the way.
# The agent decided to wait 10 seconds and try again instead of giving up."
```

#### Step 3: Simulated Failure Test

Trigger a failure in your simulated robot (e.g., remove the navigation goal). Observe the terminal logs. Does the LLM suggest a new plan or explain the error clearly?

**Success Criteria**:
- [ ] Agent correctly decomposes the mission into tool calls.
- [ ] When a "Failure" state is manually injected, the agent suggests a logical next step (retry or alternative).
- [ ] Final plan is summarized in natural language for the user.

---

## Summary

**Key Concepts Covered**:
1. **Agentic Reasoning**: Moving beyond static scripts to dynamic planning.
2. **Tool Orchestration**: Connecting LLMs to physical ROS 2 actions.
3. **Error Resilience**: Using AI to handle real-world uncertainty and failures.

**Skills Acquired**:
- Integrating LangChain with robotics middleware.
- Designing feedback-driven system prompts.
- Troubleshooting multi-step autonomous missions.

**Connection to Next Chapter**: Reasoning is powerful, but it works better when the AI can "See" the situation directly in the prompt. In **Chapter 22: Multimodal Interaction**, we will learn how to feed live images directly into our cognitive planner.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which component allows the LLM to actually trigger a physical movement in ROS 2?
A) The Prompt
B) The Tool
C) The Tokenizer

**Question 2** (Difficulty: Medium)
Explain why a "Temperature" of 0 is usually preferred for robotic planners.

**Question 3** (Difficulty: Hard)
Describe how you would implement "Long-Term Memory" (e.g., remembering where the robot left its charging dock yesterday) using a vector database like Qdrant for your robot agent.

---

## Next Chapter

Continue to **[Chapter 22: Multimodal Interaction](./22-multimodal-interaction.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
