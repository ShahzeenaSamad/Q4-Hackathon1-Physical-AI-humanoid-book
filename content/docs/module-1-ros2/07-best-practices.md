# Chapter 7: ROS 2 Best Practices

**Module**: Module 1: ROS 2 Fundamentals
**Estimated Time**: 2.0 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Chapter 6: Understanding URDF

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Organize** ROS 2 packages using standard directory structures and metadata.
2. **Implement** error handling and logging strategies for robust robot operations.
3. **Configure** Quality of Service (QoS) policies for different network conditions.
4. **Apply** unit testing and linting to ensure code quality and reliability.

---

## Introduction

Building a robot is not just about making it move; it's about making it move reliably, every time. As your codebase grows from a few scripts to hundreds of nodes, following **Best Practices** becomes essential for maintenance, collaboration, and safety.

**Topics covered in this chapter**:
- Package standard organization
- Logging with `rcutils` and `rclpy`
- Understanding Quality of Service (QoS)
- Testing with `pytest` and `launch_testing`

**Why this matters**: In Physical AI, a software bug can cause physical damage. Standardized package structures make it easier for other engineers to understand your code, while proper logging and testing help catch critical failures before they happen in the real world.

**Example Use Case**: A humanoid robot operating over a shaky Wi-Fi connection needs a specific QoS policy to ensure that "Emergency Stop" commands are always received, even if some sensor data packets are lost.

---

## Core Content

### Section 1: Package Organization

A well-structured ROS 2 package makes discovery and deployment seamless.

#### Standard Directory Layout
- `package.xml`: Metadata about the package (version, maintainer, dependencies).
- `setup.py` (Python) or `CMakeLists.txt` (C++): Build instructions.
- `launch/`: Where automation scripts live.
- `urdf/`: Robot descriptions.
- `config/`: Parameters and tuning files.
- `test/`: Unit and integration tests.

---

### Section 2: Logging and Diagnostics

ROS 2 provides a tiered logging system to categorize information.

#### Severity Levels
1. **DEBUG**: Detailed information for development.
2. **INFO**: General system status.
3. **WARN**: Something unexpected happened, but the robot can continue.
4. **ERROR**: A problem occurred that might affect part of the system.
5. **FATAL**: Critical failure; system should shut down.

#### Example 7.1: Logging in Python

```python
self.get_logger().info('Initializing joint controller...')
if battery < 20:
    self.get_logger().warn('Low battery detected!')
```

---

### Section 3: Quality of Service (QoS)

Not all data is created equal. QoS allows you to tune how nodes communicate.

#### Key Policies
- **Reliability**:
  - *Reliable*: Guarantees packet delivery (like TCP).
  - *Best Effort*: Faster, but packets might be lost (like UDP).
- **Durability**:
  - *Transient Local*: New subscribers get the last published message.
  - *Volatile*: New subscribers only get messages published after they join.

#### Visual Aid

![Figure 7.1: QoS Policy Comparison](../../../static/img/module-1/qos-policies.png)

*Figure 7.1: Choosing between Reliable (for commands) and Best Effort (for high-frequency sensors).*

---

## Hands-On Exercise

### Exercise 7: Adding Reliability and Tests to the Status Package

**Objective**: Refactor your chapter 3 status package to include standard logging, a specific QoS policy, and a basic unit test.

**Estimated Time**: 75 minutes

#### Step 1: Update the Publisher with QoS

Modify `heartbeat_publisher.py` to use a `Reliable` QoS profile.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.publisher_ = self.create_publisher(String, 'robot_heartbeat', qos_profile)
```

#### Step 2: Implement a Unit Test

Create `robot_status_pkg/test/test_logic.py`:

```python
# File: test_logic.py
import pytest

def test_heartbeat_string_format():
    # Example logic test: ensure our status string starts correctly
    msg_data = "Robot Active: Heartbeat 5"
    assert msg_data.startswith("Robot Active")
    assert "Heartbeat" in msg_data
```

#### Step 3: Run the Tests

```bash
# In your workspace root
colcon test --packages-select robot_status_pkg
colcon test-result --all
```

**Success Criteria**:
- [ ] Publisher starts with custom QoS policy.
- [ ] `colcon test` returns 0 failures.

---

## Summary

**Key Concepts Covered**:
1. **Metadata**: Using `package.xml` to manage dependencies.
2. **Robustness**: Using tiered logging to diagnose issues.
3. **Networking**: Tuning communication with QoS.

**Skills Acquired**:
- Organizing a production-ready ROS 2 package.
- Implementing Reliable vs. Best Effort communication.
- Writing and running Python unit tests for robotics logic.

**Connection to Next Chapter**: You've mastered the fundamentals of the "Robotic Nervous System." In **Chapter 8: Module 1 Summary**, we'll review everything and prepare for the next module: **Simulation**.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which logging level should you use for information that is only needed for fixing bugs?
A) INFO
B) WARN
C) DEBUG

**Question 2** (Difficulty: Medium)
When would you choose `Best Effort` reliability for a camera node over `Reliable`?

**Question 3** (Difficulty: Hard)
Explain the purpose of the `package.xml` file in a ROS 2 system.

---

## Next Chapter

Continue to **[Chapter 8: Module 1 Summary](./08-module-summary.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
