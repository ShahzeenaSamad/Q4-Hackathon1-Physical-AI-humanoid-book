# Chapter 4: ROS 2 Services and Actions

**Module**: Module 1: ROS 2 Fundamentals
**Estimated Time**: 2.5 hours (Reading: 45 min, Hands-on: 105 min)
**Prerequisites**: Chapter 3: ROS 2 Nodes and Topics

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** a ROS 2 Service (Request-Response) for synchronous robot tasks.
2. **Design** a ROS 2 Action (Goal-Feedback-Result) for long-running asynchronous behaviors.
3. **Differentiate** between Topics, Services, and Actions based on specific robotic use cases.
4. **Utilize** CLI tools to interact with service servers and action clients.

---

## Introduction

In Chapter 3, we used Topics for streaming data. However, robotics often requires more structured communication. Sometimes you need to ask a question and wait for a specific answer (**Services**), or you need to give a complex command and get feedback while it's running (**Actions**).

**Topics covered in this chapter**:
- Services: The Request-Response pattern
- Actions: Managing complex, long-duration tasks
- Comparing communication patterns
- Practical implementation in Python

**Why this matters**: Imagine telling a humanoid robot to "Walk to the kitchen." If you use a Topic, you have no way of knowing if it actually arrived or how far it has gone. Using an Action allows the robot to send back "I'm 50% there" and eventually a "Task Success" notification.

**Example Use Case**:
- **Service**: A node asks the `/battery_info` service for the current voltage and waits for the exact decimal value.
- **Action**: A node sends a goal to the `/navigation` action server to "Go to Room 101." The robot sends continuous XY coordinates (feedback) until it reaches the goal.

---

## Core Content

### Section 1: ROS 2 Services

Services follow a **Client-Server** model. A client sends a **Request**, and the server sends a **Response**. This is synchronous: the client usually waits (blocks) until the server is done.

#### Key Points
- **Synchronous**: Useful for quick tasks (e.g., toggling an LED, calculating a math function).
- **One-to-One**: A specific request goes to a specific server.

#### Example 4.1: Calling a Service via CLI

```bash
# General syntax: ros2 service call <service_name> <service_type> <arguments>
ros2 service call /clear std_srvs/srv/Empty {}
```

---

### Section 2: ROS 2 Actions

Actions are used for tasks that take time. They consist of three parts: a **Goal**, **Feedback**, and a **Result**.

#### Key Points
- **Asynchronous**: The client can do other things while the action is running.
- **Preemptible**: You can cancel an action while it's in progress (e.g., "Stop walking!").
- **Feedback**: The server sends regular updates (e.g., "Distance remaining: 1.2m").

#### Visual Aid

![Figure 4.1: Communication Comparison](../../../static/img/module-1/communication-types.png)

*Figure 4.1: Differences between Topics (Stream), Services (Query), and Actions (Task).*

---

## Hands-On Exercise

### Exercise 4: Building a Robotic Calculation Service and Move Action

**Objective**: Implement a Service for math and an Action for a simulated robot movement.

**Estimated Time**: 105 minutes

#### Step 1: Implement an AddTwoInts Service

Create `robot_status_pkg/add_ints_server.py`:

```python
# File: add_ints_server.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

def main():
    rclpy.init()
    node = MinimalService()
    rclpy.spin(node)
    rclpy.shutdown()
```

#### Step 2: Implement a Simple Timer Action (Simulated Movement)

Create `robot_status_pkg/move_action_server.py`:

```python
# File: move_action_server.py
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci # We'll repurpose Fibonacci for demo

class MoveActionServer(Node):
    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'move_robot',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()

        # Simulate movement progress
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(i)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {i}% progress')
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveActionServer()
    rclpy.spin(node)
```

#### Step 3: Interaction via CLI

After building and running the servers:

**Test Service**:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
```

**Test Action**:
```bash
ros2 action send_goal /move_robot example_interfaces/action/Fibonacci "{order: 5}" --feedback
```

**Success Criteria**:
- [ ] Service returns `sum: 15`.
- [ ] Action prints feedback (1%, 2%...) and finally returns the sequence.

---

## Summary

**Key Concepts Covered**:
1. **Services**: Best for immediate request-response.
2. **Actions**: Best for reliable, cancellable, long-running tasks.
3. **Preemption**: The ability to cancel a robot's current goal.

**Skills Acquired**:
- Writing Service Servers in Python.
- Writing Action Servers with Feedback loops.
- Interacting with Services and Actions using the command line.

**Connection to Next Chapter**: Managing dozens of nodes, services, and actions manually is difficult. In **Chapter 5: Launch Files and Parameters**, we'll learn how to orchestrate the entire system with a single command.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which communication pattern should you use for a "Get Battery Level" command?
A) Topic
B) Service
C) Action

**Question 2** (Difficulty: Medium)
What are the three main messages sent in a ROS 2 Action?

**Question 3** (Difficulty: Hard)
Why is it dangerous to use a blocking Service call for a task that might take 10 seconds in a real-time gait control node?

---

## Next Chapter

Continue to **[Chapter 5: Launch Files and Parameters](./05-launch-files.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
