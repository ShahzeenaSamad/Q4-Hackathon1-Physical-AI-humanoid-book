# Chapter 4: Services and Actions

**Module**: Module 1 - ROS 2 Fundamentals
**Estimated Time**: 2.5 hours (Reading: 45 min, Hands-on: 105 min)
**Prerequisites**: Chapter 3 (Nodes and Topics)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** a ROS 2 Service (Request-Response) for synchronous robot tasks.
2. **Design** a ROS 2 Action (Goal-Feedback-Result) for long-running asynchronous behaviors.
3. **Differentiate** between Topics, Services, and Actions based on specific robotic use cases.
4. **Utilize** CLI tools to interact with service servers and action clients.

**Bloom's Taxonomy Levels**: Apply (implement, design), Analyze (differentiate), Remember (utilize)

---

## Introduction

In Chapter 3, we used Topics for streaming data. Topics are great for continuous flows, but they lack a direct feedback mechanism. Sometimes you need to ask a specific question and wait for an answer (**Services**), or you need to give a complex command and get continuous feedback while the task is executing (**Actions**).

**Topics covered in this chapter**:
- Services: The Request-Response pattern
- Actions: Managing complex, long-duration tasks
- Comparing communication patterns
- Practical implementation in Python

**Why this matters**: Imagine telling a humanoid robot to "Walk to the kitchen." If you use a Topic, you have no way of knowing if it actually arrived or how far it has gone. Using an Action allows the robot to send back "I'm 50% there" (Feedback) and eventually a "Task Success" notification (Result).

**Example Use Case**:
- **Service**: A camera node asks the `/exposure_control` service to set the exposure to '100ms' and waits for a "Success" confirmation before taking the next picture.
- **Action**: A navigation node sends a goal to a mobile robot to "Navigate to Lab A." The robot provides continuous distance-to-goal updates (Feedback) until it reaches the destination.

---

## Core Content

### Section 1: ROS 2 Services

Services follow a **Client-Server** model. A client sends a **Request**, and the server sends a **Response**. This is usually synchronous: the client waits (blocks) until the server completes its task.

#### Characteristics
- **Synchronous**: Useful for quick transitions or system changes.
- **Many-to-One**: Multiple clients can call the same service, but only one server can exist.
- **Stateless**: Each request is handled independently.

#### Example 4.1: Calling a Service via CLI
```bash
# Syntax: ros2 service call <name> <type> <args>
ros2 service call /clear std_srvs/srv/Empty {}
```

---

### Section 2: ROS 2 Actions

Actions are designed for "long-running" tasks that might take many seconds or even minutes to complete. They use an asynchronous, three-part message structure:
1.  **Goal**: The initial command (e.g., "Drive 10 meters").
2.  **Feedback**: Continuous updates (e.g., "Distance driven: 2m, 4m, 6m...").
3.  **Result**: The final status (e.g., "Arrived at destination").

#### Characteristics
- **Asynchronous**: The client can do other work while the action runs.
- **Preemptible**: You can cancel an action if the situation changes (e.g., stopping a robot because an obstacle appeared).
- **Complex**: Combines Topic-like feedback with Service-like goal setting.

---

### Section 3: When to Use What?

| Requirement | **Topics** | **Services** | **Actions** |
| :--- | :--- | :--- | :--- |
| **Communication** | Async Stream | Sync Call/Response | Async Task Management |
| **Feedback** | None | Immediate | Continuous + Final |
| **Preemption** | No | No | Yes (Cancellable) |
| **Latency** | Low (continuous) | Medium (blocking) | High (long-running) |

---

## Hands-On Exercise

### Exercise 4: Building a Robot Command Center

**Objective**: Implement a Service for toggling a robot's light and an Action for a simulated long-running task.

**Estimated Time**: 105 minutes

#### Step 1: Create the Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_command_center --dependencies rclpy example_interfaces
```

#### Step 2: Implement a Square Calculation Service
Create `robot_command_center/square_service.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Re-using for simplicity

class SquareService(Node):
    def __init__(self):
        super().__init__('square_service')
        self.srv = self.create_service(AddTwoInts, 'sum_ints', self.calc_callback)

    def calc_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b}')
        return response

def main():
    rclpy.init()
    rclpy.spin(SquareService())
    rclpy.shutdown()
```

#### Step 3: Implement a Simple Timer Action
Actions use `.action` files. For this demo, we'll use a built-in type. Create `robot_command_center/timer_action.py`:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci # Repurposed for demo sequence

class TimerActionServer(Node):
    def __init__(self):
        super().__init__('timer_action_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'countdown', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()

        # Simulate a 1-second countdown loop
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(i)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Step {i} complete...')
            time.sleep(1.0)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TimerActionServer())
    rclpy.shutdown()
```

#### Step 4: Verify via CLI
1.  **Service**: `ros2 service call /sum_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"`
2.  **Action**: `ros2 action send_goal /countdown example_interfaces/action/Fibonacci "{order: 5}" --feedback`

---

## Summary

**Key Concepts**:
1.  **Services**: Used for quick, stateless request-response interaction.
2.  **Actions**: Used for stateful, long-duration tasks requiring feedback and the ability to cancel.
3.  **Blocking**: How client nodes behave while waiting for services vs. actions.

**Skills Acquired**:
- Implementing service servers with custom request handling.
- Designing action servers that publish continuous feedback.
- Using CLI tools to trigger and monitor robot tasks.

---

## Assessment Questions

**Question 1** (Difficulty: Easy)
Which CLI command is used to see the list of available services?
- A) `ros2 node list`
- B) `ros2 service list`
- C) `ros2 topic list`

**Question 2** (Difficulty: Medium)
Why is an Action more suitable for "Move Robot to Room 1" than a Service?

**Question 3** (Difficulty: Hard)
Scenario: You have a real-time motor controller running at 1000Hz (1ms loops). Can you call a blocking Service inside this loop? Explain the impact on the robot's hardware.

---

## Next Chapter

Continue to **[Chapter 5: Launch Files and Parameters](/docs/module-1-ros2/launch-files)**.

---

**Revision History**:
- **Version 1.0** (2025-12-31): Initial release.
