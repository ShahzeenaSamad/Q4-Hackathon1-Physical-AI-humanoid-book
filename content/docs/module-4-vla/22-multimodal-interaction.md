# Chapter 22: Multimodal Interaction

**Module**: Module 4: Vision-Language-Action (VLA)
**Estimated Time**: 2.5 hours (Reading: 60 min, Hands-on: 90 min)
**Prerequisites**: Chapter 21: Cognitive Planning with LLMs

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** a multimodal data pipeline that fuses vision (images) and language (text) for unified robotic reasoning.
2. **Utilize** Vision-Language Models (VLMs) like GPT-4o or LLaVA to ground language instructions in visual scenes.
3. **Design** human-robot interaction (HRI) scenarios where the robot provides verbal feedback based on what it "sees" in real-time.
4. **Configure** cross-modal embedding spaces to improve object identification and task grounding accuracy.

---

## Introduction

So far, our robot has been "blindly" following text plans or using separate vision nodes. A truly intelligent agent, however, sees the world and reasons about it simultaneously. This is the essence of **Multimodal Interaction**.

Instead of saying "Go to the kitchen," a human might point and say "Go over there." Or, when asked "What's wrong?", the robot should be able to look at its own arm, see that it's stuck in a door, and explain the physical situation using language.

**Topics covered in this chapter**:
- Vision-Language Models (VLM): The new interface
- Semantic Grounding: Linking words to pixels
- Visual Feedback Loops: Self-correcting via observation
- Building an "Interactive Describer" node

**Why this matters**: Multimodal reasoning allows robots to handle ambiguity. If you say "Get the mug" and there are three mugs, a multimodal robot can look at them and ask, "The red one, or the one with the handle on the left?", making it far more useful in collaborative human sessions.

**Example Use Case**: A humanoid assistant in a factory observes a worker struggling with a heavy box. The robot's multimodal brain processes the visual scene (worker's posture + box weight) and the auditory environment, and proactively asks, "Do you need help lifting that pallet?"

---

## Core Content

### Section 1: Vision-Language Models (VLMs)

VLMs are trained on billions of image-text pairs. They don't just see a "box"; they understand the concept of "fragile", "heavy", or "UPS box" based on visual context.

#### Key Patterns
- **Visual Question Answering (VQA)**: Input: (Image + "What is the robot holding?") -> Output: ("A wrench").
- **Image Captioning for State**: The robot describes its view to its own planner to update its "Mental Map."

---

### Section 2: Semantic Grounding

Grounding is the process of mapping a word (e.g., "Screwdriver") to a specific 3D coordinate or bounding box in the camera frame.

#### The Multimodal Process
1. **Input**: User says "Move the left screwdriver 10cm back."
2. **Vision Check**: VLM identifies all screwdrivers and identifies the one on the left.
3. **Coordinate Extraction**: The 2D pixels are mapped to 3D via the depth camera (Isaac ROS).
4. **Action**: Motor commands are sent.

---

## Hands-On Exercise

### Exercise 22: The "What do you see?" Robot Node

**Objective**: Create a ROS 2 node that captures a camera frame from Isaac Sim and uses a VLM to describe the scene in Urdu and English.

**Estimated Time**: 90 minutes

#### Step 1: Capture and Encode Image

To send images to a VLM API, we need to convert the ROS 2 Image type to a Base64 string.

```python
# Environment: Python 3.10
import base64
import cv2
from cv_bridge import CvBase64

def encode_image(cv_image):
    _, buffer = cv2.imencode('.jpg', cv_image)
    return base64.b64encode(buffer).decode('utf-8')
```

#### Step 2: Implement the Multimodal Explainer

Create `vla_pkg/multimodal_describer.py`:

```python
# Environment: Python + OpenAI GPT-4o
class SceneExplainer(Node):
    def __init__(self):
        super().__init__('scene_explainer')
        self.subscription = self.create_subscription(Image, '/camera/rgb', self.image_callback, 10)

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        base64_image = encode_image(cv_img)

        # Call the VLM
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": "What are you looking at? Answer in Urdu and English."},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                ]
            }]
        )
        self.get_logger().info(f"AI View: {response.choices[0].message.content}")
```

#### Step 3: Verify Grounding

Place two different colored objects in your Gazebo/Isaac Sim scene. Ask the robot to "Identify the object on the right." Verify that the VLM's natural language response matches the physical arrangement.

**Success Criteria**:
- [ ] Node successfully captures and encodes frames from simulation.
- [ ] VLM accurately describes the scene in both languages.
- [ ] Spatial reasoning (left/right/above) is correct based on the visual input.

---

## Summary

**Key Concepts Covered**:
1. **Multimodality**: The fusion of vision and language at the model level.
2. **State Description**: Using AI to verbalize current physical conditions.
3. **Feedback loops**: Correcting robot actions based on real-time visual reasoning.

**Skills Acquired**:
- Processing and sending robotic image data to cloud VLMs.
- Designing interactive HRI flows.
- Managing high-latency API calls within real-time ROS 2 loops.

**Connection to Next Chapter**: We have all the pieces: ROS 2 (Nerves), Sim (Environment), Isaac (Vision), and VLA (Brain). In **Chapter 23: Capstone Project**, we will combine them into a single, fully autonomous humanoid agent.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which model type is trained to handle both text and images in a single request?
A) LLM (Large Language Model)
B) VLM (Vision-Language Model)
C) CNN (Convolutional Neural Network)

**Question 2** (Difficulty: Medium)
Why is "Grounding" difficult for a robot when using only a standard 2D RGB camera compared to an RGB-D (Depth) camera?

**Question 3** (Difficulty: Hard)
How would you optimize the Multimodal pipeline to ensure the robot doesn't stand still for 5 seconds every time it needs to "look" at its environment?

---

## Next Chapter

Continue to **[Chapter 23: Capstone Project: The Autonomous Humanoid](./23-capstone-project.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
