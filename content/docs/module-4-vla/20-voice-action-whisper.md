# Chapter 20: Voice-to-Action with Whisper

**Module**: Module 4: Vision-Language-Action (VLA)
**Estimated Time**: 2.0 hours (Reading: 45 min, Hands-on: 75 min)
**Prerequisites**: Chapter 19: Introduction to VLA Models

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Integrate** OpenAI's Whisper model into a ROS 2 pipeline for real-time speech-to-text.
2. **Handle** audio streaming and processing in Python to optimize transcription latency.
3. **Map** transcribed text in multiple languages (English/Urdu) to unified robotic commands.
4. **Implement** error handling for noisy environments using confidence thresholds and verbal confirmation loops.

---

## Introduction

In the previous chapter, we learned how to plan tasks using text. But a physical humanoid shouldn't require a keyboard. We need a way to **Talk** to our robots.

**Whisper** is a state-of-the-art automatic speech recognition (ASR) model that can transcribe speech in dozens of languages with high accuracy, even in noisy environments. By bridging Whisper with our ROS 2 nodes, we create a "Voice-to-Action" pipeline.

**Topics covered in this chapter**:
- How Whisper works: The Encoder-Decoder architecture
- Streaming Audio in ROS 2: Choosing the right message type
- Multilingual Support: Transcribing Urdu instructions for the robot
- Building the "Command Filter": From text to structured tool calls

**Why this matters**: Voice control is the most intuitive interface for Human-Robot Interaction (HRI). For service humanoids in homes or hospitals, being able to understand a patient's spoken request—regardless of their accent or language—is a mandatory safety and usability feature.

**Example Use Case**: An elderly person says, *"Mera pani le aao"* (Bring my water) in Urdu. The robot's Whisper node transcribes this, the LLM-planner translates the intent into a navigation and pick-up goal, and the robot executes the task.

---

## Core Content

### Section 1: The Voice-to-Action Pipeline

The pipeline consists of four stages:
1. **Audio Capture**: Microphones on the humanoid capture a raw audio stream.
2. **Transcription**: Whisper converts the audio into text (either locally or via API).
3. **Intent Parsing**: The LLM (from Chapter 19) extracts the command from the text.
4. **Execution**: ROS 2 Service/Action calls move the hardware.

---

### Section 2: Audio Streaming in ROS 2

Audio data is large and high-frequency. We typically use the `audio_common_msgs/msg/AudioData` type to stream small chunks of bytes to a processing node.

#### Key Challenges
- **Latency**: We want the robot to respond in <2 seconds.
- **VAD (Voice Activity Detection)**: Detecting when the user starts and stops talking so we don't send silence to Whisper.

---

## Hands-On Exercise

### Exercise 20: Building a Voice-Controlled Robot "Mover"

**Objective**: Create a ROS 2 node that transcribes your voice and publishes a command to move the simulated robot.

**Estimated Time**: 75 minutes

#### Step 1: Install Whisper Python Client

```bash
# Install the whisper package (requires FFmpeg)
pip install openai-whisper
sudo apt update && sudo apt install ffmpeg
```

#### Step 2: Write the Whisper Transcriber Node

Create `vla_pkg/voice_command_node.py`:

```python
# Environment: Python 3.10 + Whisper
import rclpy
from rclpy.node import Node
import whisper
import numpy as np

class VoiceCommander(Node):
    def __init__(self):
        super().__init__('voice_commander')
        self.model = whisper.load_model("base") # Load small model for speed
        self.get_logger().info("Whisper Model Loaded. Ready for commands...")

    def process_audio(self, audio_data: np.ndarray):
        # Audio must be 16kHz mono
        result = self.model.transcribe(audio_data, fp16=False)
        text = result["text"].strip().lower()
        self.get_logger().info(f"Transcribed: {text}")

        if "forward" in text:
            self.publish_move_cmd(1.0)
        elif "back" in text:
            self.publish_move_cmd(-1.0)

def main():
    rclpy.init()
    node = VoiceCommander()
    # In a real app, this would be fueled by a subscriber to audio-topic
    rclpy.spin(node)
```

#### Step 3: Multilingual Test (Urdu)

Record yourself saying *"Aage jao"* (Move forward). Observe how Whisper (if configured for Urdu) transcribes the characters. You can then add a mapping like:
```python
if "aage" in text or "forward" in text:
    self.publish_move_cmd(1.0)
```

**Success Criteria**:
- [ ] Whisper model loads successfully in the ROS 2 node.
- [ ] Node correctly transcribes "Move forward" into text.
- [ ] Movement command is successfully triggered based on the specific words detected.

---

## Summary

**Key Concepts Covered**:
1. **ASR (Automatic Speech Recognition)**: Converting wave data to text strings.
2. **Multilingual HRI**: Enabling robots to understand non-English speakers.
3. **Thresholding**: Ensuring only confident transcriptions trigger physical actions.

**Skills Acquired**:
- Integrating AI models directly into Python ROS 2 nodes.
- Managing audio buffers and signal processing requirements.
- Designing error-resilient voice command systems.

**Connection to Next Chapter**: Now that we can plan and speak, we need to handle "Complex" reasoning. In **Chapter 21: Cognitive Planning with LLMs**, we'll learn how to use LangChain to manage long-term memory and complex problem-solving.

---

## Assessment Questions

### Self-Check Questions

**Question 1** (Difficulty: Easy)
Which library is used to convert deep learning models into conversational text for the robot?
A) Gazebo
B) Whisper
C) RViz

**Question 2** (Difficulty: Medium)
Why should we use the "Base" or "Tiny" Whisper models for a real-time robot instead of the "Large" model?

**Question 3** (Difficulty: Hard)
How would you implement a "Confirmation Loop" to prevent the robot from executing a dangerous command based on a mis-transcription? Use pseudocode or a flow diagram.

---

## Next Chapter

Continue to **[Chapter 21: Cognitive Planning with LLMs](./21-cognitive-planning-llms.md)**.

---

**Revision History**:
- **Version 1.0** (2025-12-30): Initial release.
