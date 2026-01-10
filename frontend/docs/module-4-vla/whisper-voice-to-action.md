# Chapter 20: Voice-to-Action with Whisper

**Module**: Module 4 - Vision-Language-Action (VLA)
**Estimated Time**: 1.5 hours (Reading: 30 min, Hands-on: 60 min)
**Prerequisites**: Chapter 19 (Introduction to VLA), Module 1 (ROS 2)

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Integrate** OpenAI Whisper for real-time speech-to-text (STT) on a robot
2. **Convert** transcribed text into structured ROS 2 commands
3. **Handle** background noise and audio artifacts in robotic environments
4. **Implement** a voice-feedback loop using Text-to-Speech (TTS)
5. **Analyze** the latency impact of local vs. cloud-based speech processing

---

## Introduction

Communication is the key to collaboration. For a Physical AI agent to work alongside humans, it must be able to listen to instructions and respond verbally. **OpenAI Whisper** is a state-of-the-art speech recognition model that is particularly robust to accents and background noise—making it ideal for busy robotics labs or industrial floors.

In this chapter, we will build a "Voice Control" pipeline that allows our robot to hear a command, interpret it, and execute it via ROS 2.

---

## Core Content

### Section 1: The Whisper Pipeline

To give our robot "ears," we need an audio processing pipeline:
1. **Microphone Input**: Capture raw audio using Python (e.g., `PyAudio` or `Sounddevice`).
2. **VAD (Voice Activity Detection)**: Detect when a human starts and stops speaking so we don't process silence.
3. **Transcription (Whisper)**: Convert the raw audio into a text string.
4. **Command Mapping**: Use keywords or an LLM to map the text to a ROS 2 Action or Topic.

### Section 2: Local vs. Cloud STT

- **Local (Whisper.cpp / Faster-Whisper)**: Best for privacy, low latency, and operation in areas without internet. Requires a decent GPU.
- **Cloud (API)**: Best for maximum accuracy and lower local compute requirements. Introduces dependency on internet connectivity.

---

## Hands-On Exercise

### Exercise 20: Building a Voice-Controlled Robot

**Objective**: Create a ROS 2 node that subscribes to voice data and moves a robot.

1. Install `faster-whisper`:
   ```bash
   pip install faster-whisper
   ```
2. Create a script `voice_cmd_node.py` that listens to the microphone.
3. Use a small keyword mapper:
   ```python
   def map_to_cmd(text):
       if "move forward" in text.lower():
           return "cmd_vel_forward"
       elif "stop" in text.lower():
           return "cmd_vel_stop"
   ```
4. Publish a `Twist` message to the robot based on the detected command.
5. Test it in Gazebo—shout "Move forward!" and watch your robot move.

---

## Assessment Questions

1. Why is Whisper better for robotics than older speech-to-text models?
2. What is Voice Activity Detection (VAD), and why is it used?
3. What is the biggest challenge of using voice commands in a noisy factory environment?

---

## Summary
Hearing is just the first step. In the next chapter, we will use the power of Large Language Models (LLMs) to go beyond simple keywords and enable our robot to perform complex, multi-step cognitive planning.
