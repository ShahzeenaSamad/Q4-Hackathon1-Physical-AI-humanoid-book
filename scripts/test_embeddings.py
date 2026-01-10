"""
Quick test script to add sample textbook content to Qdrant
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

from src.services.cohere_client import cohere_client
from src.services.qdrant_service import qdrant_service
from qdrant_client.http import models
import uuid

def add_sample_content():
    """Add sample Physical AI textbook content to Qdrant"""

    sample_chapters = [
        {
            "chapter_id": "module-1-chapter-01",
            "title": "Chapter 1: Introduction to Physical AI",
            "module": "Module 1: ROS 2 Fundamentals",
            "content": """Physical AI refers to artificial intelligence systems that interact with the physical world through embodied agents like robots.
            Unlike traditional digital AI that processes data in virtual environments, Physical AI must handle real-world complexities such as
            sensor noise, actuator delays, and dynamic environments. Humanoid robotics is a key application area of Physical AI, where robots
            are designed with human-like form factors to operate in human-centric environments. Key challenges include perception, manipulation,
            locomotion, and human-robot interaction.""",
            "estimated_time": "1.5 hours"
        },
        {
            "chapter_id": "module-1-chapter-02",
            "title": "Chapter 2: Introduction to ROS 2",
            "module": "Module 1: ROS 2 Fundamentals",
            "content": """ROS 2 (Robot Operating System 2) is an open-source middleware framework for building robot applications.
            It provides tools and libraries for hardware abstraction, device drivers, communication between processes, package management,
            and more. ROS 2 uses a Data Distribution Service (DDS) for communication, which enables real-time, reliable messaging between nodes.
            Key concepts include nodes (processes), topics (message streams), services (request-response), and actions (long-running goals with feedback).
            ROS 2 Humble is the recommended LTS release for 2024-2025.""",
            "estimated_time": "2.5 hours"
        },
        {
            "chapter_id": "module-1-chapter-03",
            "title": "Chapter 3: ROS 2 Nodes and Topics",
            "module": "Module 1: ROS 2 Fundamentals",
            "content": """In ROS 2, a node is an executable process that performs computation. Nodes communicate via topics using a publish-subscribe pattern.
            A publisher node sends messages to a topic, and subscriber nodes receive those messages. Topics are typed - each topic has a specific message type.
            Common message types include std_msgs (String, Int32, Float64), geometry_msgs (Pose, Twist, Transform), and sensor_msgs (Image, LaserScan, Imu).
            You can create custom message types using .msg files. Quality of Service (QoS) policies control message reliability and delivery.""",
            "estimated_time": "2 hours"
        },
        {
            "chapter_id": "module-2-chapter-09",
            "title": "Chapter 9: Introduction to Simulation",
            "module": "Module 2: Gazebo & Unity Simulation",
            "content": """Robot simulation enables testing algorithms without physical hardware, reducing costs and risks.
            Gazebo is a popular open-source 3D robot simulator that provides physics simulation, sensor simulation, and ROS integration.
            Unity offers photorealistic rendering and is useful for visual perception tasks. Isaac Sim provides GPU-accelerated physics and
            advanced sensor simulation. Simulation fidelity vs. performance is a key tradeoff - high-fidelity physics simulations are computationally expensive.""",
            "estimated_time": "1.5 hours"
        },
        {
            "chapter_id": "module-3-chapter-14",
            "title": "Chapter 14: Introduction to NVIDIA Isaac",
            "module": "Module 3: NVIDIA Isaac Platform",
            "content": """NVIDIA Isaac Sim is a robot simulation platform built on Omniverse, providing photorealistic rendering and GPU-accelerated physics.
            Isaac ROS includes optimized perception packages for VSLAM, depth estimation, and object detection, leveraging CUDA acceleration on NVIDIA GPUs.
            Isaac platform is designed for AI-powered robots, with tight integration with deep learning frameworks. System requirements include RTX GPUs
            (RTX 4070 Ti or better recommended), though cloud alternatives exist via AWS or Omniverse Cloud.""",
            "estimated_time": "2 hours"
        },
        {
            "chapter_id": "module-4-chapter-19",
            "title": "Chapter 19: Introduction to Vision-Language-Action (VLA)",
            "module": "Module 4: Vision-Language-Action Models",
            "content": """Vision-Language-Action (VLA) models combine computer vision, natural language processing, and robotic control in a unified framework.
            These models enable robots to understand high-level instructions like 'clean the table' and translate them into low-level actions.
            Large Language Models (LLMs) like GPT-4 serve as cognitive planners, decomposing complex tasks into executable subtasks.
            Examples include Google's RT-2 and PaLM-E, which demonstrate impressive generalization to new tasks and environments.""",
            "estimated_time": "1.5 hours"
        }
    ]

    print("Generating embeddings for sample chapters...")
    points = []

    for idx, chapter in enumerate(sample_chapters):
        # Generate embedding
        embedding = cohere_client.get_embeddings(
            [chapter["content"]],
            input_type="search_document"
        )[0]

        # Create Qdrant point
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "chapter_id": chapter["chapter_id"],
                "title": chapter["title"],
                "module": chapter["module"],
                "content": chapter["content"],
                "estimated_time": chapter["estimated_time"]
            }
        )
        points.append(point)
        print(f"[OK] Generated embedding for {chapter['title']}")

    # Upload to Qdrant
    print(f"\nUploading {len(points)} embeddings to Qdrant...")
    qdrant_service.upsert_chunks(points)
    print("[OK] Successfully uploaded embeddings to Qdrant!")

    # Verify
    collection_info = qdrant_service.client.get_collection(qdrant_service.collection_name)
    print(f"\nQdrant Collection Status:")
    print(f"  - Collection name: {collection_info.name}")
    print(f"  - Total points: {collection_info.points_count}")
    print(f"  - Status: {collection_info.status}")
    print("\n[SUCCESS] Embeddings are ready! You can now test the chatbot.")

if __name__ == "__main__":
    add_sample_content()
