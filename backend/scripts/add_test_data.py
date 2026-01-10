"""Add minimal test data to Qdrant for chatbot testing"""
import os
import sys
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.services.cohere_client import cohere_client
from src.services.qdrant_service import qdrant_service
from qdrant_client.http import models

# Sample textbook content
test_documents = [
    {
        "title": "Introduction to Physical AI",
        "module": "module-1-ros2",
        "chapter_id": "01",
        "content": """Physical AI combines artificial intelligence with physical embodiment in robots and autonomous systems.
        It enables machines to interact with and learn from the physical world. Key applications include humanoid robots,
        autonomous vehicles, and intelligent manufacturing systems. Physical AI requires tight integration between perception,
        planning, and actuation systems."""
    },
    {
        "title": "ROS 2 Fundamentals",
        "module": "module-1-ros2",
        "chapter_id": "02",
        "content": """ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.
        It provides services like hardware abstraction, device drivers, libraries, visualizers, and message-passing between processes.
        ROS 2 improves upon ROS 1 with better real-time performance, security, and multi-robot support."""
    },
    {
        "title": "Isaac Sim Overview",
        "module": "module-3-isaac",
        "chapter_id": "01",
        "content": """NVIDIA Isaac Sim is a robotics simulation platform built on Omniverse.
        It provides physically accurate simulation for robot development and testing. Features include photorealistic rendering,
        physics simulation, sensor simulation, and integration with ROS 2. Isaac Sim accelerates robot development by enabling
        safe testing of algorithms in virtual environments."""
    }
]

def main():
    print("Adding test data to Qdrant...")

    points = []
    for doc in test_documents:
        # Generate embedding
        print(f"Generating embedding for: {doc['title']}")
        embedding = cohere_client.get_embeddings([doc['content']], input_type="search_document")[0]

        # Create point
        point_id = str(uuid.uuid4())
        points.append(
            models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": doc["content"],
                    "title": doc["title"],
                    "module": doc["module"],
                    "chapter_id": doc["chapter_id"]
                }
            )
        )

    # Upload points one at a time to avoid timeout
    for point in points:
        print(f"Upserting: {point.payload['title']}")
        try:
            qdrant_service.upsert_chunks([point])
            print(f"  [OK] Success")
        except Exception as e:
            print(f"  [ERROR] Error: {e}")

    print(f"\nDone! Added {len(points)} test documents")

if __name__ == "__main__":
    main()
