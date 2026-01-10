"""Test Cohere API"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

from src.services.cohere_client import cohere_client

print("[INFO] Testing Cohere API...")

try:
    # Test embeddings
    print("\n[1] Testing embeddings...")
    embeddings = cohere_client.get_embeddings(["test"], input_type="search_query")
    print(f"[OK] Got embedding with dimension: {len(embeddings[0])}")

    # Test generation
    print("\n[2] Testing chat generation...")
    response = cohere_client.generate_response("Say hello in 3 words")
    print(f"[OK] Got response: {response}")

    print("\n[SUCCESS] All Cohere tests passed!")

except Exception as e:
    print(f"\n[ERROR] Cohere test failed: {e}")
    import traceback
    traceback.print_exc()
