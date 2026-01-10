"""Test Qdrant connection"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

from src.services.qdrant_service import qdrant_service

try:
    print("[INFO] Testing Qdrant connection...")

    # Get collection info
    info = qdrant_service.client.get_collection(qdrant_service.collection_name)
    print(f"[OK] Connected to Qdrant!")
    print(f"  - Collection: {qdrant_service.collection_name}")
    print(f"  - Points: {info.points_count}")
    print(f"  - Status: {info.status}")

    # Test search
    from src.services.cohere_client import cohere_client
    test_embedding = cohere_client.get_embeddings(["test"], input_type="search_query")[0]
    results = qdrant_service.search(test_embedding, limit=2)
    print(f"\n[OK] Search test successful! Found {len(results)} results")

except Exception as e:
    print(f"\n[ERROR] Qdrant test failed: {e}")
    import traceback
    traceback.print_exc()
