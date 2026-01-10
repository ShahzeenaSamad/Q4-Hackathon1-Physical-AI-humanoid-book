#!/usr/bin/env python3
"""
Generate embeddings for all textbook chapters and store in Qdrant

Usage:
    python scripts/generate_embeddings.py

This script:
1. Reads all markdown files from frontend/docs/
2. Generates embeddings using OpenAI
3. Stores embeddings in Qdrant vector store
"""

import sys
import os
from pathlib import Path

# Add backend src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.embedding_service import EmbeddingService
from src.services.vector_store import VectorStoreService


def extract_chapter_metadata(filepath: Path) -> dict:
    """Extract metadata from chapter markdown file"""
    content = filepath.read_text(encoding='utf-8')

    # Extract title (first # heading)
    title = "Untitled"
    for line in content.split('\n'):
        if line.startswith('# '):
            title = line.replace('# ', '').strip()
            break

    # Extract module from file path
    parts = filepath.parts
    module = "unknown"
    for part in parts:
        if part.startswith('module-'):
            module = part
            break

    # Generate chapter ID from filename
    chapter_id = filepath.stem  # filename without extension

    return {
        "chapter_id": f"{module}-{chapter_id}",
        "title": title,
        "module": module,
        "file_path": str(filepath)
    }


def main():
    print("=" * 70)
    print("Textbook Embeddings Generator")
    print("=" * 70)
    print()

    # Initialize services
    print("[1/4] Initializing services...")
    embedding_service = EmbeddingService()
    vector_store = VectorStoreService()
    print(f"  - Embedding model: {embedding_service.model}")
    print(f"  - Vector store: Qdrant ({vector_store.COLLECTION_NAME})")
    print()

    # Find all markdown chapter files
    print("[2/4] Scanning for chapter files...")
    docs_dir = Path(__file__).parent.parent.parent / "frontend" / "docs"

    chapter_files = []
    for module_dir in docs_dir.glob("module-*"):
        if module_dir.is_dir():
            for md_file in module_dir.glob("*.md"):
                # Skip placeholder files (very short)
                content = md_file.read_text(encoding='utf-8')
                if len(content) > 200:  # Real chapters are much longer
                    chapter_files.append(md_file)

    print(f"  - Found {len(chapter_files)} chapter files")
    for f in chapter_files:
        print(f"    * {f.relative_to(docs_dir)}")
    print()

    if len(chapter_files) == 0:
        print("  [WARNING] No chapters found! Exiting.")
        return

    # Generate embeddings
    print("[3/4] Generating embeddings...")
    chapters_data = []

    for i, filepath in enumerate(chapter_files, 1):
        print(f"  [{i}/{len(chapter_files)}] Processing {filepath.name}...")

        try:
            # Extract metadata
            metadata = extract_chapter_metadata(filepath)

            # Read content
            content = filepath.read_text(encoding='utf-8')

            # Generate embedding
            chapter_data = embedding_service.embed_chapter(content, metadata)
            chapters_data.append(chapter_data)

            print(f"      ✓ Embedded {len(content)} chars → {len(chapter_data['vector'])} dims")

        except Exception as e:
            print(f"      ✗ Error: {e}")
            continue

    print()

    # Store in vector database
    print("[4/4] Storing in Qdrant vector store...")
    try:
        vector_store.upsert_chapters_batch(chapters_data)
        print(f"  ✓ Successfully stored {len(chapters_data)} chapters")
    except Exception as e:
        print(f"  ✗ Error storing chapters: {e}")
        return

    print()

    # Verify
    collection_info = vector_store.get_collection_info()
    print("=" * 70)
    print("Summary")
    print("=" * 70)
    print(f"Collection: {collection_info['name']}")
    print(f"Total vectors: {collection_info['vectors_count']}")
    print(f"Status: {collection_info['status']}")
    print()
    print("✓ Embeddings generation complete!")
    print()
    print("Next steps:")
    print("  1. Start the FastAPI backend: uvicorn src.main:app --reload")
    print("  2. Test the RAG endpoint: curl http://localhost:8000/api/chat/query")
    print("  3. Build the frontend chatbot widget")
    print()


if __name__ == "__main__":
    main()
