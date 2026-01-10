import os
import sys
import glob
import uuid
import re
import time
from typing import List, Dict

# Add parent directory to path so we can import src modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.services.cohere_client import cohere_client
from src.services.qdrant_service import qdrant_service
from qdrant_client.http import models

# Paths - go up one level from backend directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_DIR = os.path.dirname(BACKEND_DIR)
CONTENT_DIR = os.path.join(PROJECT_DIR, "content", "docs")

def parse_markdown(file_path: str) -> Dict:
    """Extract metadata and content from markdown file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Simple metadata extraction (Module and Title)
    # Assumes path like content/docs/module-1-ros2/01-introduction.md
    parts = file_path.split(os.sep)
    module = parts[-2]
    filename = parts[-1]

    # Extract title from first # header
    title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    title = title_match.group(1) if title_match else filename

    return {
        "module": module,
        "title": title,
        "content": content,
        "chapter_id": filename.split('-')[0] if '-' in filename else filename
    }

def chunk_text(text: str, max_tokens: int = 500) -> List[str]:
    """Simple chunking by sentences/paragraphs to stay under token limit"""
    paragraphs = text.split('\n\n')
    chunks = []
    current_chunk = ""

    for para in paragraphs:
        if len(current_chunk) + len(para) < max_tokens * 4: # rough approx: 1 token ~ 4 chars
            current_chunk += para + "\n\n"
        else:
            chunks.append(current_chunk.strip())
            current_chunk = para + "\n\n"

    if current_chunk:
        chunks.append(current_chunk.strip())
    return chunks

def run_ingestion():
    """Main ingestion pipeline"""
    print(f"Starting ingestion from: {CONTENT_DIR}")

    # Find all .md files excluding templates
    md_files = glob.glob(os.path.join(CONTENT_DIR, "**", "*.md"), recursive=True)
    md_files = [f for f in md_files if "_templates" not in f]

    all_points = []

    # Process first 5 files for testing
    limit_files = md_files[:5]
    print(f"Processing {len(limit_files)} files out of {len(md_files)} total.")

    for idx, file_path in enumerate(limit_files):
        print(f"Processing ({idx+1}/{len(limit_files)}): {file_path}")
        doc_data = parse_markdown(file_path)
        chunks = chunk_text(doc_data["content"])
        print(f"File split into {len(chunks)} chunks")

        # Generate embeddings in batch for this file
        if chunks:
            print(f"Generating embeddings for {len(chunks)} chunks...")
            embeddings = cohere_client.get_embeddings(chunks, input_type="search_document")

            # Small delay to avoid rate limits
            time.sleep(2)

            for i, (chunk, vector) in enumerate(zip(chunks, embeddings)):
                point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_path}_{i}"))
                all_points.append(
                    models.PointStruct(
                        id=point_id,
                        vector=vector,
                        payload={
                            "content": chunk,
                            "title": doc_data["title"],
                            "module": doc_data["module"],
                            "chapter_id": doc_data["chapter_id"],
                            "src_file": file_path
                        }
                    )
                )

            # Upload per file to avoid timeout
            if all_points:
                print(f"Upserting {len(all_points)} points so far...")
                qdrant_service.upsert_chunks(all_points)
                all_points = []

    if all_points:
        print(f"Upserting final {len(all_points)} points to Qdrant...")
        qdrant_service.upsert_chunks(all_points)

    print("Ingestion complete!")

if __name__ == "__main__":
    run_ingestion()
