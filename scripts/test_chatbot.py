"""
Test the chatbot API directly
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

import asyncio
from src.services.rag_service import RAGService

async def test_chatbot():
    """Test RAG service directly"""
    print("Testing RAG service...")

    rag = RAGService()

    # Test question
    question = "What is ROS 2?"
    print(f"\nQuestion: {question}")

    try:
        result = rag.answer_question(question, top_k=3)
        print(f"\nAnswer: {result['answer']}")
        print(f"\nSources ({len(result['sources'])}):")
        for src in result['sources']:
            print(f"  - {src['title']} (score: {src['score']:.3f})")
        print(f"\nContext used: {result['context_used']}")
        print(f"Model: {result['model_used']}")
    except Exception as e:
        print(f"\n[ERROR] {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_chatbot())
