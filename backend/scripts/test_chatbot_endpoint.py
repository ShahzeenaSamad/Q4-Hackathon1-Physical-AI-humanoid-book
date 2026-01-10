"""Test chatbot endpoint directly"""
import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.api.chat import chat_query, ChatRequest
from src.core.database import get_db

async def test():
    print("Testing chatbot endpoint...")
    try:
        async for db in get_db():
            req = ChatRequest(question='What is Physical AI?', top_k=3)
            print(f"Request: {req}")
            result = await chat_query(req, db)
            print(f"\nSuccess!")
            print(f"Answer: {result.answer[:200]}")
            print(f"Sources: {len(result.sources)}")
            print(f"Session ID: {result.session_id}")
            break
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test())
