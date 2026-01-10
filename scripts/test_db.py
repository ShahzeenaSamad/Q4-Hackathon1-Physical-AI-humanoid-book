"""Test database connection"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

import asyncio
from src.core.database import engine
from sqlalchemy import text

async def test_db():
    try:
        async with engine.begin() as conn:
            result = await conn.execute(text('SELECT 1'))
            print(f"[OK] Database connection successful! Result: {result.fetchone()}")
    except Exception as e:
        print(f"[ERROR] Database connection failed: {e}")
        import traceback
        traceback.print_exc()

asyncio.run(test_db())
