"""Create chat_sessions table"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.core.database import engine
from sqlalchemy import text

async def create_chat_table():
    print("Creating chat_sessions table...")
    async with engine.begin() as conn:
        await conn.execute(text("""
            CREATE TABLE IF NOT EXISTS chat_sessions (
                id UUID PRIMARY KEY,
                messages JSONB,
                created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
            )
        """))
    print("chat_sessions table created successfully!")

if __name__ == "__main__":
    asyncio.run(create_chat_table())
