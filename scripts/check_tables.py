"""Check database tables"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

import asyncio
from src.core.database import engine
from sqlalchemy import text, inspect

async def check_tables():
    try:
        async with engine.begin() as conn:
            # Check what tables exist
            result = await conn.execute(text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
            """))
            tables = [row[0] for row in result.fetchall()]

            print(f"\n[INFO] Tables in database:")
            if tables:
                for table in tables:
                    print(f"  - {table}")
            else:
                print("  No tables found!")

            # Check if chat_sessions exists
            if 'chat_sessions' in tables:
                print(f"\n[OK] chat_sessions table exists")
                # Check structure
                result = await conn.execute(text("""
                    SELECT column_name, data_type
                    FROM information_schema.columns
                    WHERE table_name = 'chat_sessions'
                """))
                columns = result.fetchall()
                print(f"\n[INFO] chat_sessions columns:")
                for col in columns:
                    print(f"  - {col[0]}: {col[1]}")
            else:
                print(f"\n[ERROR] chat_sessions table does NOT exist!")
                print("Run: cd backend && ./venv/Scripts/python -m alembic upgrade head")

    except Exception as e:
        print(f"\n[ERROR] Failed to check tables: {e}")
        import traceback
        traceback.print_exc()

asyncio.run(check_tables())
