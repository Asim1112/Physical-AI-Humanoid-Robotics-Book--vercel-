"""
Database initialization script.

Run this script to create the database tables and indexes for the RAG chatbot.

Usage:
    python -m backend.init_db
"""

import asyncio
import sys
from pathlib import Path

# Add backend directory to path
backend_dir = Path(__file__).parent
sys.path.insert(0, str(backend_dir.parent))

from backend.db import init_database


async def main():
    """Initialize the database schema."""
    print("Initializing database...")

    try:
        await init_database()
        print("[SUCCESS] Database initialized successfully!")
        print("   - chat_sessions table created")
        print("   - chat_messages table created")
        print("   - All indexes created")
    except Exception as e:
        print(f"[ERROR] Error initializing database: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print("Database initialization complete.")


if __name__ == "__main__":
    asyncio.run(main())
