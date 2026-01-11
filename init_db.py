"""Initialize database tables for the chat application."""
import asyncio
from backend.db import init_database

async def main():
    print("Initializing database tables...")
    await init_database()
    print("SUCCESS: Database tables created successfully!")

if __name__ == "__main__":
    asyncio.run(main())
