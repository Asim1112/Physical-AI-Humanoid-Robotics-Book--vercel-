import os
import asyncpg
from contextlib import asynccontextmanager
from fastapi import FastAPI
from typing import Optional
import uuid
from datetime import datetime
from pydantic import BaseModel, Field
from typing import List, Optional
import logging
from pathlib import Path
from dotenv import load_dotenv
import json

# Load environment variables from .env file in backend directory
backend_dir = Path(__file__).parent
env_path = backend_dir / ".env"
load_dotenv(dotenv_path=env_path)


# Global variable to hold the database pool
db_pool = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager to handle database connection pool."""
    global db_pool

    # Startup: Create database connection pool
    db_pool = await asyncpg.create_pool(
        dsn=os.getenv("NEON_DATABASE_URL"),
        min_size=2,   # Keep 2 connections warm
        max_size=10   # Allow up to 10 concurrent connections
    )

    yield  # Application runs here

    # Shutdown: Close database connection pool
    await db_pool.close()


class ChatSession(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime
    last_accessed: datetime
    metadata: Optional[dict] = None
    archived: bool = False


class ChatMessage(BaseModel):
    id: str
    session_id: str
    role: str
    content: str
    selected_text: Optional[str] = None
    timestamp: datetime
    response_time_ms: Optional[float] = None
    error_message: Optional[str] = None


async def init_database():
    """Initialize the database by creating tables and indexes."""
    global db_pool

    if not db_pool:
        # Create a temporary connection if pool is not initialized
        temp_pool = await asyncpg.create_pool(
            dsn=os.getenv("NEON_DATABASE_URL"),
            min_size=1,
            max_size=2
        )
        conn = await temp_pool.acquire()
    else:
        conn = await db_pool.acquire()

    try:
        # Create chat_sessions table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chat_sessions (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id VARCHAR(255) CHECK (user_id ~ '^[a-zA-Z0-9\-]+$'),
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                last_accessed TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                metadata JSONB,
                archived BOOLEAN NOT NULL DEFAULT FALSE,
                CONSTRAINT last_accessed_after_created CHECK (last_accessed >= created_at)
            )
        """)

        # Create chat_messages table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chat_messages (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
                role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
                content TEXT NOT NULL CHECK (LENGTH(TRIM(content)) > 0),
                selected_text TEXT CHECK (selected_text IS NULL OR LENGTH(selected_text) <= 5000),
                timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                response_time_ms REAL CHECK (response_time_ms IS NULL OR response_time_ms >= 0),
                error_message TEXT
            )
        """)

        # Create indexes
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_last_accessed ON chat_sessions(last_accessed DESC)
        """)

        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_user_sessions ON chat_sessions(user_id, created_at DESC) WHERE user_id IS NOT NULL
        """)

        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_session_messages ON chat_messages(session_id, timestamp ASC)
        """)

        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_recent_messages ON chat_messages(timestamp DESC)
        """)

        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_archived_cleanup ON chat_sessions(archived, last_accessed) WHERE archived = TRUE
        """)

        logging.info("Database tables and indexes created successfully")

    except Exception as e:
        logging.error(f"Error initializing database: {e}")
        raise
    finally:
        if not db_pool:
            await temp_pool.release(conn)
            await temp_pool.close()
        else:
            await db_pool.release(conn)


async def get_or_create_session(session_id: Optional[str] = None) -> dict:
    """Get existing session or create a new one if session_id is None or invalid."""
    global db_pool
    conn = await db_pool.acquire()

    try:
        if session_id:
            # Try to get existing session
            query = """
                SELECT id, user_id, created_at, last_accessed, metadata, archived
                FROM chat_sessions
                WHERE id = $1 AND archived = FALSE
            """
            session = await conn.fetchrow(query, session_id)

            if session:
                # Update last_accessed timestamp
                update_query = """
                    UPDATE chat_sessions
                    SET last_accessed = NOW()
                    WHERE id = $1
                """
                await conn.execute(update_query, session_id)

                return {
                    'id': str(session['id']),
                    'user_id': session['user_id'],
                    'created_at': session['created_at'],
                    'last_accessed': session['last_accessed'],
                    'metadata': session['metadata'],
                    'archived': session['archived']
                }

        # Create new session
        insert_query = """
            INSERT INTO chat_sessions (user_id)
            VALUES ($1)
            RETURNING id, user_id, created_at, last_accessed, metadata, archived
        """
        new_session = await conn.fetchrow(insert_query, None)

        return {
            'id': str(new_session['id']),
            'user_id': new_session['user_id'],
            'created_at': new_session['created_at'],
            'last_accessed': new_session['last_accessed'],
            'metadata': new_session['metadata'],
            'archived': new_session['archived']
        }
    finally:
        await db_pool.release(conn)


async def save_message(session_id: str, role: str, content: str, selected_text: Optional[str] = None,
                      response_time_ms: Optional[float] = None, error_message: Optional[str] = None) -> dict:
    """Save a message to the database."""
    global db_pool
    conn = await db_pool.acquire()

    try:
        insert_query = """
            INSERT INTO chat_messages (session_id, role, content, selected_text, response_time_ms, error_message)
            VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING id, session_id, role, content, selected_text, timestamp, response_time_ms, error_message
        """
        result = await conn.fetchrow(insert_query, session_id, role, content, selected_text, response_time_ms, error_message)

        return {
            'id': str(result['id']),
            'session_id': str(result['session_id']),
            'role': result['role'],
            'content': result['content'],
            'selected_text': result['selected_text'],
            'timestamp': result['timestamp'],
            'response_time_ms': result['response_time_ms'],
            'error_message': result['error_message']
        }
    finally:
        await db_pool.release(conn)


async def get_session_messages(session_id: str, limit: int = 50, offset: int = 0) -> List[dict]:
    """Get messages for a specific session with pagination."""
    global db_pool
    conn = await db_pool.acquire()

    try:
        query = """
            SELECT id, session_id, role, content, selected_text, timestamp, response_time_ms, error_message
            FROM chat_messages
            WHERE session_id = $1
            ORDER BY timestamp ASC
            LIMIT $2 OFFSET $3
        """
        results = await conn.fetch(query, session_id, limit, offset)

        messages = []
        for row in results:
            messages.append({
                'id': str(row['id']),
                'session_id': str(row['session_id']),
                'role': row['role'],
                'content': row['content'],
                'selected_text': row['selected_text'],
                'timestamp': row['timestamp'],
                'response_time_ms': row['response_time_ms'],
                'error_message': row['error_message']
            })

        return messages
    finally:
        await db_pool.release(conn)


async def delete_session(session_id: str) -> bool:
    """Soft delete a session by setting archived=True."""
    global db_pool
    conn = await db_pool.acquire()

    try:
        query = """
            UPDATE chat_sessions
            SET archived = TRUE
            WHERE id = $1
        """
        result = await conn.execute(query, session_id)

        # Check if any rows were affected (session existed)
        # In asyncpg, execute returns something like '<command-tag> <row-count>' e.g. 'UPDATE 1'
        if result:
            # Extract the number after the command name (e.g., from 'UPDATE 1', get 1)
            parts = result.split()
            if len(parts) >= 2:
                try:
                    rows_affected = int(parts[1])
                    return rows_affected > 0
                except ValueError:
                    pass

        return False
    finally:
        await db_pool.release(conn)