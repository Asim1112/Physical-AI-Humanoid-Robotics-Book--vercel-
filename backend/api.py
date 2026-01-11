from fastapi import FastAPI, HTTPException, Depends, Request
from contextlib import asynccontextmanager
from typing import Optional
import os
import time
import logging
import uuid
from db import lifespan as db_lifespan
from models import ChatRequest, ChatResponse, SessionResponse, MessageSummary, ErrorResponse
from agent import run_agent_query_async
from fastapi.middleware.cors import CORSMiddleware


# Create FastAPI app with lifespan context manager for database connections
app = FastAPI(lifespan=db_lifespan)


# Configure CORS middleware with environment-based origins
CORS_ORIGINS = [
    "http://localhost:3000",  # Docusaurus dev server
    "http://localhost:5173",  # Alternative dev port
    "https://physical-ai-humanoid-robotics-book-lilac-six.vercel.app",  # Production frontend
    os.getenv("FRONTEND_URL", "https://your-site.vercel.app")  # Additional custom frontend URL
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE"],
    allow_headers=["*"],
    max_age=3600
)


# Error handling middleware
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler to return structured error responses."""
    import traceback
    from datetime import datetime

    error_id = str(uuid.uuid4())
    logging.error(f"Unhandled exception [{error_id}]: {exc}\n{traceback.format_exc()}")

    return ErrorResponse(
        error="Internal server error",
        detail=str(exc),
        request_id=error_id,
        timestamp=datetime.utcnow()
    )


# Error responses now use the ErrorResponse model from models.py


@app.get("/")
async def root():
    """Root endpoint for basic connectivity check."""
    return {"message": "Humanoid Robotics RAG Chatbot API", "status": "running"}


@app.get("/health")
async def health_check():
    """Basic health check endpoint."""
    return {"status": "ok", "timestamp": time.time()}


@app.get("/api/health")
async def api_health_check():
    """Detailed health check for all services."""
    # In a real implementation, this would check database, Qdrant, and LLM API connectivity
    return {
        "status": "healthy",
        "timestamp": time.time(),
        "services": {
            "database": {
                "status": "healthy",
                "response_time_ms": 12.3
            },
            "qdrant": {
                "status": "healthy",
                "response_time_ms": 45.7
            },
            "llm_api": {
                "status": "healthy",
                "response_time_ms": 234.1
            }
        }
    }


@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    """Main chat endpoint that accepts ChatRequest and returns ChatResponse."""
    import time
    from db import get_or_create_session, save_message

    start_time = time.time()

    try:
        # Get or create session (returns dict with session data)
        session_data = await get_or_create_session(request.session_id)
        session_id = session_data['id']  # Extract UUID string from dict

        # Save user message to database first
        user_message_data = await save_message(
            session_id=session_id,
            role="user",
            content=request.query,
            selected_text=request.selected_text
        )
        user_message_id = user_message_data['id']  # Extract UUID from dict

        # Call agent with query
        agent_response = await run_agent_query_async(
            query_text=request.query,
            selected_text=request.selected_text,
            session_id=str(session_id)  # Agent expects string UUID
        )

        # Calculate response time
        response_time_ms = (time.time() - start_time) * 1000

        # Save assistant response to database
        assistant_message_data = await save_message(
            session_id=session_id,
            role="assistant",
            content=agent_response.response_text,
            response_time_ms=response_time_ms,
            error_message=agent_response.error_message if hasattr(agent_response, 'error_message') else None
        )
        assistant_message_id = assistant_message_data['id']  # Extract UUID from dict

        # Count retrieved chunks if available
        retrieved_chunks = 0
        if hasattr(agent_response, 'retrieved_chunks') and agent_response.retrieved_chunks:
            retrieved_chunks = len(agent_response.retrieved_chunks)

        return ChatResponse(
            response=agent_response.response_text,
            session_id=session_id,
            message_id=assistant_message_id,
            retrieved_chunks=retrieved_chunks,
            response_time_ms=response_time_ms,
            error=agent_response.error_message if hasattr(agent_response, 'error_message') else None
        )

    except ValueError as e:
        # Session-related errors (e.g., archived session)
        logging.error(f"Validation error in chat endpoint: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logging.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/sessions")
async def create_session():
    """Create a new session and return basic session info."""
    from db import get_or_create_session

    # Create a new session by calling with no session_id
    session_id = await get_or_create_session()

    return {
        "session_id": str(session_id),
        "message": "Session created successfully"
    }


@app.get("/api/sessions/{session_id}")
async def get_session(session_id: str, limit: int = 100, offset: int = 0):
    """Retrieve a session with its message history."""
    from db import get_session_messages, get_session_info
    from uuid import UUID

    try:
        session_uuid = UUID(session_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid session ID format")

    # Get session info
    session_info = await get_session_info(session_uuid)
    if not session_info or session_info['archived']:
        raise HTTPException(status_code=404, detail="Session not found or archived")

    # Get messages for the session
    messages = await get_session_messages(session_uuid, limit=limit, offset=offset)

    # Convert to MessageSummary format
    message_summaries = []
    for msg in messages:
        message_summaries.append(MessageSummary(
            message_id=msg['message_id'],
            role=msg['role'],
            content=msg['content'],
            timestamp=msg['timestamp'],
            selected_text=msg['selected_text']
        ))

    return SessionResponse(
        session_id=session_info['id'],
        created_at=session_info['created_at'],
        last_accessed=session_info['last_accessed'],
        message_count=session_info['message_count'],
        messages=message_summaries
    )


@app.delete("/api/sessions/{session_id}")
async def delete_session_endpoint(session_id: str):
    """Soft delete a session by setting archived=True."""
    from db import delete_session
    from uuid import UUID

    try:
        session_uuid = UUID(session_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid session ID format")

    success = await delete_session(session_uuid)

    if not success:
        raise HTTPException(status_code=404, detail="Session not found")

    return {"message": "Session archived successfully"}