# FastAPI Implementation Best Practices

**Context**: RAG Chatbot Backend API
**Feature**: Frontend-Backend Integration (003-frontend-backend-integration)
**Date**: 2025-12-29

## Overview

This document provides researched best practices for implementing a FastAPI backend that:
- Exposes chat endpoints for user queries
- Integrates with existing Python agent (agent.py)
- Supports both sync and async operations
- Handles session management with Neon Postgres
- Supports streaming responses
- Implements CORS for Vercel frontend

---

## 1. Async vs Sync Routes

### Decision: Use `async def` for all route handlers

**Rationale**:
- **I/O-Bound Operations**: The RAG chatbot performs database queries (Neon Postgres), vector searches (Qdrant), LLM calls (Groq), and embedding generation (Cohere) - all I/O-bound
- **Existing Agent Architecture**: The `agent.py` module already provides `run_agent_query_async()` and `continue_agent_session_async()` - FastAPI can await these directly
- **Concurrency**: Async routes allow FastAPI to handle multiple requests concurrently without blocking, critical for chat applications with multiple users
- **Performance**: For I/O-bound workloads, async can handle 10-100x more concurrent requests than sync
- **Streaming Support**: Required for FR-002 (streaming responses), which needs async generators

**Code Example**:
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Optional
import asyncio

# Import existing agent functions
from agent import run_agent_query_async, continue_agent_session_async, AgentResponse

app = FastAPI()

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    temperature: float = 0.5
    top_k: int = 5

class ChatResponse(BaseModel):
    response_text: str
    session_id: str
    message_id: str
    response_time_ms: float

@app.post("/api/chat", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Process chat query with RAG agent (async I/O operations).

    Uses async def because:
    - Database queries (Neon Postgres session lookup)
    - Vector search (Qdrant retrieval)
    - LLM calls (Groq via agent)
    - All are I/O-bound operations that benefit from async
    """
    try:
        # Await the async agent function directly
        agent_response: AgentResponse = await run_agent_query_async(
            query_text=request.query,
            selected_text=request.selected_text,
            session_id=request.session_id,
            temperature=request.temperature,
            top_k=request.top_k
        )

        return ChatResponse(
            response_text=agent_response.response_text,
            session_id=agent_response.conversation_id,
            message_id=f"msg_{uuid.uuid4().hex[:16]}",
            response_time_ms=agent_response.response_time_ms
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Database operations also benefit from async
@app.get("/api/sessions/{session_id}")
async def get_session(session_id: str):
    """Retrieve session from database (async I/O)."""
    # Example with async database client
    async with get_db_connection() as conn:
        session = await conn.fetch_one(
            "SELECT * FROM sessions WHERE session_id = $1",
            session_id
        )
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        return session
```

**Alternatives Considered**:
- **Sync `def` routes**: Would block the event loop during I/O operations, reducing concurrency to 1 request at a time. Rejected because chat applications need to handle multiple concurrent users.
- **Mixed sync/async**: Some routes sync, others async. Rejected because it adds complexity with no benefit - all our operations are I/O-bound.
- **Threading with `def`**: FastAPI can run `def` routes in threadpool. Rejected because async is more efficient for I/O-bound work and integrates better with existing async agent code.

**Key Rule**: Use `async def` when your route performs I/O (database, API calls, file operations). Use `def` only for CPU-bound work or when integrating with sync-only libraries.

---

## 2. FastAPI Application Structure

### Decision: Modular structure with routers, dependency injection, and lifespan events

**Rationale**:
- **Separation of Concerns**: Routers separate chat, sessions, and health endpoints into logical modules
- **Dependency Injection**: Reuse database connections, agent instances, and configuration across routes without globals
- **Resource Management**: Lifespan events ensure proper initialization (database pools, agent instance) and cleanup
- **Testability**: Dependencies can be overridden in tests, enabling unit testing without real database/API calls
- **Maintainability**: Clear structure makes it easy to add new endpoints or modify existing ones

**Code Example**:
```python
# backend/api/main.py
from contextlib import asynccontextmanager
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
import logging

from .routers import chat, sessions, health
from .dependencies import get_db_pool, get_agent_instance
from .config import settings

logger = logging.getLogger(__name__)

# Lifespan event handler for startup/shutdown
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manage application lifecycle.

    Startup:
    - Initialize database connection pool
    - Warm up agent instance (loads model)
    - Set up logging

    Shutdown:
    - Close database connections
    - Clean up resources
    """
    # Startup
    logger.info("Starting FastAPI application...")

    # Initialize database pool
    app.state.db_pool = await create_db_pool(settings.DATABASE_URL)
    logger.info("Database pool created")

    # Pre-load agent instance (warm start)
    from agent import _get_agent_instance
    app.state.agent = _get_agent_instance()
    logger.info("Agent instance initialized")

    yield  # Application runs

    # Shutdown
    logger.info("Shutting down...")
    await app.state.db_pool.close()
    logger.info("Database pool closed")

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware (covered in section 3)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(sessions.router, prefix="/api", tags=["sessions"])
app.include_router(health.router, prefix="/api", tags=["health"])

# Root endpoint
@app.get("/")
async def root():
    return {"message": "RAG Chatbot API", "version": "1.0.0"}


# backend/api/routers/chat.py
from fastapi import APIRouter, Depends, HTTPException
from ..dependencies import get_db_pool, get_agent_instance
from ..models import ChatRequest, ChatResponse
from agent import run_agent_query_async, AgentResponse

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_query(
    request: ChatRequest,
    db_pool=Depends(get_db_pool),
    agent=Depends(get_agent_instance)
):
    """Process chat query with dependency injection."""
    # Use injected dependencies
    async with db_pool.acquire() as conn:
        # Save message to database
        await conn.execute(
            "INSERT INTO messages (session_id, role, content) VALUES ($1, $2, $3)",
            request.session_id, "user", request.query
        )

    # Call agent (instance already initialized in lifespan)
    agent_response = await run_agent_query_async(
        query_text=request.query,
        selected_text=request.selected_text,
        session_id=request.session_id
    )

    return ChatResponse.from_agent_response(agent_response)


# backend/api/dependencies.py
from fastapi import Request

async def get_db_pool(request: Request):
    """Dependency: Provides database connection pool."""
    return request.app.state.db_pool

async def get_agent_instance(request: Request):
    """Dependency: Provides pre-initialized agent instance."""
    return request.app.state.agent

# Can be overridden in tests:
# app.dependency_overrides[get_db_pool] = mock_db_pool
```

**Alternatives Considered**:
- **Single file with all routes**: Works for small APIs but becomes unmaintainable. Rejected for scalability.
- **Global variables for DB/agent**: Harder to test and manage lifecycle. Rejected in favor of dependency injection.
- **Startup/shutdown events (deprecated)**: FastAPI now recommends `lifespan` context manager. Using modern pattern.

**Directory Structure**:
```
backend/
├── api/
│   ├── __init__.py
│   ├── main.py              # App creation, lifespan, middleware
│   ├── config.py            # Settings (environment variables)
│   ├── dependencies.py      # Dependency injection functions
│   ├── models.py            # Pydantic request/response models
│   └── routers/
│       ├── __init__.py
│       ├── chat.py          # Chat endpoints
│       ├── sessions.py      # Session management
│       └── health.py        # Health checks
├── agent.py                 # Existing agent code
├── retrieve.py              # Existing retrieval code
└── tests/
    ├── test_chat_api.py
    └── conftest.py          # Test fixtures
```

---

## 3. CORS Configuration

### Decision: Environment-specific CORS with explicit origins for production

**Rationale**:
- **Security**: Never use `allow_origins=["*"]` in production - it allows any website to make requests
- **Vercel Deployment**: Frontend URLs are known (e.g., `https://yourapp.vercel.app`), configure explicitly
- **Local Development**: Allow `http://localhost:3000` for Docusaurus dev server
- **Preflight Requests**: Enable `allow_credentials=True` if using cookies/auth headers (required for session management)
- **Performance**: Explicit origins are more performant than wildcard matching

**Code Example**:
```python
# backend/api/config.py
from pydantic_settings import BaseSettings
from typing import List

class Settings(BaseSettings):
    """Application settings from environment variables."""

    # CORS origins (comma-separated in .env)
    CORS_ORIGINS: List[str] = [
        "http://localhost:3000",      # Docusaurus dev server
        "http://127.0.0.1:3000",      # Alternative localhost
    ]

    # Database
    DATABASE_URL: str

    # API Keys
    GROQ_API_KEY: str
    COHERE_API_KEY: str
    QDRANT_API_KEY: str
    QDRANT_URL: str

    # Environment
    ENVIRONMENT: str = "development"  # development, staging, production

    class Config:
        env_file = ".env"
        case_sensitive = True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Add production origins based on environment
        if self.ENVIRONMENT == "production":
            # Add Vercel deployment URL from environment
            vercel_url = os.getenv("VERCEL_URL")
            if vercel_url:
                self.CORS_ORIGINS.append(f"https://{vercel_url}")

            # Add custom domain if configured
            custom_domain = os.getenv("CUSTOM_DOMAIN")
            if custom_domain:
                self.CORS_ORIGINS.append(f"https://{custom_domain}")

settings = Settings()


# backend/api/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import settings

app = FastAPI()

# CORS Middleware Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,  # Explicit origins only
    allow_credentials=True,               # Allow cookies/auth headers
    allow_methods=["GET", "POST", "DELETE"],  # Only methods we use
    allow_headers=["*"],                  # Allow all headers (can be restricted)
    max_age=600,                          # Cache preflight for 10 minutes
)

# Example .env file:
# CORS_ORIGINS=http://localhost:3000,https://yourapp.vercel.app
# ENVIRONMENT=production
# VERCEL_URL=yourapp.vercel.app
# CUSTOM_DOMAIN=textbook.example.com
```

**Handling Preflight Requests**:
```python
# FastAPI automatically handles OPTIONS requests for CORS preflight
# No additional code needed - CORSMiddleware handles it

# Example of what happens under the hood:
# 1. Browser sends OPTIONS request before POST /api/chat
# 2. CORSMiddleware intercepts it
# 3. Returns 200 with Access-Control-Allow-Origin headers
# 4. Browser proceeds with actual POST request
```

**Alternatives Considered**:
- **Wildcard `allow_origins=["*"]`**: Easy but insecure. Rejected for production use (security risk).
- **Dynamic origin validation**: Check request origin at runtime. Rejected - more complex and slower than explicit list.
- **No CORS middleware**: Would require manual header setting in every route. Rejected for poor maintainability.

**Testing CORS**:
```bash
# Test preflight request
curl -X OPTIONS http://localhost:8000/api/chat \
  -H "Origin: http://localhost:3000" \
  -H "Access-Control-Request-Method: POST"

# Should return:
# Access-Control-Allow-Origin: http://localhost:3000
# Access-Control-Allow-Methods: GET, POST, DELETE
# Access-Control-Allow-Credentials: true
```

---

## 4. Error Handling

### Decision: Centralized exception handlers with custom exceptions and HTTP status codes

**Rationale**:
- **Consistency**: All errors return standardized JSON format for frontend parsing
- **User-Friendly**: Sanitize internal errors (from agent.py) before sending to client
- **Debugging**: Log full errors server-side while returning safe messages to client
- **HTTP Semantics**: Use appropriate status codes (400 for validation, 404 for not found, 500 for server errors)
- **Type Safety**: Custom exception classes provide type hints and structure

**Code Example**:
```python
# backend/api/exceptions.py
from fastapi import HTTPException
from typing import Optional

class ChatbotException(Exception):
    """Base exception for chatbot errors."""
    def __init__(self, message: str, status_code: int = 500, details: Optional[dict] = None):
        self.message = message
        self.status_code = status_code
        self.details = details or {}
        super().__init__(self.message)

class ValidationError(ChatbotException):
    """Validation error (400)."""
    def __init__(self, message: str, details: Optional[dict] = None):
        super().__init__(message, status_code=400, details=details)

class SessionNotFoundError(ChatbotException):
    """Session not found (404)."""
    def __init__(self, session_id: str):
        super().__init__(
            message=f"Session not found: {session_id}",
            status_code=404,
            details={"session_id": session_id}
        )

class AgentError(ChatbotException):
    """Agent processing error (500)."""
    def __init__(self, message: str, details: Optional[dict] = None):
        super().__init__(message, status_code=500, details=details)


# backend/api/main.py
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
import logging

logger = logging.getLogger(__name__)

app = FastAPI()

# Custom exception handler
@app.exception_handler(ChatbotException)
async def chatbot_exception_handler(request: Request, exc: ChatbotException):
    """
    Handle all chatbot exceptions with consistent format.

    Returns:
        {
            "error": "user-friendly message",
            "status_code": 400/404/500,
            "details": {...}  # optional
        }
    """
    # Log full error server-side
    logger.error(
        f"ChatbotException: {exc.message}",
        extra={
            "status_code": exc.status_code,
            "details": exc.details,
            "path": request.url.path
        }
    )

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.message,
            "status_code": exc.status_code,
            "details": exc.details
        }
    )

# Catch-all for unexpected errors
@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """
    Handle unexpected errors with sanitized message.

    Security: Never expose internal error details (file paths, API keys, etc.)
    """
    # Log full traceback server-side
    logger.error(
        f"Unexpected error: {str(exc)}",
        exc_info=True,
        extra={"path": request.url.path}
    )

    # Return sanitized message to client
    return JSONResponse(
        status_code=500,
        content={
            "error": "An internal server error occurred. Please try again later.",
            "status_code": 500,
            "details": {}  # Never include exc details in production
        }
    )


# backend/api/routers/chat.py
from fastapi import APIRouter, HTTPException
from ..exceptions import ValidationError, AgentError
from ..models import ChatRequest, ChatResponse
from agent import run_agent_query_async, AgentResponse

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Process chat query with structured error handling.
    """
    # Validation
    if not request.query.strip():
        raise ValidationError(
            message="Query text cannot be empty",
            details={"field": "query"}
        )

    if len(request.query) > 1000:
        raise ValidationError(
            message="Query text too long (max 1000 characters)",
            details={"field": "query", "max_length": 1000, "actual_length": len(request.query)}
        )

    try:
        # Call agent
        agent_response = await run_agent_query_async(
            query_text=request.query,
            selected_text=request.selected_text,
            session_id=request.session_id
        )

        # Check for agent errors
        if agent_response.error_message:
            raise AgentError(
                message="Agent processing failed",
                details={"agent_error": agent_response.error_message}
            )

        return ChatResponse.from_agent_response(agent_response)

    except AgentError:
        raise  # Re-raise custom exceptions
    except Exception as e:
        # Wrap unexpected errors
        raise AgentError(
            message="Failed to process query",
            details={"original_error": str(e)}
        )


# backend/api/routers/sessions.py
@router.get("/sessions/{session_id}")
async def get_session(session_id: str):
    """Get session with 404 handling."""
    session = await fetch_session_from_db(session_id)

    if not session:
        raise SessionNotFoundError(session_id)

    return session
```

**Frontend Error Handling**:
```typescript
// Frontend: Parse error responses
async function sendQuery(query: string): Promise<ChatResponse> {
  try {
    const response = await fetch("http://localhost:8000/api/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ query })
    });

    if (!response.ok) {
      // Parse error response
      const error = await response.json();

      if (error.status_code === 400) {
        // Validation error - show to user
        showError(`Invalid input: ${error.error}`);
      } else if (error.status_code === 404) {
        // Session not found - create new session
        createNewSession();
      } else {
        // Server error - show generic message
        showError("An error occurred. Please try again.");
      }

      throw new Error(error.error);
    }

    return await response.json();
  } catch (error) {
    console.error("Chat error:", error);
    throw error;
  }
}
```

**Alternatives Considered**:
- **HTTPException only**: FastAPI's built-in exception. Rejected because it doesn't provide structured error details or type safety.
- **Raw status codes**: Return status codes without custom exceptions. Rejected for poor maintainability and no type checking.
- **Try-except in every route**: Duplicate error handling. Rejected in favor of centralized handlers.

---

## 5. Request Validation with Pydantic

### Decision: Pydantic V2 models for all request/response validation with field validators

**Rationale**:
- **Type Safety**: Pydantic enforces types at runtime, preventing invalid data from reaching business logic
- **Automatic Documentation**: FastAPI generates OpenAPI docs from Pydantic models
- **Field Validation**: Custom validators for complex rules (e.g., selected_text length limits)
- **Default Values**: Clean handling of optional fields (selected_text, session_id)
- **Serialization**: Automatic conversion between Python objects and JSON
- **Error Messages**: Pydantic provides detailed validation errors for debugging

**Code Example**:
```python
# backend/api/models.py
from pydantic import BaseModel, Field, field_validator, ConfigDict
from typing import Optional, List
from datetime import datetime
import uuid

class ChatRequest(BaseModel):
    """
    Request model for chat endpoint (FR-001, FR-005).

    Validates:
    - query: Required, 1-1000 characters
    - selected_text: Optional, max 2000 characters
    - session_id: Optional UUID string
    - temperature: 0.0-1.0
    - top_k: 1-10
    """
    query: str = Field(
        ...,  # Required
        min_length=1,
        max_length=1000,
        description="User's query text",
        examples=["What is ROS 2?"]
    )

    selected_text: Optional[str] = Field(
        None,
        max_length=2000,
        description="Selected text from textbook for context",
        examples=["ROS 2 is a framework for robot software development..."]
    )

    session_id: Optional[str] = Field(
        None,
        description="Session ID for multi-turn conversations",
        examples=["session_abc123def456"]
    )

    temperature: float = Field(
        0.5,
        ge=0.0,  # Greater than or equal to 0.0
        le=1.0,  # Less than or equal to 1.0
        description="Response creativity (0.0 = focused, 1.0 = creative)"
    )

    top_k: int = Field(
        5,
        ge=1,
        le=10,
        description="Number of textbook chunks to retrieve"
    )

    stream: bool = Field(
        False,
        description="Enable streaming response (FR-002)"
    )

    # Custom validator for selected_text
    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v: Optional[str]) -> Optional[str]:
        """Trim whitespace and validate non-empty if provided."""
        if v is not None:
            v = v.strip()
            if not v:  # Empty after stripping
                return None
        return v

    # Custom validator for session_id format
    @field_validator("session_id")
    @classmethod
    def validate_session_id(cls, v: Optional[str]) -> Optional[str]:
        """Validate session ID format (session_<hex>)."""
        if v is not None and not v.startswith("session_"):
            raise ValueError("Session ID must start with 'session_'")
        return v

    model_config = ConfigDict(
        # Generate example values in OpenAPI docs
        json_schema_extra={
            "examples": [
                {
                    "query": "How do I install ROS 2 on Ubuntu?",
                    "selected_text": None,
                    "session_id": None,
                    "temperature": 0.5,
                    "top_k": 5,
                    "stream": False
                },
                {
                    "query": "Explain this concept in simpler terms",
                    "selected_text": "Gazebo is a physics simulation engine...",
                    "session_id": "session_abc123",
                    "temperature": 0.7,
                    "top_k": 3,
                    "stream": True
                }
            ]
        }
    )


class RetrievedChunkMetadata(BaseModel):
    """Metadata for a retrieved chunk."""
    chunk_id: str
    source_file: str
    module_name: str
    section_heading: str
    similarity_score: float = Field(ge=0.0, le=1.0)


class ChatResponse(BaseModel):
    """
    Response model for chat endpoint (FR-001).

    Returns:
    - response_text: Agent's answer
    - session_id: Session ID for continuation
    - message_id: Unique message identifier
    - response_time_ms: Processing time
    - chunks: Retrieved chunk metadata (optional)
    """
    response_text: str = Field(
        ...,
        description="Agent's response to the query"
    )

    session_id: str = Field(
        ...,
        description="Session ID (new or existing)"
    )

    message_id: str = Field(
        default_factory=lambda: f"msg_{uuid.uuid4().hex[:16]}",
        description="Unique message identifier"
    )

    response_time_ms: float = Field(
        ...,
        ge=0.0,
        description="Processing time in milliseconds"
    )

    chunks: Optional[List[RetrievedChunkMetadata]] = Field(
        None,
        description="Retrieved chunks (if available)"
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Response timestamp"
    )

    @classmethod
    def from_agent_response(cls, agent_response, session_id: str):
        """Factory method to create from AgentResponse."""
        return cls(
            response_text=agent_response.response_text,
            session_id=session_id,
            response_time_ms=agent_response.response_time_ms,
            chunks=[
                RetrievedChunkMetadata(
                    chunk_id=chunk.chunk_id,
                    source_file=chunk.source_file,
                    module_name=chunk.module_name,
                    section_heading=chunk.section_heading,
                    similarity_score=chunk.similarity_score
                )
                for chunk in agent_response.retrieved_chunks
            ] if agent_response.retrieved_chunks else None
        )


class SessionCreateRequest(BaseModel):
    """Request to create a new session."""
    metadata: Optional[dict] = Field(
        None,
        description="Optional session metadata"
    )


class SessionResponse(BaseModel):
    """Response for session endpoints."""
    session_id: str
    created_at: datetime
    last_accessed: datetime
    message_count: int = 0
    metadata: Optional[dict] = None


# Usage in route:
@router.post("/chat", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    FastAPI automatically validates request against ChatRequest model.

    If validation fails:
    - Returns 422 Unprocessable Entity
    - Includes detailed error messages:
      {
        "detail": [
          {
            "type": "string_too_long",
            "loc": ["body", "query"],
            "msg": "String should have at most 1000 characters",
            "input": "...",
            "ctx": {"max_length": 1000}
          }
        ]
      }
    """
    # Request is already validated here
    # request.query is guaranteed to be 1-1000 chars
    # request.temperature is guaranteed to be 0.0-1.0
    # etc.

    agent_response = await run_agent_query_async(
        query_text=request.query,
        selected_text=request.selected_text
    )

    return ChatResponse.from_agent_response(agent_response, "session_123")
```

**Validation Error Response Example**:
```json
// POST /api/chat with invalid data:
// {"query": "", "temperature": 1.5}

// Response: 422 Unprocessable Entity
{
  "detail": [
    {
      "type": "string_too_short",
      "loc": ["body", "query"],
      "msg": "String should have at least 1 character",
      "input": "",
      "ctx": {"min_length": 1}
    },
    {
      "type": "less_than_equal",
      "loc": ["body", "temperature"],
      "msg": "Input should be less than or equal to 1.0",
      "input": 1.5,
      "ctx": {"le": 1.0}
    }
  ]
}
```

**Alternatives Considered**:
- **Manual validation**: Check fields manually in route handlers. Rejected - duplicate code and error-prone.
- **Dataclasses**: Python's built-in dataclasses. Rejected - no runtime validation or automatic API docs.
- **TypedDict**: Type hints only, no validation. Rejected - validation is critical for user input.

---

## 6. Streaming Responses (FR-002)

### Decision: Server-Sent Events (SSE) with async generators

**Rationale**:
- **Real-Time UX**: Stream LLM responses word-by-word for better perceived performance
- **Existing Support**: The agent.py already has `run_agent_query_streaming()` async generator
- **SSE vs WebSocket**: SSE is simpler for unidirectional server→client streaming (no need for bidirectional)
- **Browser Support**: SSE is natively supported by EventSource API in all modern browsers
- **Error Handling**: SSE automatically reconnects on connection drops

**Code Example**:
```python
# backend/api/routers/chat.py
from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from typing import AsyncIterator
import json

from agent import run_agent_query_streaming
from ..models import ChatRequest

router = APIRouter()

@router.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """
    Stream chat response using Server-Sent Events (SSE).

    Returns events:
    - retrieval_start: Retrieval begins
    - retrieval_complete: Chunks retrieved
    - response_chunk: Each word/token as it's generated
    - response_complete: Final metadata
    - error: If something fails
    """

    async def event_stream() -> AsyncIterator[str]:
        """
        Async generator that yields SSE-formatted events.

        SSE Format:
        data: {"event": "response_chunk", "text": "word"}\n\n
        """
        try:
            # Call agent's streaming function
            async for event in run_agent_query_streaming(
                query_text=request.query,
                selected_text=request.selected_text,
                session_id=request.session_id,
                temperature=request.temperature,
                top_k=request.top_k
            ):
                # Format as SSE event
                # Each event must end with \n\n
                yield f"data: {json.dumps(event)}\n\n"

        except Exception as e:
            # Send error event
            error_event = {
                "event": "error",
                "message": str(e)
            }
            yield f"data: {json.dumps(error_event)}\n\n"

    # Return StreamingResponse with text/event-stream content type
    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",  # Disable nginx buffering
        }
    )


# Frontend: Consume SSE stream
// TypeScript/JavaScript
async function streamChat(query: string) {
  const eventSource = new EventSource(
    `http://localhost:8000/api/chat/stream?query=${encodeURIComponent(query)}`
  );

  eventSource.onmessage = (event) => {
    const data = JSON.parse(event.data);

    if (data.event === "response_chunk") {
      // Append text to chat UI
      appendToMessage(data.text);
    } else if (data.event === "response_complete") {
      // Show final metadata
      console.log("Response time:", data.metadata.response_time_ms);
      eventSource.close();
    } else if (data.event === "error") {
      // Show error
      showError(data.message);
      eventSource.close();
    }
  };

  eventSource.onerror = (error) => {
    console.error("SSE error:", error);
    eventSource.close();
  };
}

// Alternative: Fetch API with streaming (more control)
async function streamChatFetch(query: string) {
  const response = await fetch("http://localhost:8000/api/chat/stream", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ query })
  });

  const reader = response.body!.getReader();
  const decoder = new TextDecoder();

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    const chunk = decoder.decode(value);
    const lines = chunk.split("\n\n");

    for (const line of lines) {
      if (line.startsWith("data: ")) {
        const data = JSON.parse(line.substring(6));

        if (data.event === "response_chunk") {
          appendToMessage(data.text);
        }
      }
    }
  }
}
```

**Alternatives Considered**:
- **WebSocket**: Full bidirectional communication. Rejected - overkill for unidirectional streaming, more complex setup.
- **Long Polling**: Repeatedly poll for updates. Rejected - inefficient, high latency.
- **JSON Streaming**: Return JSON array gradually. Rejected - SSE is standard for this use case.

---

## 7. Database Integration (Neon Postgres)

### Decision: asyncpg with connection pooling and SQLAlchemy for migrations

**Rationale**:
- **Async Native**: asyncpg is the fastest async Postgres driver for Python
- **Connection Pooling**: Reuse database connections across requests (critical for performance)
- **Neon Compatibility**: Neon Serverless Postgres works seamlessly with asyncpg
- **SQLAlchemy**: Use for schema migrations and ORM (optional), asyncpg for raw queries
- **Type Safety**: asyncpg provides good type hints for query results

**Code Example**:
```python
# backend/api/database.py
import asyncpg
from typing import AsyncIterator
import os

class Database:
    """Database connection pool manager."""

    def __init__(self, database_url: str):
        self.database_url = database_url
        self.pool: asyncpg.Pool = None

    async def connect(self):
        """Create connection pool (called in lifespan)."""
        self.pool = await asyncpg.create_pool(
            self.database_url,
            min_size=2,           # Minimum connections
            max_size=10,          # Maximum connections
            command_timeout=60,   # Query timeout
            max_queries=50000,    # Recycle connection after N queries
            max_inactive_connection_lifetime=300  # 5 minutes
        )

    async def disconnect(self):
        """Close connection pool (called in lifespan)."""
        if self.pool:
            await self.pool.close()

    async def fetch_one(self, query: str, *args):
        """Fetch single row."""
        async with self.pool.acquire() as conn:
            return await conn.fetchrow(query, *args)

    async def fetch_all(self, query: str, *args):
        """Fetch all rows."""
        async with self.pool.acquire() as conn:
            return await conn.fetch(query, *args)

    async def execute(self, query: str, *args):
        """Execute query without return."""
        async with self.pool.acquire() as conn:
            return await conn.execute(query, *args)


# backend/api/main.py
from contextlib import asynccontextmanager
from fastapi import FastAPI
from .database import Database
from .config import settings

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    app.state.db = Database(settings.DATABASE_URL)
    await app.state.db.connect()

    yield

    # Shutdown
    await app.state.db.disconnect()

app = FastAPI(lifespan=lifespan)


# backend/api/dependencies.py
from fastapi import Request
from .database import Database

async def get_db(request: Request) -> Database:
    """Dependency: Provides database instance."""
    return request.app.state.db


# backend/api/routers/sessions.py
from fastapi import APIRouter, Depends
from ..dependencies import get_db
from ..database import Database
from ..models import SessionCreateRequest, SessionResponse
from datetime import datetime
import uuid

router = APIRouter()

@router.post("/sessions", response_model=SessionResponse)
async def create_session(
    request: SessionCreateRequest,
    db: Database = Depends(get_db)
):
    """Create new chat session in Neon Postgres."""
    session_id = f"session_{uuid.uuid4().hex[:16]}"
    now = datetime.utcnow()

    # Insert session
    await db.execute(
        """
        INSERT INTO chat_sessions (session_id, created_at, last_accessed, metadata)
        VALUES ($1, $2, $3, $4)
        """,
        session_id, now, now, request.metadata or {}
    )

    return SessionResponse(
        session_id=session_id,
        created_at=now,
        last_accessed=now,
        message_count=0,
        metadata=request.metadata
    )

@router.get("/sessions/{session_id}", response_model=SessionResponse)
async def get_session(session_id: str, db: Database = Depends(get_db)):
    """Retrieve session from database."""
    row = await db.fetch_one(
        """
        SELECT session_id, created_at, last_accessed, metadata,
               (SELECT COUNT(*) FROM chat_messages WHERE session_id = $1) as message_count
        FROM chat_sessions
        WHERE session_id = $1
        """,
        session_id
    )

    if not row:
        raise HTTPException(status_code=404, detail="Session not found")

    return SessionResponse(
        session_id=row["session_id"],
        created_at=row["created_at"],
        last_accessed=row["last_accessed"],
        message_count=row["message_count"],
        metadata=row["metadata"]
    )

@router.post("/sessions/{session_id}/messages")
async def add_message(
    session_id: str,
    role: str,
    content: str,
    db: Database = Depends(get_db)
):
    """Add message to session."""
    message_id = f"msg_{uuid.uuid4().hex[:16]}"

    await db.execute(
        """
        INSERT INTO chat_messages (message_id, session_id, role, content, timestamp)
        VALUES ($1, $2, $3, $4, $5)
        """,
        message_id, session_id, role, content, datetime.utcnow()
    )

    # Update session last_accessed
    await db.execute(
        """
        UPDATE chat_sessions
        SET last_accessed = $1
        WHERE session_id = $2
        """,
        datetime.utcnow(), session_id
    )

    return {"message_id": message_id}


# Schema migrations (use Alembic or raw SQL)
# backend/migrations/001_create_tables.sql
CREATE TABLE IF NOT EXISTS chat_sessions (
    session_id VARCHAR(32) PRIMARY KEY,
    created_at TIMESTAMP NOT NULL,
    last_accessed TIMESTAMP NOT NULL,
    metadata JSONB DEFAULT '{}'
);

CREATE TABLE IF NOT EXISTS chat_messages (
    message_id VARCHAR(32) PRIMARY KEY,
    session_id VARCHAR(32) REFERENCES chat_sessions(session_id) ON DELETE CASCADE,
    role VARCHAR(10) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    selected_text TEXT,
    timestamp TIMESTAMP NOT NULL,
    response_time_ms FLOAT
);

CREATE INDEX idx_messages_session ON chat_messages(session_id, timestamp DESC);
CREATE INDEX idx_sessions_accessed ON chat_sessions(last_accessed DESC);
```

**Alternatives Considered**:
- **SQLAlchemy ORM**: Full ORM with models. Rejected - adds overhead for simple queries, asyncpg is faster.
- **psycopg3**: Newer Postgres driver. Rejected - asyncpg has better async performance and maturity.
- **Django ORM**: Rejected - not FastAPI compatible, too heavy for this use case.

---

## Summary: Key Decisions

| Topic | Decision | Key Benefit |
|-------|----------|-------------|
| **Route Type** | `async def` for all routes | Handle concurrent I/O-bound operations (DB, LLM, vector search) |
| **Structure** | Routers + dependency injection + lifespan | Modular, testable, proper resource management |
| **CORS** | Explicit origins list with environment config | Security (no wildcard) + flexibility (dev/prod) |
| **Error Handling** | Custom exceptions + centralized handlers | Consistent error format, user-friendly messages |
| **Validation** | Pydantic V2 models with validators | Type safety, auto docs, detailed error messages |
| **Streaming** | SSE with async generators | Real-time UX, simple implementation |
| **Database** | asyncpg with connection pooling | High performance, Neon compatibility |

---

## Integration with Existing Code

The existing `agent.py` module is well-structured for FastAPI integration:

1. **Async Functions Ready**: `run_agent_query_async()`, `continue_agent_session_async()`, `run_agent_query_streaming()` can be awaited directly in FastAPI routes
2. **Data Classes**: `AgentRequest`, `AgentResponse` match our Pydantic models - easy conversion
3. **Session Management**: SQLiteSession can be replaced with Neon Postgres using the same interface
4. **Streaming Support**: `run_agent_query_streaming()` yields dict events that map perfectly to SSE format
5. **Health Checks**: `check_health()` can be wrapped in a FastAPI `/health` endpoint

**Minimal FastAPI Wrapper**:
```python
# backend/api/main.py - Complete minimal example
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import sys
sys.path.append("..")

from agent import run_agent_query_async, AgentResponse

app = FastAPI(title="RAG Chatbot API")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    response_text: str
    session_id: str
    response_time_ms: float

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        result: AgentResponse = await run_agent_query_async(
            query_text=request.query,
            selected_text=request.selected_text,
            session_id=request.session_id
        )

        return ChatResponse(
            response_text=result.response_text,
            session_id=result.conversation_id,
            response_time_ms=result.response_time_ms
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Run: uvicorn api.main:app --reload
```

This minimal wrapper provides immediate functionality while allowing expansion for production features (database persistence, advanced error handling, monitoring).

---

## Deployment Considerations

**Running Locally**:
```bash
# Install dependencies
pip install fastapi uvicorn[standard] asyncpg pydantic-settings

# Run development server
uvicorn backend.api.main:app --reload --host 0.0.0.0 --port 8000

# Access:
# - API: http://localhost:8000
# - Docs: http://localhost:8000/docs (auto-generated from Pydantic models)
```

**Production Deployment** (Vercel/Render/Railway):
```bash
# Install production server
pip install gunicorn uvicorn[standard]

# Run with Gunicorn + Uvicorn workers
gunicorn backend.api.main:app \
  --workers 4 \
  --worker-class uvicorn.workers.UvicornWorker \
  --bind 0.0.0.0:8000 \
  --timeout 120
```

**Environment Variables** (.env):
```bash
# Database
DATABASE_URL=postgresql://user:pass@neon.tech:5432/chatbot

# CORS
CORS_ORIGINS=http://localhost:3000,https://yourapp.vercel.app
ENVIRONMENT=production

# API Keys
GROQ_API_KEY=your-key
COHERE_API_KEY=your-key
QDRANT_API_KEY=your-key
QDRANT_URL=https://your-qdrant.io
```

---

## Testing Strategy

```python
# backend/tests/test_chat_api.py
import pytest
from httpx import AsyncClient
from fastapi import FastAPI

# Override dependencies for testing
from api.main import app
from api.dependencies import get_db, get_agent_instance

# Mock database
@pytest.fixture
async def mock_db():
    # Return mock database instance
    pass

# Mock agent
@pytest.fixture
def mock_agent():
    # Return mock agent
    pass

@pytest.mark.asyncio
async def test_chat_endpoint(mock_db, mock_agent):
    """Test chat endpoint with mocked dependencies."""
    # Override dependencies
    app.dependency_overrides[get_db] = lambda: mock_db
    app.dependency_overrides[get_agent_instance] = lambda: mock_agent

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/chat",
            json={
                "query": "What is ROS 2?",
                "temperature": 0.5
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "response_text" in data
        assert "session_id" in data
```

---

**Document Status**: Ready for implementation
**Next Steps**: Create tasks.md with specific implementation tasks based on these decisions
