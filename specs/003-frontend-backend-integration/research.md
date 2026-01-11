# Research: Frontend-Backend Integration for RAG Chatbot

**Feature**: 003-frontend-backend-integration
**Created**: 2025-12-29
**Purpose**: Technical research and architectural decisions for integrating FastAPI backend with Docusaurus frontend

---

## Overview

This document captures research findings, architectural decisions, and rationale for the frontend-backend integration feature. The research focuses on three primary areas:

1. **Backend Architecture**: FastAPI implementation patterns for RAG agent exposure
2. **Database Design**: Neon Serverless Postgres schema for session management
3. **Frontend Integration**: OpenAI ChatKit SDK embedding in Docusaurus

---

## 1. FastAPI Backend Architecture

### 1.1 Async vs Sync Routes

**Decision**: Use `async def` for all routes that involve I/O operations (database, LLM calls, retrieval)

**Rationale**:
- **I/O-bound operations dominate**: Agent calls to Groq API, Qdrant retrieval, Cohere embeddings, and Neon Postgres queries are all I/O-bound
- **Concurrency benefits**: Async routes allow FastAPI to handle multiple concurrent requests without blocking, critical for 50+ concurrent users (SC-006)
- **Agent SDK compatibility**: OpenAI Agents SDK uses `await Runner.run()` - natural fit for async routes
- **Performance**: Non-blocking I/O can improve response time by 30-50% under concurrent load

**Implementation Pattern**:
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import asyncio

@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    """Async route for chat queries - allows concurrent request handling"""
    try:
        # Async database query
        session = await db.get_or_create_session(request.session_id)

        # Async agent call (I/O-bound)
        response = await run_agent_query_async(
            query_text=request.query,
            selected_text=request.selected_text,
            session_id=session.id
        )

        # Async database write
        await db.save_message(session.id, request.query, response.response_text)

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

**Alternatives Considered**:
- **Sync routes (`def`)**: Simpler but blocks the event loop during I/O, limiting concurrency to thread pool size (typically 40)
- **Mixed sync/async**: Complexity increases with no clear benefit; async is suitable for all our routes

**Trade-offs**:
- Pros: Better concurrency, natural integration with existing async agent code, scalability
- Cons: Slightly more complex debugging, requires async-compatible libraries

---

### 1.2 Application Structure & Lifespan Management

**Decision**: Use FastAPI lifespan events for database connection pooling and resource management

**Rationale**:
- **Connection pooling**: Neon Postgres connections should be pooled and reused across requests
- **Startup/shutdown hooks**: Properly initialize Qdrant client, database pool, and clean up resources
- **State management**: Agent instance can be singleton, database pool is shared app state

**Implementation Pattern**:
```python
from contextlib import asynccontextmanager
from fastapi import FastAPI
import asyncpg

# Global state
db_pool = None
qdrant_client = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle"""
    global db_pool, qdrant_client

    # Startup: Initialize connections
    db_pool = await asyncpg.create_pool(
        dsn=os.getenv("NEON_DATABASE_URL"),
        min_size=2,
        max_size=10
    )

    qdrant_client = _get_cached_qdrant_client()

    yield  # Application runs

    # Shutdown: Clean up resources
    await db_pool.close()

app = FastAPI(lifespan=lifespan)
```

**Alternatives Considered**:
- **On-demand connections**: Create connection per request - inefficient for serverless, high latency
- **Startup events only**: Deprecated in FastAPI 0.109+, lifespan is the modern approach

**Trade-offs**:
- Pros: Efficient resource usage, proper cleanup, follows FastAPI best practices
- Cons: Adds complexity to app initialization

---

### 1.3 CORS Configuration

**Decision**: Configure CORS to allow specific origins with credentials support

**Rationale**:
- **Security**: Only allow known origins (localhost for dev, Vercel domain for prod)
- **Credentials**: Support cookies/sessions if needed in future
- **Preflight caching**: Set max_age to reduce OPTIONS requests

**Implementation Pattern**:
```python
from fastapi.middleware.cors import CORSMiddleware

# Environment-based origins
CORS_ORIGINS = [
    "http://localhost:3000",  # Docusaurus dev server
    "http://localhost:5173",  # Alternative dev port
    os.getenv("FRONTEND_URL", "https://your-site.vercel.app")
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE"],
    allow_headers=["*"],
    max_age=3600  # Cache preflight for 1 hour
)
```

**Alternatives Considered**:
- **Allow all origins (`["*"]`)**: Security risk, not recommended for production
- **No CORS middleware**: Frontend cannot make cross-origin requests

**Trade-offs**:
- Pros: Secure, explicit origins, supports future authentication
- Cons: Requires environment variable management for different deployments

---

### 1.4 Error Handling & HTTP Status Codes

**Decision**: Use custom exception handlers with consistent error response format

**Rationale**:
- **User experience**: Frontend needs consistent error structure for display
- **Debugging**: Include request_id and timestamp for troubleshooting
- **Security**: Don't expose internal error details to clients

**Implementation Pattern**:
```python
from fastapi import Request, status
from fastapi.responses import JSONResponse

class ChatException(Exception):
    def __init__(self, message: str, status_code: int = 500):
        self.message = message
        self.status_code = status_code

@app.exception_handler(ChatException)
async def chat_exception_handler(request: Request, exc: ChatException):
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.message,
            "request_id": request.headers.get("x-request-id"),
            "timestamp": datetime.utcnow().isoformat()
        }
    )

# Usage in routes
if not query.strip():
    raise ChatException("Query cannot be empty", status_code=400)
```

**HTTP Status Codes**:
- `200 OK`: Successful query with response
- `400 Bad Request`: Invalid input (empty query, malformed session_id)
- `404 Not Found`: Session not found
- `429 Too Many Requests`: Rate limiting (if implemented)
- `500 Internal Server Error`: Agent failure, database error
- `503 Service Unavailable`: Dependent service down (Groq API, Qdrant)

**Alternatives Considered**:
- **Generic HTTPException**: Less semantic, harder to categorize errors
- **No custom handlers**: Default FastAPI error responses lack context

---

### 1.5 Request/Response Validation with Pydantic

**Decision**: Use Pydantic v2 models with validators for all API contracts

**Rationale**:
- **Type safety**: Automatic validation of request bodies
- **Documentation**: FastAPI auto-generates OpenAPI schema from Pydantic models
- **Flexibility**: Optional fields (selected_text, session_id) with defaults

**Implementation Pattern**:
```python
from pydantic import BaseModel, Field, validator
from typing import Optional
from uuid import UUID

class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    session_id: Optional[UUID] = Field(None, description="Session ID for multi-turn conversations")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Highlighted text from textbook")
    stream: bool = Field(False, description="Enable streaming response")

    @validator('query')
    def query_not_empty(cls, v):
        if not v.strip():
            raise ValueError('Query cannot be only whitespace')
        return v.strip()

class ChatResponse(BaseModel):
    response: str = Field(..., description="Agent's response")
    session_id: UUID = Field(..., description="Session ID")
    message_id: UUID = Field(..., description="Message ID")
    retrieved_chunks: int = Field(..., description="Number of chunks retrieved")
    response_time_ms: float = Field(..., description="Response time in milliseconds")
```

**Alternatives Considered**:
- **Manual validation**: Error-prone, duplicates logic
- **Dict-based requests**: No type checking, harder to maintain

---

## 2. Neon Serverless Postgres Schema Design

### 2.1 Sessions Table Schema

**Decision**: Two-table design - `chat_sessions` and `chat_messages` with foreign key relationship

**Rationale**:
- **Normalization**: Separates session metadata from message content
- **Scalability**: Can query sessions without loading all messages
- **Extensibility**: Easy to add session-level features (user_id, preferences)
- **Performance**: Indexed queries for recent sessions, efficient pagination

**Schema**:
```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255),  -- Future: auth integration
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_accessed TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB,  -- Flexible storage for future features
    archived BOOLEAN NOT NULL DEFAULT FALSE,
    INDEX idx_last_accessed (last_accessed DESC),
    INDEX idx_user_sessions (user_id, created_at DESC) WHERE user_id IS NOT NULL
);

CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    selected_text TEXT,  -- Optional: user-selected context
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    response_time_ms REAL,  -- NULL for user messages
    error_message TEXT,  -- NULL on success
    INDEX idx_session_messages (session_id, timestamp ASC),
    INDEX idx_recent_messages (timestamp DESC)
);
```

**Alternatives Considered**:
- **Single table**: All data in `messages` with session denormalized - slower queries, data duplication
- **Separate tables per session**: Not scalable, schema management nightmare
- **NoSQL (JSONB column)**: Harder to query, less type safety

**Trade-offs**:
- Pros: Normalized, scalable, query-efficient, supports pagination
- Cons: JOIN required for session history (acceptable performance cost)

---

### 2.2 Neon-Specific Optimizations

**Decision**: Use connection pooling with PgBouncer and prepared statements

**Rationale**:
- **Serverless cold starts**: Neon has ~100ms connection overhead; pooling amortizes this
- **Concurrent connections**: Serverless functions can spawn many connections; PgBouncer multiplexes
- **Cost efficiency**: Neon pricing based on compute time; faster queries = lower cost

**Connection Pattern**:
```python
import asyncpg
import os

# Connection pool configured in lifespan
db_pool = await asyncpg.create_pool(
    dsn=os.getenv("NEON_DATABASE_URL"),
    min_size=2,      # Minimum connections (warm pool)
    max_size=10,     # Maximum connections
    command_timeout=60,  # Query timeout
    server_settings={
        'application_name': 'rag-chatbot-api',
        'jit': 'off'  # Disable JIT for short-lived queries
    }
)

# Usage in routes
async def get_session(session_id: UUID):
    async with db_pool.acquire() as conn:
        return await conn.fetchrow(
            "SELECT * FROM chat_sessions WHERE id = $1",
            session_id
        )
```

**Neon-Specific Settings**:
- Use `pooler.` subdomain for connection pooling (e.g., `pooler.neon.tech`)
- Set `sslmode=require` for security
- Use parameterized queries to prevent SQL injection

**Alternatives Considered**:
- **Direct connections**: High latency, connection limits
- **SQLAlchemy ORM**: Heavier, more complex for simple queries

---

### 2.3 Data Retention & Archival

**Decision**: Soft-delete with archival flag, automated cleanup via cron/scheduled task

**Rationale**:
- **24-hour retention (SC-002)**: Mark sessions as `archived` after 24 hours
- **Audit trail**: Keep archived sessions for debugging, analytics
- **Storage optimization**: Periodically delete archived sessions >30 days old

**Implementation**:
```sql
-- Mark sessions older than 24 hours as archived
UPDATE chat_sessions
SET archived = TRUE
WHERE last_accessed < NOW() - INTERVAL '24 hours'
  AND archived = FALSE;

-- Optional: Delete archived sessions older than 30 days
DELETE FROM chat_sessions
WHERE archived = TRUE
  AND last_accessed < NOW() - INTERVAL '30 days';
```

**Scheduled Task**:
- Use Neon's serverless functions or external cron (GitHub Actions, Vercel Cron)
- Run daily at low-traffic hours

**Alternatives Considered**:
- **Hard delete after 24 hours**: No audit trail, can't recover accidentally cleared sessions
- **TTL extension on activity**: More complex, harder to reason about

---

### 2.4 Query Patterns & Indexing

**Decision**: Compound indexes for common query patterns

**Most Frequent Queries**:
1. **Get active session**: `WHERE id = $1 AND archived = FALSE`
2. **Get session history**: `JOIN messages WHERE session_id = $1 ORDER BY timestamp ASC`
3. **List recent sessions**: `WHERE user_id = $1 ORDER BY last_accessed DESC LIMIT 10`
4. **Cleanup archived sessions**: `WHERE archived = TRUE AND last_accessed < $1`

**Index Strategy**:
```sql
-- Primary key on id (automatic)
-- Composite index for user sessions
CREATE INDEX idx_user_sessions ON chat_sessions(user_id, last_accessed DESC) WHERE user_id IS NOT NULL;

-- Index for archival queries
CREATE INDEX idx_archived_sessions ON chat_sessions(archived, last_accessed) WHERE archived = TRUE;

-- Index for message retrieval
CREATE INDEX idx_session_messages ON chat_messages(session_id, timestamp ASC);
```

**Performance**:
- Session lookup: <10ms (indexed by UUID)
- Message retrieval: <50ms for 100-message conversations
- Pagination: <20ms with LIMIT/OFFSET

**Alternatives Considered**:
- **Full-text search on messages**: Over-engineering for current needs
- **Materialized views**: Adds complexity, not needed for current scale

---

## 3. Frontend Integration with OpenAI ChatKit SDK

### 3.1 ChatKit Installation & Setup

**Decision**: Install ChatKit SDK via npm and initialize as custom React component

**Rationale**:
- **Official SDK**: Maintained by OpenAI, handles chat UI, message rendering, typing indicators
- **Customization**: Can override default styles and behaviors
- **React compatibility**: Docusaurus is React-based, seamless integration

**Installation**:
```bash
cd frontend
npm install @openai/chatkit-react
```

**Component Structure**:
```javascript
// src/components/ChatWidget/index.jsx
import React, { useState, useEffect } from 'react';
import { ChatUI } from '@openai/chatkit-react';
import '@openai/chatkit-react/styles.css';

export default function ChatWidget() {
    const [messages, setMessages] = useState([]);
    const [sessionId, setSessionId] = useState(null);

    const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

    const handleSendMessage = async (query) => {
        try {
            const response = await fetch(`${API_BASE_URL}/api/chat`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    query,
                    session_id: sessionId,
                    selected_text: getSelectedText()
                })
            });

            const data = await response.json();
            setSessionId(data.session_id);
            setMessages([...messages,
                { role: 'user', content: query },
                { role: 'assistant', content: data.response }
            ]);
        } catch (error) {
            console.error('Chat error:', error);
        }
    };

    return <ChatUI messages={messages} onSendMessage={handleSendMessage} />;
}
```

**Alternatives Considered**:
- **Build custom chat UI**: Time-consuming, reinvents the wheel
- **Other chat libraries (react-chat-ui, chatscope)**: Less feature-rich, more configuration

---

### 3.2 Docusaurus Integration Strategy

**Decision**: Use Docusaurus theme swizzling to inject chat widget globally

**Rationale**:
- **Global availability**: Widget accessible on all pages
- **Persistent state**: Single component instance across navigation
- **Docusaurus-native**: Follows official customization patterns

**Integration Steps**:
1. **Swizzle Root component**:
```bash
npm run swizzle @docusaurus/theme-classic Root -- --eject
```

2. **Inject ChatWidget in Root**:
```javascript
// src/theme/Root.jsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
    return (
        <>
            {children}
            <ChatWidget />
        </>
    );
}
```

3. **Configure environment variables**:
```javascript
// docusaurus.config.js
module.exports = {
    customFields: {
        apiBaseUrl: process.env.VERCEL_ENV === 'production'
            ? 'https://your-backend.com'
            : 'http://localhost:8000'
    }
};
```

**Alternatives Considered**:
- **Page-specific widgets**: Duplicates state, inconsistent UX
- **MDX component**: Requires manual insertion on each page

**Trade-offs**:
- Pros: Always available, maintains state, follows Docusaurus patterns
- Cons: Loads on all pages (minimal impact with code splitting)

---

### 3.3 Selected-Text Capture Implementation

**Decision**: Capture selection on widget open, store in component state

**Rationale**:
- **User intent**: User highlights text, then opens chat - capture at that moment
- **Context preservation**: Store selected text in state until query sent
- **Edge case handling**: Handle empty selections, code blocks, special characters

**Implementation**:
```javascript
function getSelectedText() {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    // Limit to 2000 characters
    return text ? text.substring(0, 2000) : null;
}

function ChatWidget() {
    const [selectedText, setSelectedText] = useState(null);
    const [isOpen, setIsOpen] = useState(false);

    const handleWidgetOpen = () => {
        setSelectedText(getSelectedText());
        setIsOpen(true);
    };

    const handleSendMessage = async (query) => {
        // Include selectedText in API request
        const response = await fetch('/api/chat', {
            method: 'POST',
            body: JSON.stringify({
                query,
                selected_text: selectedText,
                session_id: sessionId
            })
        });

        // Clear selection after sending
        setSelectedText(null);
    };

    return (
        <FloatingChatButton onClick={handleWidgetOpen}>
            <ChatUI
                isOpen={isOpen}
                selectedContext={selectedText}
                onSendMessage={handleSendMessage}
            />
        </FloatingChatButton>
    );
}
```

**Edge Cases**:
- **Code blocks**: Preserve formatting with backticks
- **HTML elements**: Strip tags, keep text content only
- **Multiple paragraphs**: Preserve line breaks
- **Non-English text**: UTF-8 encoding handled automatically

**Alternatives Considered**:
- **Selection API listener**: Continuous tracking - battery drain, privacy concerns
- **Browser extension**: Out of scope for web app

---

### 3.4 Widget Customization vs Defaults

**Decision**: Use ChatKit defaults with minimal custom styling for branding

**Rationale**:
- **Rapid development**: Default UI is well-designed, responsive
- **Maintenance**: Less custom code to maintain
- **Customization scope**: Override colors, logo, button position only

**Custom Styles**:
```css
/* src/components/ChatWidget/styles.module.css */
.chatWidget {
    --chatkit-primary-color: #007bff;  /* Match Docusaurus theme */
    --chatkit-font-family: inherit;
    --chatkit-widget-position: bottom-right;
    --chatkit-border-radius: 12px;
}

.chatWidget :global(.chatkit-message) {
    font-size: 14px;
}
```

**Customization Scope**:
- **Colors**: Match Docusaurus theme
- **Position**: Bottom-right floating button
- **Logo**: Add textbook icon to chat header
- **Typography**: Inherit from Docusaurus

**Alternatives Considered**:
- **Full custom UI**: Time-consuming, duplicates ChatKit features
- **No customization**: Looks disconnected from site branding

**Trade-offs**:
- Pros: Fast implementation, professional UI, low maintenance
- Cons: Limited flexibility for deep customization

---

### 3.5 Environment-Based API Configuration

**Decision**: Use environment variables with fallbacks for local dev and production

**Rationale**:
- **Local development**: Default to `localhost:8000` when env var not set
- **Vercel deployment**: Inject production backend URL via Vercel env vars
- **Security**: Don't hardcode URLs in source code

**Configuration**:
```javascript
// src/config/api.js
export const API_CONFIG = {
    baseUrl: process.env.REACT_APP_API_URL
        || process.env.VERCEL_ENV === 'production'
            ? process.env.PRODUCTION_API_URL
            : 'http://localhost:8000',
    timeout: 30000,  // 30 seconds
    retries: 2
};
```

**Vercel Environment Variables**:
```bash
# Vercel Dashboard → Settings → Environment Variables
PRODUCTION_API_URL=https://your-backend.hf.space
REACT_APP_API_URL=https://your-backend.hf.space
```

**Local Development**:
```bash
# .env.local (not committed to git)
REACT_APP_API_URL=http://localhost:8000
```

**Alternatives Considered**:
- **Hardcoded URLs**: Not flexible for deployments
- **Runtime configuration file**: Requires build-time injection

---

## 4. Deployment Architecture

### 4.1 Backend Deployment (Hugging Face Spaces)

**Decision**: Deploy FastAPI backend to Hugging Face Spaces with Docker

**Rationale**:
- **Free tier**: 2 vCPU, 16GB RAM, suitable for chatbot workload
- **GPU support**: Optional, not needed for API layer (agent uses Groq)
- **Easy deployment**: Git push triggers automatic deployment
- **Custom domains**: Supports custom domain names

**Dockerfile**:
```dockerfile
FROM python:3.13-slim

WORKDIR /app

# Install dependencies
COPY backend/pyproject.toml backend/uv.lock ./
RUN pip install uv && uv sync --frozen

# Copy application code
COPY backend/ ./

# Expose port
EXPOSE 7860

# Run FastAPI with Uvicorn
CMD ["uv", "run", "uvicorn", "api:app", "--host", "0.0.0.0", "--port", "7860"]
```

**Environment Variables**:
- Set in Hugging Face Spaces settings
- `NEON_DATABASE_URL`, `GROQ_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`

**Alternatives Considered**:
- **Vercel Serverless Functions**: Limited to 10-second execution (agent can exceed this)
- **Railway**: More expensive, overkill for current needs
- **AWS Lambda**: Complex setup, cold start issues

---

### 4.2 Frontend Deployment (Vercel)

**Decision**: Deploy Docusaurus to Vercel (already configured from previous specs)

**Configuration**:
```json
// vercel.json
{
    "buildCommand": "npm run build",
    "outputDirectory": "build",
    "framework": "docusaurus",
    "env": {
        "PRODUCTION_API_URL": "https://your-space.hf.space"
    }
}
```

**Deployment Flow**:
1. Push to GitHub
2. Vercel auto-builds and deploys
3. Environment variables injected at build time
4. ChatWidget connects to Hugging Face backend

---

## 5. Testing Strategy

### 5.1 Backend Unit Tests

**Test Categories**:
1. **API endpoint tests**: Request validation, error handling
2. **Database tests**: CRUD operations, session management
3. **Integration tests**: Agent calls with mocked responses

**Test Framework**: pytest with pytest-asyncio

**Example Test**:
```python
# backend/tests/test_api.py
import pytest
from httpx import AsyncClient

@pytest.mark.asyncio
async def test_chat_endpoint_success(client: AsyncClient):
    response = await client.post("/api/chat", json={
        "query": "What is ROS 2?",
        "selected_text": None
    })
    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "session_id" in data

@pytest.mark.asyncio
async def test_chat_endpoint_empty_query(client: AsyncClient):
    response = await client.post("/api/chat", json={
        "query": "",
    })
    assert response.status_code == 400
```

---

### 5.2 Frontend Component Tests

**Test Framework**: Jest + React Testing Library

**Example Test**:
```javascript
// src/components/ChatWidget/__tests__/ChatWidget.test.jsx
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from '../index';

test('sends message on submit', async () => {
    render(<ChatWidget />);

    const input = screen.getByPlaceholderText('Ask a question...');
    fireEvent.change(input, { target: { value: 'What is Gazebo?' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    await waitFor(() => {
        expect(screen.getByText('What is Gazebo?')).toBeInTheDocument();
    });
});
```

---

### 5.3 End-to-End Test Queries

**Test Queries** (from SC-003):
1. "How do I install ROS 2?" - Verify installation instructions retrieved
2. "What is Gazebo simulation?" - Verify physics engine explanation
3. "Explain VLA models" - Verify multi-turn context maintenance
4. Selected text: Highlight paragraph about humanoid robots, ask "Summarize this"
5. "What are the key components of a humanoid robot?" - Verify comprehensive response

**Validation Criteria**:
- Response time <5 seconds (SC-001)
- Session persists across page navigation (SC-002)
- Selected-text mode includes context (SC-004)
- Mobile responsive (SC-009)

---

## 6. Summary of Key Decisions

| Decision Area | Choice | Rationale |
|---------------|--------|-----------|
| **Route Pattern** | Async def for all I/O routes | Better concurrency for I/O-bound operations |
| **App Lifecycle** | FastAPI lifespan events | Proper connection pooling and resource cleanup |
| **CORS** | Explicit origins list | Security while allowing Vercel + localhost |
| **Error Handling** | Custom exception handlers | Consistent error format for frontend |
| **Request Validation** | Pydantic v2 models | Type safety and auto-documentation |
| **Database Schema** | Two tables (sessions, messages) | Normalized, scalable, query-efficient |
| **Connection Pooling** | asyncpg with PgBouncer | Serverless-optimized, handles cold starts |
| **Data Retention** | 24-hour soft delete | Audit trail, scheduled cleanup |
| **ChatKit Integration** | React component in Root | Global availability, persistent state |
| **Selected-Text** | Capture on widget open | Preserves user intent, handles edge cases |
| **Widget Styling** | ChatKit defaults + theme colors | Fast dev, low maintenance, branded |
| **Backend Deployment** | Hugging Face Spaces | Free tier, sufficient resources, easy deploy |
| **Frontend Deployment** | Vercel | Already configured, auto-deploy from GitHub |

---

## Next Steps

1. **Phase 1**: Create `data-model.md` with detailed entity schemas
2. **Phase 1**: Generate OpenAPI contracts in `/contracts/`
3. **Phase 1**: Write `quickstart.md` with local dev setup instructions
4. **Phase 2**: Use this research to inform implementation plan in `plan.md`

---

**Research Status**: ✅ Complete
**Ready for Phase 1**: Yes
