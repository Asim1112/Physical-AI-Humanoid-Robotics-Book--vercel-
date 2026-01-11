# Data Model: Frontend-Backend Integration for RAG Chatbot

**Feature**: 003-frontend-backend-integration
**Created**: 2025-12-29
**Purpose**: Define data entities, relationships, and validation rules

---

## Overview

This document defines the data model for the frontend-backend integration feature. The model consists of database entities (Neon Postgres), API contracts (Pydantic models), and frontend state (React component props).

---

## 1. Database Entities (Neon Postgres)

### 1.1 ChatSession

**Purpose**: Represents a conversation session between a user and the RAG agent

**Table**: `chat_sessions`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique session identifier |
| `user_id` | VARCHAR(255) | NULL | Optional user identifier (future: authentication) |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Session creation timestamp |
| `last_accessed` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last interaction timestamp (updated on each message) |
| `metadata` | JSONB | NULL | Flexible storage for session-level data (user agent, IP, preferences) |
| `archived` | BOOLEAN | NOT NULL, DEFAULT FALSE | Soft-delete flag for 24-hour expiration |

**Indexes**:
- `PRIMARY KEY (id)` - Fast lookups by session ID
- `INDEX idx_last_accessed (last_accessed DESC)` - Recent sessions query
- `INDEX idx_user_sessions (user_id, created_at DESC) WHERE user_id IS NOT NULL` - User session history (partial index)
- `INDEX idx_archived_cleanup (archived, last_accessed) WHERE archived = TRUE` - Cleanup queries

**Validation Rules**:
- `id` must be valid UUID v4
- `last_accessed` must be >= `created_at`
- `metadata` must be valid JSON object (no arrays at top level)
- `user_id` format: alphanumeric + hyphens only (regex: `^[a-zA-Z0-9\-]+$`)

**Lifecycle**:
1. **Creation**: New session created on first chat message without session_id
2. **Active**: `last_accessed` updated on each message
3. **Expiration**: Marked `archived = TRUE` after 24 hours of inactivity
4. **Deletion**: Permanently deleted after 30 days in archived state

**SQL Schema**:
```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255) CHECK (user_id ~ '^[a-zA-Z0-9\-]+$'),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_accessed TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB,
    archived BOOLEAN NOT NULL DEFAULT FALSE,
    CONSTRAINT last_accessed_after_created CHECK (last_accessed >= created_at)
);

CREATE INDEX idx_last_accessed ON chat_sessions(last_accessed DESC);
CREATE INDEX idx_user_sessions ON chat_sessions(user_id, created_at DESC) WHERE user_id IS NOT NULL;
CREATE INDEX idx_archived_cleanup ON chat_sessions(archived, last_accessed) WHERE archived = TRUE;
```

---

### 1.2 ChatMessage

**Purpose**: Represents a single message (user query or agent response) in a conversation

**Table**: `chat_messages`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique message identifier |
| `session_id` | UUID | NOT NULL, FOREIGN KEY REFERENCES chat_sessions(id) ON DELETE CASCADE | Session this message belongs to |
| `role` | VARCHAR(20) | NOT NULL, CHECK (role IN ('user', 'assistant')) | Message author (user or agent) |
| `content` | TEXT | NOT NULL, CHECK (LENGTH(content) > 0) | Message text content |
| `selected_text` | TEXT | NULL, CHECK (LENGTH(selected_text) <= 5000) | User-selected textbook context (user messages only) |
| `timestamp` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Message creation timestamp |
| `response_time_ms` | REAL | NULL, CHECK (response_time_ms >= 0) | Agent response time in milliseconds (assistant messages only) |
| `error_message` | TEXT | NULL | Error details if message generation failed |

**Indexes**:
- `PRIMARY KEY (id)` - Fast lookups by message ID
- `INDEX idx_session_messages (session_id, timestamp ASC)` - Retrieve conversation history in order
- `INDEX idx_recent_messages (timestamp DESC)` - Recent messages across all sessions

**Validation Rules**:
- `content` cannot be empty or whitespace-only
- `selected_text` max length: 5000 characters (prevents excessive context)
- `role` must be exactly 'user' or 'assistant' (lowercase)
- `response_time_ms` only set for assistant messages, must be non-negative
- `error_message` only set when response generation fails

**Message Pairs**:
- Each user message (role='user') should be followed by an assistant message (role='assistant')
- Conversation history retrieved in chronological order (oldest first)

**SQL Schema**:
```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (LENGTH(TRIM(content)) > 0),
    selected_text TEXT CHECK (LENGTH(selected_text) <= 5000),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    response_time_ms REAL CHECK (response_time_ms >= 0),
    error_message TEXT
);

CREATE INDEX idx_session_messages ON chat_messages(session_id, timestamp ASC);
CREATE INDEX idx_recent_messages ON chat_messages(timestamp DESC);
```

---

## 2. API Contract Models (Pydantic)

### 2.1 ChatRequest

**Purpose**: Input model for POST /api/chat endpoint

**Fields**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| `query` | str | Yes | 1-2000 chars, non-empty after strip | User's question |
| `session_id` | UUID \| None | No | Valid UUID v4 or null | Session ID for multi-turn conversations |
| `selected_text` | str \| None | No | Max 5000 chars | Highlighted textbook text |
| `stream` | bool | No | Default: false | Enable streaming response |

**Validation**:
- `query`: Must not be empty or whitespace-only after trimming
- `session_id`: If provided, must exist in database and not be archived
- `selected_text`: Truncated to 5000 chars if exceeds limit
- `stream`: Defaults to false for simplicity

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, validator
from typing import Optional
from uuid import UUID

class ChatRequest(BaseModel):
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question about the textbook"
    )
    session_id: Optional[UUID] = Field(
        None,
        description="Session ID for multi-turn conversations (omit for new session)"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=5000,
        description="Highlighted text from textbook for contextual queries"
    )
    stream: bool = Field(
        False,
        description="Enable streaming response (not implemented in MVP)"
    )

    @validator('query')
    def query_must_not_be_empty(cls, v):
        if not v.strip():
            raise ValueError('Query cannot be empty or whitespace-only')
        return v.strip()

    @validator('selected_text')
    def truncate_selected_text(cls, v):
        if v and len(v) > 5000:
            return v[:5000]
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do I install ROS 2?",
                "session_id": None,
                "selected_text": None,
                "stream": False
            }
        }
```

---

### 2.2 ChatResponse

**Purpose**: Output model for POST /api/chat endpoint

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `response` | str | Yes | Agent's response text |
| `session_id` | UUID | Yes | Session ID (new or existing) |
| `message_id` | UUID | Yes | Unique ID for this message |
| `retrieved_chunks` | int | Yes | Number of chunks retrieved from Qdrant |
| `response_time_ms` | float | Yes | Total response time in milliseconds |
| `error` | str \| None | No | Error message if generation failed |

**Pydantic Model**:
```python
class ChatResponse(BaseModel):
    response: str = Field(..., description="Agent's response to the query")
    session_id: UUID = Field(..., description="Session ID (newly created or provided)")
    message_id: UUID = Field(..., description="Unique message identifier")
    retrieved_chunks: int = Field(..., ge=0, description="Number of textbook chunks retrieved")
    response_time_ms: float = Field(..., ge=0, description="Response time in milliseconds")
    error: Optional[str] = Field(None, description="Error message if generation failed")

    class Config:
        json_schema_extra = {
            "example": {
                "response": "To install ROS 2 Humble...",
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "message_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
                "retrieved_chunks": 5,
                "response_time_ms": 4250.5,
                "error": None
            }
        }
```

---

### 2.3 SessionResponse

**Purpose**: Output model for GET /api/sessions/{id} endpoint

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `session_id` | UUID | Session identifier |
| `created_at` | str (ISO 8601) | Session creation timestamp |
| `last_accessed` | str (ISO 8601) | Last interaction timestamp |
| `message_count` | int | Number of messages in session |
| `messages` | List[MessageSummary] | Conversation history |

**MessageSummary Sub-Model**:
| Field | Type | Description |
|-------|------|-------------|
| `message_id` | UUID | Message identifier |
| `role` | Literal['user', 'assistant'] | Message author |
| `content` | str | Message text |
| `timestamp` | str (ISO 8601) | Message timestamp |
| `selected_text` | str \| None | Selected textbook context (if applicable) |

**Pydantic Models**:
```python
from typing import Literal, List
from datetime import datetime

class MessageSummary(BaseModel):
    message_id: UUID
    role: Literal['user', 'assistant']
    content: str
    timestamp: datetime
    selected_text: Optional[str] = None

class SessionResponse(BaseModel):
    session_id: UUID
    created_at: datetime
    last_accessed: datetime
    message_count: int
    messages: List[MessageSummary]

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "created_at": "2025-12-29T10:00:00Z",
                "last_accessed": "2025-12-29T10:15:00Z",
                "message_count": 4,
                "messages": [
                    {
                        "message_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
                        "role": "user",
                        "content": "What is ROS 2?",
                        "timestamp": "2025-12-29T10:00:05Z",
                        "selected_text": None
                    },
                    {
                        "message_id": "8d0f7780-8536-51ef-b827-f18gd2g01bf8",
                        "role": "assistant",
                        "content": "ROS 2 is...",
                        "timestamp": "2025-12-29T10:00:09Z",
                        "selected_text": None
                    }
                ]
            }
        }
```

---

### 2.4 ErrorResponse

**Purpose**: Standard error response for all endpoints

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `error` | str | Human-readable error message |
| `detail` | str \| None | Additional error details (optional) |
| `request_id` | str \| None | Request ID for debugging |
| `timestamp` | str (ISO 8601) | Error timestamp |

**Pydantic Model**:
```python
class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Additional error context")
    request_id: Optional[str] = Field(None, description="Request ID for tracing")
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Session not found",
                "detail": "Session ID 550e8400-e29b-41d4-a716-446655440000 does not exist or is archived",
                "request_id": "req_abc123xyz",
                "timestamp": "2025-12-29T10:30:00Z"
            }
        }
```

---

## 3. Frontend State Models (TypeScript/React)

### 3.1 ChatMessage (Frontend)

**Purpose**: Message object in React component state

**TypeScript Interface**:
```typescript
interface ChatMessage {
    id: string;  // UUID
    role: 'user' | 'assistant';
    content: string;
    timestamp: Date;
    isLoading?: boolean;  // For optimistic UI updates
    error?: string;  // If message failed to send/receive
}
```

---

### 3.2 ChatWidgetState

**Purpose**: React component state for ChatWidget

**TypeScript Interface**:
```typescript
interface ChatWidgetState {
    isOpen: boolean;
    messages: ChatMessage[];
    sessionId: string | null;  // UUID
    selectedText: string | null;
    isLoading: boolean;
    error: string | null;
    apiBaseUrl: string;
}
```

---

## 4. Data Flow & Relationships

### 4.1 Entity Relationship Diagram

```
┌─────────────────────┐
│   chat_sessions     │
│─────────────────────│
│ id (PK)            │
│ user_id            │
│ created_at         │
│ last_accessed      │
│ metadata           │
│ archived           │
└─────────────────────┘
          │
          │ 1:N
          │
          ▼
┌─────────────────────┐
│   chat_messages     │
│─────────────────────│
│ id (PK)            │
│ session_id (FK)    │─────┐
│ role               │     │
│ content            │     │ FOREIGN KEY
│ selected_text      │     │ ON DELETE CASCADE
│ timestamp          │     │
│ response_time_ms   │     │
│ error_message      │     │
└─────────────────────┘     │
          │                  │
          └──────────────────┘
```

---

### 4.2 API-to-Database Mapping

**POST /api/chat Flow**:
1. Receive `ChatRequest` from frontend
2. If `session_id` provided: Lookup in `chat_sessions`, update `last_accessed`
3. If no `session_id`: Create new row in `chat_sessions`
4. Insert user message: `INSERT INTO chat_messages (session_id, role='user', content=query, selected_text=...)`
5. Call agent with query and context
6. Insert assistant message: `INSERT INTO chat_messages (session_id, role='assistant', content=response, response_time_ms=...)`
7. Return `ChatResponse` with session_id, message_id, response

**GET /api/sessions/{id} Flow**:
1. Lookup `chat_sessions` by id
2. If not found or archived: Return 404
3. Fetch messages: `SELECT * FROM chat_messages WHERE session_id = $1 ORDER BY timestamp ASC`
4. Return `SessionResponse` with session metadata and messages

---

## 5. Validation Summary

### 5.1 Database Constraints

| Constraint | Type | Enforcement |
|------------|------|-------------|
| Non-empty query | CHECK | `LENGTH(TRIM(content)) > 0` |
| Valid role | CHECK | `role IN ('user', 'assistant')` |
| Positive response time | CHECK | `response_time_ms >= 0` |
| Selected text limit | CHECK | `LENGTH(selected_text) <= 5000` |
| Last accessed ordering | CHECK | `last_accessed >= created_at` |
| User ID format | CHECK | `user_id ~ '^[a-zA-Z0-9\-]+$'` |

### 5.2 API Validation (Pydantic)

| Validation | Field | Rule |
|------------|-------|------|
| Required fields | query | min_length=1, max_length=2000 |
| Optional UUID | session_id | UUID4 format or None |
| Text truncation | selected_text | Auto-truncate to 5000 chars |
| Boolean default | stream | Default: false |
| Whitespace trimming | query | Strip leading/trailing whitespace |

### 5.3 Frontend Validation (TypeScript)

| Validation | Field | Rule |
|------------|-------|------|
| Non-empty input | query | Must have at least 1 character after trim |
| UUID format | sessionId | Regex: `/^[0-9a-f]{8}-[0-9a-f]{4}-[4][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i` |
| Text selection | selectedText | Max 5000 chars with user warning |

---

## 6. Sample Data

### 6.1 Sample Session

**chat_sessions row**:
```json
{
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "user_id": null,
    "created_at": "2025-12-29T10:00:00Z",
    "last_accessed": "2025-12-29T10:15:00Z",
    "metadata": {
        "user_agent": "Mozilla/5.0...",
        "initial_page": "/docs/module1-ros2"
    },
    "archived": false
}
```

**chat_messages rows** (for above session):
```json
[
    {
        "id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
        "session_id": "550e8400-e29b-41d4-a716-446655440000",
        "role": "user",
        "content": "What is ROS 2?",
        "selected_text": null,
        "timestamp": "2025-12-29T10:00:05Z",
        "response_time_ms": null,
        "error_message": null
    },
    {
        "id": "8d0f7780-8536-51ef-b827-f18gd2g01bf8",
        "session_id": "550e8400-e29b-41d4-a716-446655440000",
        "role": "assistant",
        "content": "ROS 2 (Robot Operating System 2) is...",
        "selected_text": null,
        "timestamp": "2025-12-29T10:00:09Z",
        "response_time_ms": 4250.5,
        "error_message": null
    },
    {
        "id": "9e1g8891-9647-62fg-c938-g29he3h12cg9",
        "session_id": "550e8400-e29b-41d4-a716-446655440000",
        "role": "user",
        "content": "How do I install it?",
        "selected_text": null,
        "timestamp": "2025-12-29T10:15:00Z",
        "response_time_ms": null,
        "error_message": null
    },
    {
        "id": "0f2h9902-0758-73gh-d049-h30if4i23dh0",
        "session_id": "550e8400-e29b-41d4-a716-446655440000",
        "role": "assistant",
        "content": "To install ROS 2 Humble on Ubuntu...",
        "selected_text": null,
        "timestamp": "2025-12-29T10:15:05Z",
        "response_time_ms": 5123.2,
        "error_message": null
    }
]
```

---

**Data Model Status**: ✅ Complete
**Ready for API Contracts**: Yes
