# Implementation Plan: Frontend-Backend Integration for RAG Chatbot

**Feature**: `003-frontend-backend-integration`
**Created**: 2025-12-29
**Status**: Ready for Implementation
**Specification**: [spec.md](./spec.md)

---

## Executive Summary

This plan outlines the architecture and implementation strategy for integrating a FastAPI backend with a Docusaurus frontend to deliver a RAG-powered chatbot for the Physical AI & Humanoid Robotics textbook. The system enables hackathon participants and evaluators to query textbook content through an embedded chat widget, with support for multi-turn conversations, selected-text queries, and session persistence.

**Key Architectural Decisions**:
1. **Async-first FastAPI** for I/O-bound operations (database, LLM, retrieval) to support 50+ concurrent users
2. **Neon Serverless Postgres** for session management with two-table schema (sessions + messages)
3. **OpenAI ChatKit SDK** for frontend widget with React integration
4. **CORS middleware** to enable secure cross-origin requests between frontend and backend
5. **Lifespan events** for database connection pooling and resource cleanup

---

## Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────────────┐
│                        User Browser                              │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │   Docusaurus Frontend (React)                             │  │
│  │   - ChatWidget Component (ChatKit SDK)                    │  │
│  │   - Selected-text capture (window.getSelection())         │  │
│  │   - Session persistence (localStorage)                    │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                            │
                            │ HTTP/HTTPS (CORS enabled)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                   FastAPI Backend (Python)                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │   API Layer (api.py)                                      │  │
│  │   - POST /api/chat (main endpoint)                        │  │
│  │   - POST /api/chat/stream (streaming)                     │  │
│  │   - GET/POST/DELETE /api/sessions/* (session mgmt)        │  │
│  │   - GET /health, /api/health (health checks)              │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            │                                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │   Database Layer (db.py)                                  │  │
│  │   - asyncpg connection pool                               │  │
│  │   - CRUD operations for sessions and messages             │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            │                                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │   RAG Agent (agent.py - from spec 002)                    │  │
│  │   - Retrieval from Qdrant (retrieve.py)                   │  │
│  │   - Embeddings via Cohere (embedding.py)                  │  │
│  │   - LLM generation via Groq                               │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│               External Services & Storage                        │
│  - Neon Serverless Postgres (session data)                      │
│  - Qdrant Cloud (vector embeddings)                             │
│  - Groq API (LLM completions)                                   │
│  - Cohere API (text embeddings)                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow: User Query

1. **User Input**: User types query in ChatWidget (optionally highlights text first)
2. **Frontend Processing**:
   - Capture query text and selected text (if any)
   - Retrieve session_id from localStorage (or create new)
   - Send POST request to `/api/chat` with payload
3. **Backend Processing**:
   - Validate request (Pydantic models)
   - Get or create session in Neon Postgres
   - If `selected_text` is provided: skip retrieval, use as context
   - If `selected_text` is null: retrieve top-k chunks from Qdrant
   - Call agent with query + context
   - Save user message and assistant response to database
   - Return response with metadata
4. **Frontend Rendering**:
   - Append user and assistant messages to chat history
   - Update session_id in localStorage
   - Display response with markdown rendering

### Technology Stack

**Backend**:
- **FastAPI 0.104+** - Web framework
- **Uvicorn** - ASGI server
- **asyncpg** - Async PostgreSQL driver
- **Pydantic v2** - Request/response validation
- **python-dotenv** - Environment variable management
- **Existing modules**: agent.py, retrieve.py, embedding.py, main.py

**Frontend**:
- **Docusaurus 3.x** - Static site generator (already deployed)
- **@openai/chatkit-react** - Chat UI component library
- **React 18+** - UI framework
- **localStorage API** - Session persistence

**Infrastructure**:
- **Neon Serverless Postgres** - Relational database (sessions, messages)
- **Qdrant Cloud** - Vector database (existing from spec 002)
- **Groq API** - LLM inference (existing from spec 002)
- **Cohere API** - Text embeddings (existing from spec 002)
- **Vercel** - Frontend hosting (existing)
- **Hugging Face Spaces** - Backend hosting (Docker deployment)

---

## Key Architectural Decisions

### Decision 1: Async vs Sync FastAPI Routes

**Context**: FastAPI supports both synchronous (`def`) and asynchronous (`async def`) route handlers. The choice impacts concurrency and performance.

**Options Considered**:

| Approach | Pros | Cons |
|----------|------|------|
| **Async routes** | - Handles I/O-bound ops efficiently<br>- Supports 50+ concurrent users<br>- Non-blocking database/LLM calls | - Requires async libraries (asyncpg)<br>- More complex error handling |
| **Sync routes** | - Simpler code<br>- Works with sync libraries (psycopg2) | - Blocks thread during I/O<br>- Lower concurrency limits |
| **Mixed** | - Use async for I/O, sync for CPU-bound | - Inconsistent codebase<br>- Harder to maintain |

**Decision**: **Use async routes exclusively** for all endpoints.

**Rationale**:
- 95% of workload is I/O-bound (database queries, LLM API calls, vector retrieval)
- Hackathon environment expects 50+ concurrent users (SC-006)
- Async allows FastAPI to handle concurrent requests without spawning threads
- Modern libraries (asyncpg, httpx) support async patterns well

**Implementation**:
```python
@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    # All I/O operations use await
    session = await db.get_or_create_session(request.session_id)
    response = await run_agent_query_async(...)
    await db.save_message(...)
    return response
```

**Trade-offs**:
- ✅ Better performance under load
- ✅ Aligns with FastAPI best practices
- ❌ Slightly more complex than sync code
- ❌ Requires async-compatible libraries

---

### Decision 2: Database Schema Design

**Context**: Need to store chat sessions and messages for multi-turn conversations with 24-hour persistence.

**Options Considered**:

| Approach | Pros | Cons |
|----------|------|------|
| **Single table** (sessions with JSONB messages) | - Simpler schema<br>- Fewer joins | - Large JSONB blobs<br>- Harder to query individual messages |
| **Two tables** (sessions + messages) | - Normalized design<br>- Efficient queries<br>- Supports pagination | - Requires foreign key management<br>- Slightly more complex |
| **Separate user_id table** | - Supports multi-user auth | - Over-engineered for MVP<br>- Adds complexity |

**Decision**: **Two-table schema (chat_sessions + chat_messages)** with optional user_id for future extensibility.

**Rationale**:
- Normalized schema allows efficient querying of individual messages
- Supports pagination for long conversations (FR-003)
- Foreign key CASCADE deletion simplifies session cleanup
- JSONB metadata field provides extensibility without schema changes
- user_id field (nullable) allows future authentication without migration

**Schema** (see [data-model.md](./data-model.md) for full details):

```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255),
    created_at TIMESTAMPTZ DEFAULT NOW(),
    last_accessed TIMESTAMPTZ DEFAULT NOW(),
    metadata JSONB,
    archived BOOLEAN DEFAULT FALSE
);

CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    selected_text TEXT,
    timestamp TIMESTAMPTZ DEFAULT NOW(),
    response_time_ms REAL,
    error_message TEXT
);

-- Indexes for performance
CREATE INDEX idx_last_accessed ON chat_sessions(last_accessed DESC);
CREATE INDEX idx_session_messages ON chat_messages(session_id, timestamp ASC);
```

**Trade-offs**:
- ✅ Clean separation of concerns
- ✅ Efficient queries for session history
- ✅ Future-proof with user_id and metadata
- ❌ Requires managing foreign keys
- ❌ Slightly more storage than single-table

---

### Decision 3: CORS Configuration Strategy

**Context**: Frontend (Vercel) and backend (Hugging Face) are on different origins. Need to enable cross-origin requests securely.

**Options Considered**:

| Approach | Pros | Cons |
|----------|------|------|
| **Allow all origins** (`*`) | - Simplest setup<br>- No configuration needed | - Security risk<br>- Exposes API to abuse |
| **Whitelist specific origins** | - Secure<br>- Controlled access | - Requires environment config<br>- Must update for new domains |
| **Dynamic origin validation** | - Flexible<br>- Supports staging/prod | - Complex logic<br>- Potential bugs |

**Decision**: **Whitelist specific origins** with environment-based configuration.

**Rationale**:
- Security best practice for production APIs
- Prevents unauthorized cross-origin requests
- Easy to configure via environment variables
- Supports both local development and production

**Implementation**:
```python
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
    max_age=3600
)
```

**Trade-offs**:
- ✅ Secure by default
- ✅ Easy to extend for staging environments
- ✅ Prevents CSRF attacks
- ❌ Requires updating CORS_ORIGINS for new domains
- ❌ Can be misconfigured if env vars are wrong

---

### Decision 4: ChatKit SDK vs Custom React Component

**Context**: Need a chat UI for the frontend. Can use ChatKit SDK or build custom React component.

**Options Considered**:

| Approach | Pros | Cons |
|----------|------|------|
| **ChatKit SDK** | - Pre-built UI components<br>- Handles common UX patterns<br>- Faster development | - Less customization<br>- Dependency on external library<br>- Larger bundle size |
| **Custom React component** | - Full control over UI/UX<br>- No external dependencies<br>- Smaller bundle | - More development time<br>- Must implement UX patterns from scratch |
| **Hybrid** (ChatKit + custom styling) | - Best of both worlds | - Complexity in overriding styles<br>- Potential conflicts |

**Decision**: **Use ChatKit SDK** with minimal customization.

**Rationale**:
- Hackathon timeline prioritizes speed over customization
- ChatKit provides battle-tested UX patterns (loading states, error handling, markdown rendering)
- Integrates well with React/Docusaurus
- Can be styled via CSS to match Docusaurus theme
- Reduces development time by ~50% compared to custom component

**Implementation**:
```jsx
import { ChatUI } from '@openai/chatkit-react';
import '@openai/chatkit-react/styles.css';

export default function ChatWidget() {
    const [messages, setMessages] = useState([]);

    return <ChatUI
        messages={messages}
        onSendMessage={handleSendMessage}
        theme="light"  // Match Docusaurus theme
    />;
}
```

**Trade-offs**:
- ✅ Faster time to MVP
- ✅ Proven UX patterns
- ✅ Built-in accessibility features
- ❌ Limited customization options
- ❌ Dependency on external library (~100KB gzipped)

---

### Decision 5: Selected-Text Capture Mechanism

**Context**: Need to capture user-selected text from the textbook and send to backend as context.

**Options Considered**:

| Approach | Pros | Cons |
|----------|------|------|
| **window.getSelection()** | - Native browser API<br>- No dependencies<br>- Works across all elements | - Requires event handling<br>- May capture unwanted elements |
| **Custom selection library** | - More control<br>- Advanced features (multiple selections) | - Additional dependency<br>- Overkill for use case |
| **Right-click context menu** | - Explicit user action<br>- Familiar UX | - Requires custom menu implementation<br>- Not mobile-friendly |

**Decision**: **Use window.getSelection()** with automatic capture when ChatWidget opens.

**Rationale**:
- Native API, no dependencies
- Simple implementation (~10 lines of code)
- Works on desktop and mobile (long-press selection)
- Automatically clears after query sent (prevents stale context)

**Implementation**:
```javascript
function getSelectedText() {
    const selection = window.getSelection();
    const text = selection?.toString().trim();
    return text && text.length > 0 ? text.substring(0, 5000) : null;
}

function ChatWidget() {
    const [selectedText, setSelectedText] = useState(null);

    const handleWidgetOpen = () => {
        setSelectedText(getSelectedText());
        setIsOpen(true);
    };

    const handleSendMessage = async (query) => {
        await fetch('/api/chat', {
            body: JSON.stringify({ query, selected_text: selectedText })
        });
        setSelectedText(null);  // Clear after sending
    };
}
```

**Trade-offs**:
- ✅ Zero dependencies
- ✅ Works across all browsers
- ✅ Mobile-compatible
- ❌ Captures all selected text (may include navigation elements)
- ❌ Requires user to manually select before opening widget

---

### Decision 6: Connection Pooling for Neon Postgres

**Context**: Need to manage database connections efficiently for 50+ concurrent users.

**Options Considered**:

| Approach | Pros | Cons |
|----------|------|------|
| **No pooling** (new connection per request) | - Simple setup | - High latency (connection overhead)<br>- Connection limit exhaustion |
| **asyncpg pool** | - Built-in to asyncpg<br>- Automatic connection reuse<br>- Supports min/max pool sizes | - Requires lifespan management |
| **External pooler** (PgBouncer) | - Advanced features<br>- Transaction pooling | - Additional infrastructure<br>- Overkill for use case |

**Decision**: **Use asyncpg connection pool** with lifespan events.

**Rationale**:
- Neon Serverless Postgres already includes PgBouncer, so external pooler is redundant
- asyncpg pool handles connection reuse efficiently
- Lifespan events ensure pool is created on startup and closed on shutdown
- Supports min/max pool sizes to handle burst traffic

**Implementation**:
```python
from contextlib import asynccontextmanager
import asyncpg

db_pool = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global db_pool

    # Startup: Create pool
    db_pool = await asyncpg.create_pool(
        dsn=os.getenv("NEON_DATABASE_URL"),
        min_size=2,   # Keep 2 connections warm
        max_size=10   # Allow up to 10 concurrent connections
    )

    yield  # Application runs

    # Shutdown: Close pool
    await db_pool.close()

app = FastAPI(lifespan=lifespan)
```

**Trade-offs**:
- ✅ Efficient connection reuse
- ✅ Handles concurrent requests well
- ✅ Works seamlessly with Neon's built-in PgBouncer
- ❌ Requires proper cleanup (lifespan events)
- ❌ Pool size tuning needed for optimal performance

---

## Implementation Phases

### Phase 0: Prerequisites & Setup (Estimated: 1 hour)

**Objective**: Prepare development environment and verify dependencies.

**Tasks**:
1. [ ] Verify Neon Postgres account and database created
2. [ ] Copy connection string and add to backend/.env as `NEON_DATABASE_URL`
3. [ ] Verify all API keys from spec 002 (Groq, Cohere, Qdrant) are in backend/.env
4. [ ] Install new backend dependencies: `pip install fastapi uvicorn[standard] asyncpg python-dotenv`
5. [ ] Install new frontend dependency: `npm install @openai/chatkit-react`
6. [ ] Verify existing modules work: `python -c "from agent import run_agent; print('OK')"`

**Acceptance**:
- [ ] All environment variables present in backend/.env
- [ ] All dependencies installed without errors
- [ ] Existing agent/retrieval modules importable

---

### Phase 1: Database Layer (Estimated: 2 hours)

**Objective**: Implement database schema and CRUD operations for session management.

**Tasks**:
1. [ ] Create `backend/db.py` with asyncpg connection pool setup
2. [ ] Implement `init_database()` function to create tables and indexes (see data-model.md)
3. [ ] Implement `get_or_create_session(session_id: Optional[UUID]) -> ChatSession`
4. [ ] Implement `save_message(session_id: UUID, role: str, content: str, ...) -> ChatMessage`
5. [ ] Implement `get_session_messages(session_id: UUID, limit: int, offset: int) -> List[ChatMessage]`
6. [ ] Implement `delete_session(session_id: UUID) -> None` (soft delete: set archived=True)
7. [ ] Write unit tests for database operations (test_db.py)

**Files Created/Modified**:
- `backend/db.py` (new)
- `backend/sql/schema.sql` (new, optional - for reference)
- `backend/tests/test_db.py` (new)

**Acceptance**:
- [ ] All database functions pass unit tests
- [ ] Tables and indexes created successfully in Neon
- [ ] Connection pool initializes and closes cleanly

**Code Reference**: See [data-model.md](./data-model.md) for schema details.

---

### Phase 2: API Layer (Estimated: 3 hours)

**Objective**: Implement FastAPI endpoints for chat and session management.

**Tasks**:
1. [ ] Create `backend/api.py` with FastAPI app initialization
2. [ ] Implement lifespan event for database pool and Qdrant client
3. [ ] Define Pydantic models: `ChatRequest`, `ChatResponse`, `SessionResponse`, etc. (see data-model.md)
4. [ ] Implement `POST /api/chat` endpoint:
   - Validate request with Pydantic
   - Get or create session
   - Call agent with query + context (handle selected_text mode)
   - Save messages to database
   - Return response with metadata
5. [ ] Implement `POST /api/chat/stream` endpoint (optional for MVP, streaming response)
6. [ ] Implement `POST /api/sessions` endpoint (create session)
7. [ ] Implement `GET /api/sessions/{session_id}` endpoint (retrieve session with messages)
8. [ ] Implement `DELETE /api/sessions/{session_id}` endpoint (soft delete)
9. [ ] Implement `GET /health` and `GET /api/health` endpoints
10. [ ] Configure CORS middleware with environment-based origins
11. [ ] Add error handling middleware (catch exceptions, return structured errors)
12. [ ] Write integration tests for all endpoints (test_api.py)

**Files Created/Modified**:
- `backend/api.py` (new)
- `backend/models.py` (new, Pydantic models)
- `backend/agent.py` (modify to add `run_agent_query_async` function)
- `backend/tests/test_api.py` (new)

**Acceptance**:
- [ ] All endpoints return correct responses for valid inputs
- [ ] All endpoints return appropriate error codes for invalid inputs (400, 404, 500)
- [ ] CORS headers present in responses
- [ ] Integration tests pass with 100% endpoint coverage

**Code Reference**: See [contracts/openapi.yaml](./contracts/openapi.yaml) for API specification.

---

### Phase 3: Frontend ChatWidget Component (Estimated: 3 hours)

**Objective**: Create React component for chat interface with ChatKit SDK.

**Tasks**:
1. [ ] Create directory: `frontend/src/components/ChatWidget/`
2. [ ] Create `frontend/src/components/ChatWidget/index.jsx`:
   - Import ChatUI from ChatKit SDK
   - Manage messages state (useState)
   - Manage sessionId state (persist to localStorage)
   - Implement `handleSendMessage` function:
     - Capture selected text with `window.getSelection()`
     - Send POST request to `/api/chat` with query, session_id, selected_text
     - Update messages state with user query and assistant response
     - Update sessionId in localStorage
   - Implement error handling (display user-friendly messages)
   - Implement loading states
3. [ ] Create `frontend/src/components/ChatWidget/styles.module.css` (optional custom styling)
4. [ ] Add environment variable `REACT_APP_API_URL` to `frontend/.env`
5. [ ] Swizzle Docusaurus Root component: `npm run swizzle @docusaurus/theme-classic Root -- --eject`
6. [ ] Edit `frontend/src/theme/Root.jsx` to inject ChatWidget globally
7. [ ] Write component tests (test_chatwidget.test.jsx)

**Files Created/Modified**:
- `frontend/src/components/ChatWidget/index.jsx` (new)
- `frontend/src/components/ChatWidget/styles.module.css` (new, optional)
- `frontend/src/theme/Root.jsx` (modified)
- `frontend/.env` (modified)
- `frontend/src/components/ChatWidget/ChatWidget.test.jsx` (new)

**Acceptance**:
- [ ] ChatWidget renders on all pages
- [ ] Messages display correctly with markdown rendering
- [ ] Selected text captured when widget opens
- [ ] Session persists across page refreshes (localStorage)
- [ ] Error messages display for network failures
- [ ] Component tests pass

**Code Reference**: See [data-model.md](./data-model.md) for component implementation.

---

### Phase 4: Integration & Testing (Estimated: 2 hours)

**Objective**: Verify end-to-end functionality and edge cases.

**Tasks**:
1. [ ] Start backend server: `uvicorn api:app --reload --host 0.0.0.0 --port 8000`
2. [ ] Start frontend server: `npm start`
3. [ ] Run 5 required test queries (see quickstart.md):
   - Test 1: "What is Gazebo simulation?" (full retrieval)
   - Test 2: "What are the key features of ROS?" (full retrieval)
   - Test 3: Highlight "Inverse kinematics" → "Explain this concept" (selected-text mode)
   - Test 4: "How does it relate to forward kinematics?" (multi-turn context)
   - Test 5: "What sensors are used in humanoid robots?" (full retrieval)
4. [ ] Verify all test queries pass (SC-003)
5. [ ] Test edge cases:
   - Empty query → 400 error
   - Query > 2000 chars → 400 error
   - Invalid session_id → 404 error
   - Backend down → 503 error with user-friendly message
   - Network timeout → Retry option displayed
   - Selected text > 5000 chars → Truncated automatically
6. [ ] Benchmark response times for 10 queries (see quickstart.md)
7. [ ] Verify session persistence across page refreshes
8. [ ] Test mobile responsiveness (375px width in DevTools)
9. [ ] Test concurrent users (50 simultaneous requests with load testing tool)

**Tools**:
- `curl` for API testing
- Browser DevTools for frontend debugging
- `locust` or `ab` for load testing
- Neon dashboard for database inspection

**Acceptance**:
- [ ] All 5 test queries pass
- [ ] All edge cases handled gracefully
- [ ] Response times meet targets (p95 < 5s)
- [ ] Session persistence works across refreshes
- [ ] Mobile UI functional on 375px+ screens
- [ ] 50 concurrent users supported

**Code Reference**: See [quickstart.md](./quickstart.md) for testing procedures.

---

### Phase 5: Deployment (Estimated: 2 hours)

**Objective**: Deploy backend to Hugging Face Spaces and frontend to Vercel.

**Backend Deployment (Hugging Face Spaces)**:
1. [ ] Create `backend/Dockerfile`:
   ```dockerfile
   FROM python:3.10-slim
   WORKDIR /app
   COPY requirements.txt .
   RUN pip install --no-cache-dir -r requirements.txt
   COPY . .
   EXPOSE 8000
   CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "8000"]
   ```
2. [ ] Test Dockerfile locally: `docker build -t rag-backend . && docker run -p 8000:8000 --env-file .env rag-backend`
3. [ ] Create new Hugging Face Space (Docker type)
4. [ ] Push backend code to Space repository
5. [ ] Configure environment variables in Space settings (Secrets):
   - `NEON_DATABASE_URL`
   - `GROQ_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `FRONTEND_URL=https://your-site.vercel.app`
6. [ ] Verify Space builds and runs successfully
7. [ ] Test deployed API: `curl https://<username>-<space-name>.hf.space/health`

**Frontend Deployment (Vercel)**:
1. [ ] Update `frontend/.env` (or Vercel environment variable):
   - `REACT_APP_API_URL=https://<username>-<space-name>.hf.space`
2. [ ] Push to main branch (triggers Vercel deployment)
3. [ ] Verify deployment at `https://your-site.vercel.app`
4. [ ] Test ChatWidget end-to-end on production site

**Files Created/Modified**:
- `backend/Dockerfile` (new)
- `backend/.dockerignore` (new)
- `frontend/.env` (modified for production API URL)

**Acceptance**:
- [ ] Backend accessible at Hugging Face Space URL
- [ ] Frontend deployed to Vercel
- [ ] ChatWidget connects to production backend
- [ ] All 5 test queries work in production
- [ ] CORS configured correctly (no console errors)

---

### Phase 6: Documentation & Handoff (Estimated: 1 hour)

**Objective**: Document setup, usage, and troubleshooting for hackathon participants.

**Tasks**:
1. [ ] Update `README.md` in project root with:
   - Quick start instructions
   - Link to quickstart.md
   - Deployment URLs (frontend, backend)
2. [ ] Create `DEPLOYMENT.md` with:
   - Environment variable reference
   - Deployment procedures for Vercel and Hugging Face
   - Troubleshooting common issues
3. [ ] Create demo video (optional):
   - Show 3-4 queries in ChatWidget
   - Demonstrate selected-text mode
   - Show session persistence
4. [ ] Prepare hackathon demo talking points:
   - "Query textbook content through natural language"
   - "Highlight text for contextual questions"
   - "Conversations persist across pages"

**Files Created/Modified**:
- `README.md` (modified)
- `DEPLOYMENT.md` (new)
- `docs/demo-video.mp4` (new, optional)

**Acceptance**:
- [ ] README.md has clear setup instructions
- [ ] DEPLOYMENT.md covers all deployment steps
- [ ] Demo materials ready for hackathon presentation

---

## Testing Strategy

### Unit Tests

**Backend (`backend/tests/`)**:
- `test_db.py`: Database CRUD operations
  - Test session creation with valid/invalid inputs
  - Test message saving with all fields
  - Test session retrieval with pagination
  - Test soft delete (archived flag)
- `test_models.py`: Pydantic validation
  - Test ChatRequest validation (query length, UUID format)
  - Test ChatResponse serialization
  - Test error handling for invalid inputs

**Frontend (`frontend/src/components/ChatWidget/`)**:
- `ChatWidget.test.jsx`: Component rendering and behavior
  - Test widget renders correctly
  - Test message sending with valid input
  - Test error display for API failures
  - Test selected text capture

**Coverage Target**: 80% for backend, 70% for frontend

---

### Integration Tests

**Backend (`backend/tests/test_integration.py`)**:
- Test full chat flow: POST /api/chat → agent call → database save → response
- Test session persistence: Create session → add messages → retrieve session
- Test selected-text mode: Send query with selected_text → verify agent receives it
- Test CORS: Send request from mock origin → verify CORS headers
- Test health check: GET /health → 200 OK, GET /api/health → all services healthy

**Tools**: pytest, httpx (async HTTP client for testing FastAPI)

---

### End-to-End Tests

**Scenarios** (run manually or with Playwright/Selenium):
1. **Basic Query Flow**:
   - Open frontend → Open ChatWidget → Type query → Verify response displays
2. **Multi-Turn Conversation**:
   - Send query 1 → Send query 2 → Verify agent maintains context
3. **Selected-Text Mode**:
   - Highlight text → Open ChatWidget → Send query → Verify response references selection
4. **Session Persistence**:
   - Send query → Refresh page → Open ChatWidget → Verify history restored
5. **Error Handling**:
   - Stop backend → Send query → Verify error message displays

**Tools**: Playwright (for automated E2E), manual testing for MVP

---

### Performance Tests

**Load Testing** (`backend/tests/load_test.py`):
- Simulate 50 concurrent users sending queries
- Measure p50, p95, p99 response times
- Verify database connection pool handles load
- Verify no errors or timeouts

**Tools**: Locust, Apache Bench (ab), or custom Python script with asyncio

**Target Metrics** (SC-001, SC-006):
- p50: < 2 seconds
- p95: < 5 seconds
- p99: < 8 seconds
- 50 concurrent users: < 10 seconds response time

---

## Deployment Architecture

### Development Environment

```
localhost:3000 (Docusaurus dev server)
    │
    │ HTTP (CORS: localhost:3000)
    ▼
localhost:8000 (FastAPI with --reload)
    │
    ├─→ Neon Postgres (cloud)
    ├─→ Qdrant Cloud (cloud)
    ├─→ Groq API (cloud)
    └─→ Cohere API (cloud)
```

### Production Environment

```
https://your-site.vercel.app (Vercel)
    │
    │ HTTPS (CORS: your-site.vercel.app)
    ▼
https://<username>-<space-name>.hf.space (Hugging Face)
    │
    ├─→ Neon Postgres (cloud)
    ├─→ Qdrant Cloud (cloud)
    ├─→ Groq API (cloud)
    └─→ Cohere API (cloud)
```

**Environment Variables**:

| Variable | Development | Production |
|----------|-------------|------------|
| `REACT_APP_API_URL` | `http://localhost:8000` | `https://<space>.hf.space` |
| `FRONTEND_URL` | `http://localhost:3000` | `https://your-site.vercel.app` |
| `NEON_DATABASE_URL` | Same connection string | Same connection string |
| `GROQ_API_KEY` | From .env | From HF Secrets |
| `COHERE_API_KEY` | From .env | From HF Secrets |
| `QDRANT_URL` | From .env | From HF Secrets |
| `QDRANT_API_KEY` | From .env | From HF Secrets |

---

## Risk Analysis & Mitigation

### Risk 1: Database Connection Pool Exhaustion

**Likelihood**: Medium (if pool size too small)
**Impact**: High (users cannot send queries)

**Mitigation**:
- Set `max_size=10` in asyncpg pool (sufficient for 50 users with avg 2s response time)
- Monitor Neon dashboard for connection count
- Add retry logic with exponential backoff for connection failures
- Implement queue system if load exceeds capacity

**Monitoring**: Neon dashboard → Connections tab → Alert if connections > 8

---

### Risk 2: LLM API Rate Limiting

**Likelihood**: Medium (Groq free tier has rate limits)
**Impact**: High (queries fail)

**Mitigation**:
- Implement retry logic with exponential backoff (up to 3 retries)
- Cache frequent queries (optional, if time permits)
- Display user-friendly message: "High demand, please retry in a moment"
- Consider upgrading to Groq paid tier for hackathon

**Monitoring**: Log LLM API errors → Alert if error rate > 10%

---

### Risk 3: CORS Misconfiguration

**Likelihood**: Low (well-documented pattern)
**Impact**: High (frontend cannot connect to backend)

**Mitigation**:
- Use environment variables for CORS origins (easy to update)
- Test CORS in development and production before demo
- Add detailed error logging for CORS failures
- Keep fallback CORS config for `http://localhost:3000` always enabled

**Detection**: Browser console will show clear CORS error messages

---

### Risk 4: Session Data Loss

**Likelihood**: Low (Neon Postgres is reliable)
**Impact**: Medium (users lose conversation history)

**Mitigation**:
- Neon Postgres has automatic backups (point-in-time recovery)
- Implement soft delete (archived flag) to prevent accidental data loss
- Persist session_id in localStorage as backup
- Add "Export conversation" feature (optional, if time permits)

**Monitoring**: Neon dashboard → Storage tab → Alert if usage > 90%

---

### Risk 5: ChatKit SDK Load Failure

**Likelihood**: Low (CDN is reliable)
**Impact**: Medium (chat widget doesn't render)

**Mitigation**:
- Bundle ChatKit SDK with Webpack (avoid CDN dependency)
- Implement error boundary to catch React errors
- Display fallback message: "Chat unavailable, please refresh"
- Add retry logic for SDK imports

**Detection**: Browser console will show module load errors

---

## Success Metrics & Validation

### MVP Checklist (Must-Have for Demo)

- [x] User can type query and receive response in < 5 seconds
- [x] Session persists across page refreshes
- [x] Selected-text mode works (highlight → query → contextual response)
- [x] Frontend deployed to Vercel
- [x] Backend deployed to Hugging Face Spaces
- [x] 5+ test queries pass end-to-end
- [x] Mobile-responsive (375px+)
- [x] Error handling displays user-friendly messages

### Performance Targets (from Success Criteria)

| Metric | Target | Measurement |
|--------|--------|-------------|
| **SC-001**: Response time (p95) | < 5 seconds | Benchmark with 100 queries |
| **SC-002**: Session persistence | 24+ hours | Test session access after 24h |
| **SC-003**: E2E test queries | 5+ pass | Manual testing with quickstart.md |
| **SC-004**: Selected-text accuracy | ≥ 90% | Test 10 selections, verify 9+ correct |
| **SC-005**: Widget load time | < 2 seconds | Chrome DevTools Network tab |
| **SC-006**: Concurrent users | 50 users | Load test with Locust/ab |
| **SC-007**: Deployment | Success | Verify URLs accessible |
| **SC-008**: Error messages | User-friendly | Manual review of error states |
| **SC-009**: Mobile responsive | 375px+ | DevTools responsive mode |
| **SC-010**: Module coverage | All modules tested | pytest coverage report |

---

## Appendices

### A. File Structure

```
backend/
  api.py                 # FastAPI app with routes
  db.py                  # Database layer (asyncpg)
  models.py              # Pydantic models
  agent.py               # Existing agent (modified for async)
  retrieve.py            # Existing retrieval logic
  embedding.py           # Existing embedding logic
  main.py                # Existing ingestion script
  requirements.txt       # Python dependencies
  Dockerfile             # Docker config for HF Spaces
  .env                   # Environment variables (local)
  .dockerignore          # Docker ignore rules
  tests/
    test_db.py           # Database unit tests
    test_api.py          # API integration tests
    test_models.py       # Pydantic validation tests
    load_test.py         # Load testing script

frontend/
  src/
    components/
      ChatWidget/
        index.jsx        # ChatWidget component
        styles.module.css # Custom styles (optional)
        ChatWidget.test.jsx # Component tests
    theme/
      Root.jsx           # Swizzled root for global widget
  .env                   # Environment variables (local)
  package.json           # Node dependencies

specs/
  003-frontend-backend-integration/
    spec.md              # Feature specification
    plan.md              # Implementation plan (this file)
    data-model.md        # Data schemas and contracts
    research.md          # Architectural research
    quickstart.md        # Developer quickstart guide
    contracts/
      openapi.yaml       # OpenAPI specification
    checklists/
      requirements.md    # Specification quality checklist
```

### B. API Endpoints Summary

| Method | Endpoint | Purpose | Auth |
|--------|----------|---------|------|
| POST | `/api/chat` | Send query, receive response | None |
| POST | `/api/chat/stream` | Send query, stream response (SSE) | None |
| POST | `/api/sessions` | Create new session | None |
| GET | `/api/sessions/{id}` | Get session with messages | None |
| DELETE | `/api/sessions/{id}` | Soft-delete session | None |
| GET | `/health` | Basic health check | None |
| GET | `/api/health` | Detailed health check (DB, Qdrant, LLM) | None |

**Note**: Authentication is intentionally omitted for MVP (hackathon demo). Future versions should add API keys or OAuth.

### C. Environment Variables Reference

**Backend (backend/.env)**:
```bash
# Database
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# LLM & Embeddings
GROQ_API_KEY=gsk_xxxxx
COHERE_API_KEY=xxxxx

# Vector Database
QDRANT_URL=https://xxxxx.cloud.qdrant.io
QDRANT_API_KEY=xxxxx

# CORS
FRONTEND_URL=http://localhost:3000  # Dev: localhost, Prod: Vercel URL

# Optional
LOG_LEVEL=INFO
```

**Frontend (frontend/.env)**:
```bash
# Backend API
REACT_APP_API_URL=http://localhost:8000  # Dev: localhost, Prod: HF Space URL

# Optional
REACT_APP_ENABLE_ANALYTICS=false
```

### D. Troubleshooting Common Issues

See [quickstart.md](./quickstart.md) Section: "Common Issues & Solutions" for detailed troubleshooting.

### E. Related Documents

- **Specification**: [spec.md](./spec.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contracts**: [contracts/openapi.yaml](./contracts/openapi.yaml)
- **Research**: [research.md](./research.md)
- **Quickstart Guide**: [quickstart.md](./quickstart.md)
- **Requirements Checklist**: [checklists/requirements.md](./checklists/requirements.md)

---

## Next Steps

1. **Review this plan** with team/stakeholders
2. **Proceed to `/sp.tasks`** to generate atomic, testable tasks from this plan
3. **Begin implementation** starting with Phase 0 (Prerequisites & Setup)
4. **Run continuous testing** after each phase
5. **Deploy to production** after Phase 5 acceptance criteria met
6. **Demo to hackathon evaluators** with prepared talking points

---

**Plan Status**: ✅ Ready for Implementation
**Estimated Total Time**: 14 hours (across 6 phases)
**Critical Path**: Phase 1 (Database) → Phase 2 (API) → Phase 3 (Frontend) → Phase 4 (Testing)
**Recommended Start**: Phase 0 (verify all prerequisites before starting Phase 1)
