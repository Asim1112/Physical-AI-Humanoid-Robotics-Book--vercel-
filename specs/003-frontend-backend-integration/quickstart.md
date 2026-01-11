# Quickstart Guide: Frontend-Backend Integration

**Feature**: RAG Chatbot Integration for Physical AI & Humanoid Robotics Textbook
**Last Updated**: 2025-12-29
**Estimated Setup Time**: 30 minutes

## Overview

This guide walks you through setting up the complete RAG chatbot system locally, including:
- FastAPI backend with RAG agent integration
- Neon Serverless Postgres for session management
- Docusaurus frontend with embedded ChatKit widget
- End-to-end testing with selected-text mode

## Prerequisites

### Required Software
- **Python 3.10+** (for backend)
- **Node.js 18+** and **npm/yarn** (for frontend)
- **Git** (for version control)
- **PostgreSQL client** (optional, for database inspection)

### Required Accounts & API Keys
1. **Neon Serverless Postgres Account** (free tier available)
   - Sign up at https://neon.tech
   - Create a new project and database
   - Copy the connection string (format: `postgresql://user:pass@host/dbname?sslmode=require`)

2. **Groq API Key** (for LLM)
   - Sign up at https://groq.com
   - Generate API key from dashboard

3. **Cohere API Key** (for embeddings)
   - Sign up at https://cohere.com
   - Generate API key from dashboard

4. **Qdrant Cloud** (for vector storage)
   - Already configured from prior specs (reuse existing endpoint and API key)

### Environment Variables Checklist
Create a checklist of all required environment variables:
- [ ] `NEON_DATABASE_URL` - Neon Postgres connection string
- [ ] `GROQ_API_KEY` - Groq LLM API key
- [ ] `COHERE_API_KEY` - Cohere embeddings API key
- [ ] `QDRANT_URL` - Qdrant endpoint (from prior setup)
- [ ] `QDRANT_API_KEY` - Qdrant API key (from prior setup)
- [ ] `FRONTEND_URL` - Frontend URL for CORS (e.g., `http://localhost:3000`)

---

## Part 1: Backend Setup (FastAPI + Neon Postgres)

### Step 1: Navigate to Backend Directory

```bash
cd backend
```

### Step 2: Create Python Virtual Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
```

### Step 3: Install Dependencies

```bash
# Install existing dependencies
pip install -r requirements.txt

# Install new dependencies for this feature
pip install fastapi uvicorn[standard] asyncpg python-dotenv pydantic[email]
```

**Expected Output**: All packages installed successfully, including:
- `fastapi` (web framework)
- `uvicorn` (ASGI server)
- `asyncpg` (async PostgreSQL driver)
- `pydantic` (validation)

### Step 4: Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
# backend/.env

# Database
NEON_DATABASE_URL=postgresql://user:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# LLM & Embeddings
GROQ_API_KEY=gsk_xxxxxxxxxxxxxxxxxxxxxxxxxxxxx
COHERE_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Vector Database (from prior setup)
QDRANT_URL=https://xxxxx.cloud.qdrant.io
QDRANT_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# CORS
FRONTEND_URL=http://localhost:3000

# Optional: Logging
LOG_LEVEL=INFO
```

**Security Note**: Never commit `.env` to version control. Ensure `.gitignore` includes `.env`.

### Step 5: Initialize Database Schema

Create the database tables by running the initialization script:

```bash
# Option 1: Using Python script (recommended)
python -c "from db import init_database; import asyncio; asyncio.run(init_database())"

# Option 2: Using SQL file directly (if provided)
# psql $NEON_DATABASE_URL -f sql/schema.sql
```

**Expected Output**:
```
✓ Connected to Neon Postgres
✓ Created table: chat_sessions
✓ Created table: chat_messages
✓ Created indexes: idx_last_accessed, idx_user_sessions, idx_session_messages, idx_recent_messages
✓ Database initialization complete
```

**Verification**: Check that tables exist:
```bash
psql $NEON_DATABASE_URL -c "\dt"
```

Should show:
```
 Schema |      Name       | Type  | Owner
--------+-----------------+-------+-------
 public | chat_sessions   | table | user
 public | chat_messages   | table | user
```

### Step 6: Test Backend Components

Verify that all prior components (agent, retrieval, embedding) are working:

```bash
# Test retrieval
python -c "from retrieve import retrieve_content; print(retrieve_content('Gazebo simulation', top_k=3))"

# Test embedding
python -c "from embedding import generate_embedding; import numpy as np; vec = generate_embedding('test'); print(f'Embedding shape: {np.array(vec).shape}')"

# Test agent (if available)
python -c "from agent import run_agent; print(run_agent('What is ROS?'))"
```

**Expected Output**: Each command should complete without errors and return relevant data.

### Step 7: Start Backend Server

```bash
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Verification**: Open http://localhost:8000/health in browser, should see:
```json
{
  "status": "ok",
  "timestamp": "2025-12-29T10:30:00Z"
}
```

### Step 8: Test API Endpoints

Open a new terminal (keep server running) and test the chat endpoint:

```bash
# Test basic chat query
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Gazebo simulation?",
    "session_id": null,
    "selected_text": null,
    "stream": false
  }'
```

**Expected Response** (example):
```json
{
  "response": "Gazebo is an open-source 3D robotics simulator that provides realistic physics simulation, high-quality graphics, and sensor simulation for testing robot algorithms in realistic environments without requiring physical hardware.",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "message_id": "660e8400-e29b-41d4-a716-446655440001",
  "retrieved_chunks": 5,
  "response_time_ms": 2341.5,
  "error": null
}
```

**Troubleshooting**:
- **Error: "Database connection failed"** → Check `NEON_DATABASE_URL` in `.env`
- **Error: "Qdrant connection failed"** → Verify `QDRANT_URL` and `QDRANT_API_KEY`
- **Error: "LLM API error"** → Check `GROQ_API_KEY` validity and quota
- **Error: "Module not found"** → Ensure virtual environment is activated and dependencies installed

---

## Part 2: Frontend Setup (Docusaurus + ChatKit)

### Step 1: Navigate to Frontend Directory

```bash
cd ../frontend  # or: cd frontend (from project root)
```

### Step 2: Install Dependencies

```bash
# Install existing dependencies
npm install

# Install ChatKit SDK
npm install @openai/chatkit-react

# Optional: Install additional UI libraries for styling
npm install @heroicons/react  # For icons
```

**Expected Output**: All packages installed successfully, including `@openai/chatkit-react`.

### Step 3: Configure Environment Variables

Create a `.env` file in the `frontend/` directory:

```bash
# frontend/.env

# Backend API URL
REACT_APP_API_URL=http://localhost:8000

# Optional: Analytics, feature flags, etc.
REACT_APP_ENABLE_ANALYTICS=false
```

**Note**: Docusaurus uses environment variables prefixed with `REACT_APP_` for runtime access.

### Step 4: Create ChatWidget Component

Create the directory structure:

```bash
mkdir -p src/components/ChatWidget
```

Create `src/components/ChatWidget/index.jsx` (see data-model.md for full implementation).

**Key Features** to verify in the component:
- [x] Imports `ChatUI` from `@openai/chatkit-react`
- [x] Manages `messages` state with useState
- [x] Manages `sessionId` state (persisted to localStorage)
- [x] Implements `handleSendMessage` function that calls `/api/chat`
- [x] Captures selected text with `window.getSelection()`
- [x] Handles errors gracefully with error boundaries
- [x] Responsive design (mobile-friendly)

### Step 5: Integrate ChatWidget into Docusaurus

**Option 1: Theme Swizzling (Recommended)**

Swizzle the Root component to inject ChatWidget globally:

```bash
npm run swizzle @docusaurus/theme-classic Root -- --eject
```

Edit `src/theme/Root.jsx`:

```jsx
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

**Option 2: Manual Integration (Alternative)**

Import ChatWidget in specific pages (e.g., `src/pages/index.jsx`):

```jsx
import ChatWidget from '@site/src/components/ChatWidget';

export default function Home() {
  return (
    <Layout>
      {/* Existing content */}
      <ChatWidget />
    </Layout>
  );
}
```

### Step 6: Start Frontend Development Server

```bash
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/

✔ Client
  Compiled successfully in 3.45s
```

**Verification**: Open http://localhost:3000 in browser, should see:
- Docusaurus homepage rendered
- ChatWidget visible (floating button or embedded widget)

### Step 7: Test Chat Widget

**Test Case 1: Basic Query**
1. Open http://localhost:3000
2. Open ChatWidget (click button or widget)
3. Type: "What is Gazebo simulation?"
4. Click Send
5. **Expected**: Response appears within 5 seconds with relevant answer

**Test Case 2: Multi-Turn Conversation**
1. Continue from Test Case 1
2. Type: "What are its key features?"
3. Click Send
4. **Expected**: Response considers previous context (session persistence)

**Test Case 3: Selected-Text Mode**
1. Navigate to a textbook page with content
2. Highlight text: "ROS (Robot Operating System)"
3. Open ChatWidget (should capture selection)
4. Type: "Explain this in more detail"
5. Click Send
6. **Expected**: Response references the selected text specifically

**Test Case 4: Error Handling**
1. Stop backend server (Ctrl+C in backend terminal)
2. Try sending a message in ChatWidget
3. **Expected**: User-friendly error message displayed (e.g., "Unable to reach server. Please try again.")

**Test Case 5: Session Persistence**
1. Send a message in ChatWidget
2. Refresh the page (F5)
3. Open ChatWidget
4. **Expected**: Previous conversation history is restored from localStorage and backend

**Troubleshooting**:
- **Widget not visible** → Check Root.jsx integration, inspect console for errors
- **"Network Error"** → Verify backend is running on port 8000, check CORS configuration
- **"CORS policy error"** → Add `http://localhost:3000` to CORS_ORIGINS in backend/api.py
- **Selected text not captured** → Check browser permissions, ensure `window.getSelection()` is supported
- **Session not persisting** → Check localStorage in browser DevTools (Application tab)

---

## Part 3: End-to-End Testing

### Test Suite: 5 Required Queries

Run all 5 test cases and verify results:

| Test # | Query | Selected Text | Expected Outcome | Status |
|--------|-------|---------------|------------------|--------|
| 1 | "What is Gazebo simulation?" | None | Response mentions "3D robotics simulator" | ☐ |
| 2 | "What are the key features of ROS?" | None | Response lists ROS features (nodes, topics, services) | ☐ |
| 3 | "Explain this concept" | "Inverse kinematics" | Response explains inverse kinematics specifically | ☐ |
| 4 | "How does it relate to forward kinematics?" | Same session as #3 | Response maintains context from previous query | ☐ |
| 5 | "What sensors are used in humanoid robots?" | None | Response lists sensor types (IMU, cameras, force sensors) | ☐ |

**Acceptance Criteria**:
- [x] All 5 queries receive responses within 5 seconds (95th percentile)
- [x] Selected-text mode (Test #3) overrides full retrieval and references specific text
- [x] Multi-turn conversation (Test #4) maintains session context
- [x] Session persists across page refreshes
- [x] No errors in browser console or backend logs

### Performance Benchmarking

Measure response times for 10 consecutive queries:

```bash
# Run benchmark script (if available)
python tests/benchmark_api.py --queries 10 --endpoint http://localhost:8000/api/chat

# Manual testing: Record response_time_ms from each API response
```

**Target Metrics**:
- p50 latency: < 2 seconds
- p95 latency: < 5 seconds
- p99 latency: < 8 seconds

**Example Results**:
```
Query 1: 2341.5 ms
Query 2: 1893.2 ms
Query 3: 2156.7 ms
...
Query 10: 2421.1 ms

p50: 2.15s ✓
p95: 4.32s ✓
p99: 5.12s ✓
```

---

## Part 4: Deployment Preparation

### Backend Deployment (Hugging Face Spaces)

**Step 1: Create Dockerfile** (in `backend/`)

```dockerfile
FROM python:3.10-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8000

CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Step 2: Test Dockerfile Locally**

```bash
# Build image
docker build -t rag-chatbot-backend .

# Run container
docker run -p 8000:8000 --env-file .env rag-chatbot-backend

# Test endpoint
curl http://localhost:8000/health
```

**Step 3: Deploy to Hugging Face Spaces**

1. Create new Space at https://huggingface.co/spaces
2. Select "Docker" as Space type
3. Push code to Space repository:
   ```bash
   git remote add hf https://huggingface.co/spaces/<username>/<space-name>
   git push hf main
   ```
4. Configure Secrets in Space settings (Environment Variables):
   - `NEON_DATABASE_URL`
   - `GROQ_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `FRONTEND_URL=https://your-site.vercel.app`

**Expected Outcome**: Space builds and runs successfully, accessible at `https://<username>-<space-name>.hf.space`

### Frontend Deployment (Vercel)

**Step 1: Update Environment Variables in Vercel**

1. Go to Vercel project settings
2. Add environment variable:
   - `REACT_APP_API_URL=https://<username>-<space-name>.hf.space`

**Step 2: Deploy**

```bash
# Push to main branch (triggers automatic deployment)
git push origin main

# Or deploy manually
vercel --prod
```

**Step 3: Verify Deployment**

1. Visit deployed site: `https://your-site.vercel.app`
2. Test ChatWidget end-to-end (all 5 test cases from Part 3)
3. Verify CORS is configured correctly (no console errors)

**Expected Outcome**: All test cases pass on production deployment.

---

## Part 5: Validation & Acceptance

### Acceptance Checklist

Run through this final checklist before marking the feature complete:

#### Functional Requirements
- [ ] **FR-001**: POST /api/chat endpoint accepts valid requests and returns responses
- [ ] **FR-002**: Database stores sessions with UUID, timestamps, and metadata
- [ ] **FR-003**: Database stores messages with role, content, timestamps
- [ ] **FR-004**: GET /api/sessions/{id} retrieves conversation history
- [ ] **FR-005**: DELETE /api/sessions/{id} soft-deletes sessions
- [ ] **FR-006**: ChatWidget renders in Docusaurus frontend
- [ ] **FR-007**: ChatWidget sends queries to backend and displays responses
- [ ] **FR-008**: ChatWidget captures selected text with `window.getSelection()`
- [ ] **FR-009**: Selected-text mode overrides full retrieval in backend
- [ ] **FR-010**: Session IDs persist in localStorage across page refreshes
- [ ] **FR-011**: CORS allows requests from localhost and production domains
- [ ] **FR-012**: Pydantic validates all request fields (query length, UUID format)
- [ ] **FR-013**: Error responses include human-readable messages
- [ ] **FR-014**: ChatWidget responsive on mobile (375px+ width)
- [ ] **FR-015**: Error boundaries catch and display errors gracefully
- [ ] **FR-016**: Agent reuses existing retrieval and embedding logic
- [ ] **FR-017**: Health check endpoints return 200 OK
- [ ] **FR-018**: Database indexes improve query performance
- [ ] **FR-019**: Connection pooling handles 50+ concurrent users
- [ ] **FR-020**: Environment variables loaded from .env files

#### Success Criteria
- [ ] **SC-001**: 95% of queries respond in < 5 seconds
- [ ] **SC-002**: Sessions persist for 24+ hours
- [ ] **SC-003**: 5+ end-to-end test queries pass
- [ ] **SC-004**: Selected-text mode accuracy ≥ 90%
- [ ] **SC-005**: ChatWidget loads in < 2 seconds
- [ ] **SC-006**: Backend handles 50 concurrent users without errors
- [ ] **SC-007**: Frontend deployed to Vercel, backend to Hugging Face
- [ ] **SC-008**: Errors display user-friendly messages
- [ ] **SC-009**: Responsive design works on 375px+ screens
- [ ] **SC-010**: All modules (agent, retrieval, API, frontend) covered by tests

#### Edge Cases
- [ ] **EDGE-001**: Empty query returns 400 error
- [ ] **EDGE-002**: Query > 2000 chars returns 400 error
- [ ] **EDGE-003**: Invalid session_id returns 404 error
- [ ] **EDGE-004**: Database connection failure returns 503 error
- [ ] **EDGE-005**: LLM API timeout handled gracefully
- [ ] **EDGE-006**: Network failure displays retry option
- [ ] **EDGE-007**: Selected text > 5000 chars truncated automatically
- [ ] **EDGE-008**: localStorage disabled shows warning message
- [ ] **EDGE-009**: Mobile landscape orientation handled correctly
- [ ] **EDGE-010**: 50 concurrent users don't overwhelm database pool

---

## Common Issues & Solutions

### Issue: "Database connection failed"
**Symptoms**: Backend returns 503 error on /api/chat
**Solution**:
1. Check `NEON_DATABASE_URL` in backend/.env
2. Verify Neon database is running (check Neon dashboard)
3. Test connection manually: `psql $NEON_DATABASE_URL -c "SELECT 1"`

### Issue: "CORS policy error"
**Symptoms**: Browser console shows "blocked by CORS policy"
**Solution**:
1. Add frontend URL to CORS_ORIGINS in backend/api.py
2. Restart backend server
3. Clear browser cache (Ctrl+Shift+R)

### Issue: "Module 'agent' not found"
**Symptoms**: Backend crashes on startup
**Solution**:
1. Ensure all prior specs are implemented (agent.py, retrieve.py, embedding.py)
2. Verify virtual environment is activated
3. Check PYTHONPATH includes backend directory

### Issue: "ChatWidget not rendering"
**Symptoms**: Widget not visible on frontend
**Solution**:
1. Check src/theme/Root.jsx integration
2. Inspect browser console for errors
3. Verify @openai/chatkit-react is installed (check package.json)

### Issue: "Session not persisting"
**Symptoms**: Conversation history lost on page refresh
**Solution**:
1. Check browser localStorage in DevTools (Application tab)
2. Verify session_id is saved correctly
3. Ensure backend returns same session_id in responses

---

## Next Steps

After completing this quickstart:

1. **Run Full Test Suite**: Execute all unit, integration, and E2E tests
2. **Performance Optimization**: Profile slow queries and optimize database indexes
3. **Security Audit**: Review CORS, input validation, and secret management
4. **Documentation**: Update README.md with deployment instructions
5. **User Feedback**: Share with hackathon participants and gather feedback

---

## Resources

- **Backend API Documentation**: See `contracts/openapi.yaml` for full API specification
- **Database Schema**: See `data-model.md` for schema details
- **Architecture Overview**: See `research.md` for architectural decisions
- **Feature Specification**: See `spec.md` for user stories and requirements

---

## Support

For issues or questions:
- **Backend Issues**: Check backend logs with `tail -f backend.log`
- **Frontend Issues**: Check browser console (F12)
- **Database Issues**: Check Neon dashboard for connection stats
- **Deployment Issues**: Check Vercel/Hugging Face build logs

**Estimated Completion Time**: 30 minutes (first-time setup)
**Re-run Time**: 5 minutes (after initial setup)
