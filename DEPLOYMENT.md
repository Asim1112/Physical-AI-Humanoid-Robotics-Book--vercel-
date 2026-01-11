# Deployment Guide

Comprehensive guide for deploying the Humanoid Robotics Textbook with RAG chatbot integration.

## Table of Contents

1. [Environment Variables](#environment-variables)
2. [Local Development](#local-development)
3. [Production Deployment](#production-deployment)
4. [Testing](#testing)
5. [Troubleshooting](#troubleshooting)

---

## Environment Variables

### Backend Environment Variables

Create a `.env` file in the `backend/` directory with the following variables:

```env
# Database Configuration
NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# LLM API Configuration (choose one)
GROQ_API_KEY=your-groq-api-key-here
# OR
OPENAI_API_KEY=your-openai-api-key-here

# Embedding API Configuration
COHERE_API_KEY=your-cohere-api-key-here

# Vector Database Configuration
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=humanoid-robotics-textbook

# Frontend Configuration
FRONTEND_URL=https://your-site.vercel.app

# Optional: API Rate Limiting
MAX_REQUESTS_PER_MINUTE=60
```

#### Environment Variable Details

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `NEON_DATABASE_URL` | Yes | PostgreSQL connection string from Neon Serverless | `postgresql://user:pass@host.neon.tech/db` |
| `GROQ_API_KEY` | Yes* | Groq API key for LLM inference (fast, cost-effective) | `gsk_...` |
| `OPENAI_API_KEY` | Yes* | Alternative to Groq for LLM inference | `sk-...` |
| `COHERE_API_KEY` | Yes | Cohere API key for embedding generation | `...` |
| `QDRANT_URL` | Yes | Qdrant Cloud instance URL | `https://xxx.cloud.qdrant.io` |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key | `...` |
| `QDRANT_COLLECTION_NAME` | Yes | Collection name in Qdrant (must match ingestion pipeline) | `humanoid-robotics-textbook` |
| `FRONTEND_URL` | Yes | Frontend URL for CORS configuration | `https://your-site.vercel.app` |

\* Either `GROQ_API_KEY` or `OPENAI_API_KEY` is required, not both.

### Frontend Environment Variables

Create a `.env` file in the `frontend/` directory:

```env
REACT_APP_API_URL=http://localhost:8000
```

For production, update to your backend URL:
```env
REACT_APP_API_URL=https://your-backend.hf.space
```

---

## Local Development

### Prerequisites

- Python 3.10+
- Node.js 16+
- PostgreSQL database (Neon Serverless recommended)
- API keys for all required services

### Backend Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Create virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment**:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys
   ```

5. **Initialize database** (first time only):
   ```bash
   python -c "from db import init_database; import asyncio; asyncio.run(init_database())"
   ```

6. **Start backend server**:
   ```bash
   uvicorn api:app --reload --host 0.0.0.0 --port 8000
   ```

   The API will be available at:
   - http://localhost:8000 (root endpoint)
   - http://localhost:8000/docs (Swagger UI)
   - http://localhost:8000/redoc (ReDoc documentation)

### Frontend Setup

1. **Navigate to frontend directory**:
   ```bash
   cd frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Configure environment**:
   ```bash
   cp .env.example .env
   # Ensure REACT_APP_API_URL=http://localhost:8000
   ```

4. **Start development server**:
   ```bash
   npm start
   # or
   yarn start
   ```

   The frontend will be available at http://localhost:3000

### Verify Local Setup

1. Open http://localhost:3000 in your browser
2. Click the chat widget button (💬)
3. Ask a test question: "What is Gazebo simulation?"
4. Verify response appears within 5 seconds

---

## Production Deployment

### Option 1: Hugging Face Spaces (Backend) + Vercel (Frontend)

This is the recommended production setup for cost-effectiveness and scalability.

#### Backend Deployment (Hugging Face Spaces)

1. **Create Hugging Face Space**:
   - Go to https://huggingface.co/spaces
   - Click "Create new Space"
   - Name: `rag-backend` (or your preferred name)
   - SDK: Docker
   - Hardware: CPU Basic (free tier)
   - Visibility: Public or Private

2. **Push code to Space repository**:
   ```bash
   cd backend
   git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/rag-backend
   git add .
   git commit -m "Deploy backend to Hugging Face"
   git push hf main
   ```

3. **Configure environment variables in Hugging Face**:
   - Go to Space Settings → Repository Secrets
   - Add all backend environment variables:
     - `NEON_DATABASE_URL`
     - `GROQ_API_KEY` or `OPENAI_API_KEY`
     - `COHERE_API_KEY`
     - `QDRANT_URL`
     - `QDRANT_API_KEY`
     - `QDRANT_COLLECTION_NAME`
     - `FRONTEND_URL` (will be your Vercel URL)

4. **Wait for build to complete** (usually 2-5 minutes)

5. **Test deployment**:
   ```bash
   curl https://YOUR_USERNAME-rag-backend.hf.space/health
   ```

   Expected response:
   ```json
   {"status": "ok", "timestamp": 1234567890.123}
   ```

#### Frontend Deployment (Vercel)

1. **Update environment variables**:
   - Edit `frontend/.env`:
     ```env
     REACT_APP_API_URL=https://YOUR_USERNAME-rag-backend.hf.space
     ```

2. **Deploy to Vercel** (if not already connected):
   ```bash
   cd frontend
   vercel
   ```

   Follow the prompts:
   - Link to existing project or create new
   - Root directory: `./frontend`
   - Build command: `npm run build`
   - Output directory: `build`

3. **Configure environment variables in Vercel**:
   - Go to Vercel Dashboard → Project Settings → Environment Variables
   - Add: `REACT_APP_API_URL` = `https://YOUR_USERNAME-rag-backend.hf.space`

4. **Trigger deployment**:
   ```bash
   git add .
   git commit -m "Update API URL for production"
   git push origin main
   ```

   Vercel will auto-deploy on push to main branch.

5. **Update CORS in backend**:
   - Update `FRONTEND_URL` in Hugging Face Space secrets to your Vercel URL
   - Restart the Space to apply changes

#### Verify Production Deployment

1. Open your Vercel URL (e.g., https://your-site.vercel.app)
2. Open browser DevTools (F12) → Console
3. Verify no CORS errors
4. Click chat widget and test a query
5. Verify response time <5 seconds

### Option 2: Docker Compose (Self-Hosted)

For self-hosted deployments on your own infrastructure:

1. **Create `docker-compose.yml` in project root**:
   ```yaml
   version: '3.8'
   services:
     backend:
       build: ./backend
       ports:
         - "8000:8000"
       env_file:
         - ./backend/.env
       restart: unless-stopped

     frontend:
       build: ./frontend
       ports:
         - "3000:3000"
       environment:
         - REACT_APP_API_URL=http://localhost:8000
       depends_on:
         - backend
       restart: unless-stopped
   ```

2. **Build and run**:
   ```bash
   docker-compose up -d
   ```

3. **Access**:
   - Frontend: http://localhost:3000
   - Backend: http://localhost:8000

---

## Testing

### Manual Testing Checklist

#### User Story 1: Basic Chat Functionality

- [ ] Open chat widget by clicking 💬 button
- [ ] Ask: "What is Gazebo simulation?"
- [ ] Verify response mentions 3D robotics simulator
- [ ] Verify response time <5 seconds
- [ ] Verify message appears in chat history

#### User Story 2: Session Persistence

- [ ] Ask: "What is ROS 2?"
- [ ] Navigate to different page
- [ ] Verify chat history persists
- [ ] Ask: "How do I install it?"
- [ ] Verify agent understands "it" refers to ROS 2
- [ ] Close browser and reopen (within 24 hours)
- [ ] Verify conversation history restored
- [ ] Click trash icon (🗑️) to clear chat
- [ ] Verify history is cleared

#### User Story 3: Selected-Text Mode

- [ ] Highlight paragraph about "Gazebo physics simulation"
- [ ] Open chat widget
- [ ] Verify selected text indicator appears (📎 Selected text: ...)
- [ ] Ask: "Explain this in simpler terms"
- [ ] Verify response references the selected text
- [ ] Verify selection clears after sending
- [ ] Highlight text >5000 characters
- [ ] Verify automatic truncation to 5000 chars

#### Edge Cases

- [ ] Empty query → Verify 400 error with clear message
- [ ] Query >2000 characters → Verify validation error
- [ ] Backend down → Verify user-friendly error message
- [ ] Network timeout → Verify timeout handling

### Automated Testing

Run backend tests:
```bash
cd backend
pytest tests/ -v
```

Run frontend tests (if available):
```bash
cd frontend
npm test
```

### Performance Benchmarking

Test response times with 10 queries:
```bash
cd backend
python -c "
import time
import requests

queries = ['What is Gazebo?'] * 10
times = []

for q in queries:
    start = time.time()
    r = requests.post('http://localhost:8000/api/chat', json={'query': q})
    times.append((time.time() - start) * 1000)

print(f'p50: {sorted(times)[5]:.0f}ms')
print(f'p95: {sorted(times)[9]:.0f}ms')
print(f'p99: {sorted(times)[9]:.0f}ms')
"
```

Target: p95 <5000ms

---

## Troubleshooting

### Common Issues

#### 1. CORS Errors in Browser Console

**Symptom**:
```
Access to fetch at 'http://localhost:8000/api/chat' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Solution**:
- Verify `FRONTEND_URL` in backend `.env` matches your frontend URL
- Restart backend server after changing environment variables
- Check CORS configuration in `backend/api.py`:
  ```python
  CORS_ORIGINS = [
      "http://localhost:3000",
      os.getenv("FRONTEND_URL")
  ]
  ```

#### 2. Database Connection Failed

**Symptom**:
```
asyncpg.exceptions.InvalidPasswordError: password authentication failed
```

**Solution**:
- Verify `NEON_DATABASE_URL` is correct and includes `?sslmode=require`
- Check database user has proper permissions
- Verify database exists and is accessible
- Test connection:
  ```bash
  psql "$NEON_DATABASE_URL"
  ```

#### 3. Chat Widget Not Appearing

**Symptom**: Chat button (💬) not visible on page

**Solution**:
- Verify `frontend/src/theme/Root.jsx` exists and imports ChatWidget
- Check browser console for errors
- Verify ChatKit dependencies installed:
  ```bash
  npm list @openai/chatkit-react
  ```
- Clear browser cache and hard refresh (Ctrl+Shift+R)

#### 4. Session Not Persisting

**Symptom**: Conversation history lost on page navigation

**Solution**:
- Check browser localStorage:
  ```javascript
  localStorage.getItem('chatSessionId')
  ```
- Verify session ID is being returned from `/api/chat`
- Check backend logs for database errors
- Verify session not archived:
  ```sql
  SELECT * FROM chat_sessions WHERE archived = FALSE;
  ```

#### 5. Selected Text Not Detected

**Symptom**: Selected text indicator not showing

**Solution**:
- Verify text is selected before opening chat widget
- Check `window.getSelection()` returns valid selection:
  ```javascript
  window.getSelection().toString()
  ```
- Verify `getSelectedText()` function in ChatWidget component
- Check browser console for JavaScript errors

#### 6. Slow Response Times (>5 seconds)

**Symptom**: Queries take longer than 5 seconds to respond

**Solution**:
- Check Qdrant response time in backend logs
- Verify LLM API (Groq/OpenAI) response time
- Consider upgrading to paid tiers for faster APIs
- Optimize Qdrant collection (ensure HNSW index built)
- Check network latency to external APIs

#### 7. Docker Build Fails

**Symptom**:
```
ERROR: failed to solve: process "/bin/sh -c pip install -r requirements.txt" did not complete successfully
```

**Solution**:
- Verify `requirements.txt` is valid
- Check Python version in Dockerfile matches your development version
- Try building with `--no-cache`:
  ```bash
  docker build --no-cache -t rag-backend .
  ```
- Check for pip dependency conflicts

#### 8. Hugging Face Space Build Failing

**Symptom**: Space shows "Build failed" status

**Solution**:
- Check Space logs for error messages
- Verify Dockerfile is in root of backend directory
- Verify all dependencies in `requirements.txt` are compatible
- Check that environment variables are set in Space secrets
- Ensure port 8000 is exposed in Dockerfile

#### 9. Vercel Deployment Failing

**Symptom**: Vercel build fails with error

**Solution**:
- Check Vercel build logs for specific error
- Verify build command is correct: `npm run build`
- Verify output directory is `build` (or `dist` for Vite)
- Check `REACT_APP_API_URL` is set in Vercel environment variables
- Clear Vercel build cache and retry

#### 10. API Returns 500 Internal Server Error

**Symptom**: All queries return 500 error

**Solution**:
- Check backend logs for stack trace
- Verify all environment variables are set
- Test API health endpoint:
  ```bash
  curl http://localhost:8000/health
  ```
- Check database is initialized:
  ```bash
  python -c "from db import init_database; import asyncio; asyncio.run(init_database())"
  ```
- Verify agent.py has `run_agent_query_async` function

### Getting Help

If you encounter issues not covered here:

1. Check backend logs in `backend/logs/`
2. Check browser console (F12) for frontend errors
3. Review the [specification](./specs/003-frontend-backend-integration/spec.md)
4. Search existing GitHub issues
5. Open a new issue with:
   - Exact error message
   - Steps to reproduce
   - Environment details (OS, Python version, Node version)
   - Relevant log excerpts

---

## Production Checklist

Before going live, verify:

### Security
- [ ] All API keys in environment variables (not hardcoded)
- [ ] `.env` files in `.gitignore`
- [ ] CORS configured with specific origins (not `*`)
- [ ] Database uses SSL connections (`?sslmode=require`)
- [ ] Input validation on all API endpoints
- [ ] Rate limiting configured (optional)

### Performance
- [ ] Response times <5 seconds for test queries
- [ ] Database connection pooling enabled
- [ ] Qdrant HNSW index built and optimized
- [ ] Frontend static assets cached

### Monitoring
- [ ] Health check endpoints responding
- [ ] Error logging configured
- [ ] Database connection monitoring
- [ ] API response time monitoring

### Documentation
- [ ] README.md updated with deployment URL
- [ ] Environment variables documented
- [ ] Troubleshooting guide reviewed
- [ ] User testing completed

---

## Maintenance

### Regular Tasks

- **Weekly**: Check error logs for issues
- **Monthly**: Review database session cleanup (archived sessions >24h old)
- **Quarterly**: Update dependencies and security patches
- **As needed**: Update textbook content and re-run ingestion pipeline

### Updating Textbook Content

When textbook content changes:

1. Update markdown files in `frontend/docs/`
2. Re-run ingestion pipeline:
   ```bash
   cd backend
   uv run python main.py
   ```
3. Verify new content in Qdrant:
   ```bash
   uv run python main.py --retrieval-only
   ```
4. Deploy frontend updates to Vercel

### Scaling Considerations

As usage grows, consider:

- Upgrading Neon database tier for more connections
- Upgrading Hugging Face Space to GPU for faster LLM inference
- Implementing Redis caching for frequently asked questions
- Adding CDN for frontend static assets
- Implementing rate limiting and API key authentication

---

For more information, see:
- [README.md](./README.md) - Quick start guide
- [Specification](./specs/003-frontend-backend-integration/spec.md) - Detailed requirements
- [Implementation Plan](./specs/003-frontend-backend-integration/plan.md) - Architecture decisions
