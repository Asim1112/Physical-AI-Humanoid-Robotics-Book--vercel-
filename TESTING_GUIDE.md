# Testing Guide - Backend & Frontend

This guide provides step-by-step instructions for testing both the backend and frontend of the Humanoid Robotics RAG Chatbot.

---

## 🎯 Quick Start

### Backend (Already Running)
Your backend is currently running at: **http://localhost:8000**

### Start Frontend
```powershell
cd frontend
npm start
```

The frontend will start at: **http://localhost:3000**

---

## 🔧 Backend Testing

### 1. Health Check
Test if the backend is running:

```powershell
# Using PowerShell
Invoke-WebRequest -Uri 'http://localhost:8000/health' -UseBasicParsing | Select-Object -ExpandProperty Content

# Expected response:
# {"status":"ok","timestamp":1234567890.123}
```

### 2. API Documentation
Open your browser and visit:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

This shows all available API endpoints with interactive testing capabilities.

### 3. Test Chat Endpoint

Create a test file `test-chat.ps1`:

```powershell
# test-chat.ps1
$body = @{
    message = "What is ROS2?"
    session_id = $null
} | ConvertTo-Json

$response = Invoke-WebRequest `
    -Uri 'http://localhost:8000/api/chat' `
    -Method POST `
    -ContentType 'application/json' `
    -Body $body `
    -UseBasicParsing

$response.Content | ConvertFrom-Json | ConvertTo-Json -Depth 10
```

Run it:
```powershell
.\test-chat.ps1
```

**Expected Response Structure:**
```json
{
  "response": "ROS2 is...",
  "session_id": "uuid-here",
  "sources": [
    {
      "source_file": "module-1-ros2/ros2-basics.txt",
      "similarity_score": 0.85,
      "content_preview": "..."
    }
  ],
  "response_time_ms": 1234.56
}
```

### 4. Test Session Management

**Create a new session:**
```powershell
Invoke-WebRequest -Uri 'http://localhost:8000/api/sessions' -Method POST -UseBasicParsing
```

**Get session history:**
```powershell
# Replace <session-id> with actual session ID
Invoke-WebRequest -Uri 'http://localhost:8000/api/sessions/<session-id>' -UseBasicParsing | Select-Object -ExpandProperty Content
```

**List all sessions:**
```powershell
Invoke-WebRequest -Uri 'http://localhost:8000/api/sessions' -UseBasicParsing | Select-Object -ExpandProperty Content
```

### 5. Test Database Connection

Check the backend console output. You should see:
```
INFO:     Application startup complete.
```

If you see database errors, check:
1. `.env` file has correct `NEON_DATABASE_URL`
2. Neon database is accessible (check Neon dashboard)

### 6. Test RAG Pipeline Components

**Test Qdrant Connection:**
```powershell
# In Python (backend venv)
.\backend\venv\Scripts\python.exe -c "
from backend.storage import create_qdrant_client
client = create_qdrant_client()
print('Qdrant connected successfully')
"
```

**Test Cohere Embeddings:**
```powershell
.\backend\venv\Scripts\python.exe -c "
from backend.embedding import generate_embedding
embedding = generate_embedding('test query')
print(f'Embedding dimension: {len(embedding)}')
"
```

**Test Retrieval:**
```powershell
.\backend\venv\Scripts\python.exe -c "
from backend.retrieve import retrieve_content
from backend.storage import create_qdrant_client

client = create_qdrant_client()
result = retrieve_content('What is ROS2?', client, 'humanoid-robotics-textbook')
print(f'Retrieved {len(result.retrieved_chunks)} chunks')
for chunk in result.retrieved_chunks:
    print(f'- {chunk.source_file} (score: {chunk.similarity_score:.2f})')
"
```

---

## 🎨 Frontend Testing

### 1. Start Frontend Development Server

```powershell
cd frontend
npm start
```

The server will start at http://localhost:3000 and automatically open in your browser.

**Expected Console Output:**
```
[SUCCESS] Compiled successfully!

You can now view humanoid-robotics-book in the browser.

  Local:            http://localhost:3000/
  On Your Network:  http://192.168.x.x:3000/
```

### 2. Test Homepage

Visit: http://localhost:3000

**What to check:**
- ✅ Page loads without errors
- ✅ Navigation menu appears
- ✅ Content is visible
- ✅ No console errors in browser DevTools (F12)

### 3. Test Chat Interface

Visit: http://localhost:3000/chat (or wherever the chat page is located)

**What to test:**

1. **Chat Input Field**
   - Type a question: "What is ROS2?"
   - Submit the message
   - Check that the message appears in the chat

2. **Backend Integration**
   - Open Browser DevTools (F12) → Network tab
   - Send a message
   - Look for POST request to `http://localhost:8000/api/chat`
   - Check if request status is `200 OK`
   - Verify response contains `response`, `session_id`, `sources`

3. **Response Display**
   - Verify the AI response appears
   - Check that sources are displayed (if UI supports it)
   - Test response time is reasonable (<5 seconds)

4. **Session Persistence**
   - Send multiple messages
   - Verify conversation context is maintained
   - Check that session_id remains the same

5. **Error Handling**
   - Stop the backend server (`Ctrl+C` in backend terminal)
   - Try sending a message
   - Verify frontend shows appropriate error message
   - Restart backend and test recovery

### 4. Test Documentation Pages

Visit different documentation sections:
- http://localhost:3000/docs/module-1-ros2/intro
- http://localhost:3000/docs/module-2-gazebo/intro
- etc.

**What to check:**
- ✅ Pages load correctly
- ✅ Images display properly
- ✅ Code blocks are syntax highlighted
- ✅ Links work correctly
- ✅ Search functionality works (if enabled)

### 5. Test Responsive Design

1. Open Browser DevTools (F12)
2. Toggle device toolbar (Ctrl+Shift+M)
3. Test different screen sizes:
   - Mobile (375px)
   - Tablet (768px)
   - Desktop (1920px)

**What to check:**
- ✅ Layout adapts to screen size
- ✅ Navigation menu collapses on mobile
- ✅ Chat interface is usable on all devices
- ✅ No horizontal scrolling

---

## 🔗 Integration Testing

### Test Full End-to-End Flow

1. **Start both servers:**
   - Backend: `.\start-backend.ps1` (already running)
   - Frontend: `cd frontend && npm start`

2. **Open frontend in browser:** http://localhost:3000

3. **Test complete user flow:**

   a. Navigate to chat page

   b. Ask a question about ROS2:
      - "What are the key components of ROS2?"

   c. Verify:
      - ✅ Question appears in chat
      - ✅ Backend processes request (check backend console logs)
      - ✅ Response appears in chat
      - ✅ Sources are shown (if UI supports)
      - ✅ Session is created/maintained

   d. Ask follow-up question:
      - "How do I install it?"

   e. Verify:
      - ✅ Context from previous message is maintained
      - ✅ Response is relevant to the conversation

   f. Test different topics:
      - Ask about Gazebo simulation
      - Ask about VLA models
      - Ask about humanoid robotics

   g. Check session persistence:
      - Refresh the page
      - Verify conversation history is restored (if implemented)

4. **Monitor backend logs:**
   - Watch the backend console for log messages
   - Look for successful database queries
   - Check for Qdrant retrieval logs
   - Verify no errors appear

5. **Monitor browser console:**
   - Open DevTools (F12) → Console
   - Look for any JavaScript errors
   - Check Network tab for API calls
   - Verify all requests return 200 status

---

## 🐛 Common Issues & Solutions

### Backend Issues

**Issue: "ModuleNotFoundError: No module named 'agents'"**
- **Solution**: OpenAI Agents SDK not installed. Run:
  ```powershell
  .\backend\venv\Scripts\python.exe -m pip install git+https://github.com/openai/openai-agents-python.git@main
  ```

**Issue: "OSError: Multiple exceptions: [Errno 10061] Connect call failed"**
- **Solution**: Database connection failed. Check:
  1. `.env` has correct `NEON_DATABASE_URL`
  2. Neon database is active (check Neon dashboard)
  3. No quotes or `psql` prefix in URL

**Issue: "COHERE_API_KEY not found"**
- **Solution**: Check `backend/.env` has `COHERE_API_KEY` set

**Issue: "QDRANT_URL not found"**
- **Solution**: Check `backend/.env` has `QDRANT_URL` and `QDRANT_API_KEY`

### Frontend Issues

**Issue: "Cannot GET /"**
- **Solution**: Frontend not started. Run:
  ```powershell
  cd frontend
  npm start
  ```

**Issue: "Network Error" when sending chat messages**
- **Solution**: Backend not running or CORS issue
  1. Verify backend is running: http://localhost:8000/health
  2. Check `backend/.env` has `FRONTEND_URL=http://localhost:3000`

**Issue: "npm ERR! missing script: start"**
- **Solution**: Wrong directory. Ensure you're in `frontend/` folder

**Issue: Frontend can't connect to backend**
- **Solution**: Check `frontend/.env` has:
  ```
  REACT_APP_API_URL=http://localhost:8000
  ```

---

## 📊 Performance Testing

### Backend Performance

```powershell
# Test response time for 10 queries
1..10 | ForEach-Object {
    $start = Get-Date
    $body = @{ message = "What is ROS2?"; session_id = $null } | ConvertTo-Json
    Invoke-WebRequest -Uri 'http://localhost:8000/api/chat' -Method POST -ContentType 'application/json' -Body $body -UseBasicParsing | Out-Null
    $elapsed = ((Get-Date) - $start).TotalMilliseconds
    Write-Output "Request $_: ${elapsed}ms"
}
```

**Expected Performance:**
- First request: 2000-5000ms (cold start)
- Subsequent requests: 500-2000ms
- Database queries: <100ms
- Qdrant retrieval: 100-500ms
- LLM generation: 1000-3000ms

### Frontend Performance

1. Open DevTools (F12) → Lighthouse
2. Click "Generate report"
3. Check scores:
   - Performance: >80
   - Accessibility: >90
   - Best Practices: >80
   - SEO: >80

---

## ✅ Test Checklist

### Backend
- [ ] Health endpoint returns 200 OK
- [ ] API docs accessible at /docs
- [ ] Chat endpoint processes queries
- [ ] Session creation works
- [ ] Session history retrieval works
- [ ] Database connection active
- [ ] Qdrant retrieval working
- [ ] Cohere embeddings working
- [ ] No errors in console logs

### Frontend
- [ ] Dev server starts successfully
- [ ] Homepage loads without errors
- [ ] Chat interface displays correctly
- [ ] Can send messages
- [ ] Receives responses from backend
- [ ] No console errors in browser
- [ ] Responsive design works
- [ ] Navigation functions properly

### Integration
- [ ] Frontend connects to backend
- [ ] Chat messages sent successfully
- [ ] Responses displayed correctly
- [ ] Session persistence works
- [ ] Sources displayed (if implemented)
- [ ] Error handling works
- [ ] Performance acceptable (<5s response)

---

## 🚀 Next Steps After Testing

1. **If everything works:**
   - Start using the chatbot
   - Test with various questions
   - Provide feedback on response quality

2. **If issues found:**
   - Check the troubleshooting section above
   - Review backend console logs
   - Review browser console logs
   - Check `.env` configuration files

3. **For production deployment:**
   - Build frontend: `cd frontend && npm run build`
   - Configure production database
   - Set up environment variables for production
   - Deploy to hosting platform (Vercel, etc.)

---

## 📝 Useful Commands Reference

### Backend
```powershell
# Start backend
.\start-backend.ps1

# Stop backend
# Press Ctrl+C in the terminal

# View backend logs
# Check the terminal where backend is running

# Test Python imports
.\backend\venv\Scripts\python.exe -c "from backend.agent import run_agent_query_async; print('OK')"
```

### Frontend
```powershell
# Start frontend
cd frontend
npm start

# Build for production
npm run build

# Serve production build
npm run serve

# Clear cache
npm run clear
```

### Database
```powershell
# Test database connection
.\backend\venv\Scripts\python.exe -c "
import asyncio
import os
from pathlib import Path
from dotenv import load_dotenv

backend_dir = Path('backend')
load_dotenv(dotenv_path=backend_dir / '.env')

print(f'Database URL: {os.getenv(\"NEON_DATABASE_URL\")[:50]}...')
"
```

---

**Happy Testing! 🎉**
