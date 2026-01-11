# Quick Start Guide

## Starting the Backend (3 Methods)

### ⭐ Method 1: Use the Startup Script (EASIEST)

```powershell
cd F:\re-trying\Book3\Humanoid-Robotics-Book
.\start-backend.ps1
```

This is the **recommended** way to start the backend.

---

### Method 2: Direct Path (NO ACTIVATION NEEDED)

```powershell
cd F:\re-trying\Book3\Humanoid-Robotics-Book
.\backend\venv\Scripts\uvicorn.exe backend.api:app --reload --host 0.0.0.0 --port 8000
```

This method:
- ✅ Works without activating the venv
- ✅ Uses the correct Python environment automatically
- ✅ No `(venv)` prefix needed

---

### Method 3: Traditional Activation (If You Prefer)

```powershell
cd F:\Humanoid-Robotics-Hackathon
.\backend\venv\Scripts\Activate.ps1
uvicorn backend.api:app --reload --host 0.0.0.0 --port 8000
```

**Note**: You should see `(venv)` in your prompt after activation. If you don't, the activation still works—just use Method 2 instead.

---

## Testing the Backend

Once the server starts, open a **new PowerShell window** and test:

```powershell
# Test health endpoint
curl http://localhost:8000/health

# Or in PowerShell
Invoke-WebRequest -Uri http://localhost:8000/health | Select-Object -ExpandProperty Content
```

**Expected response:**
```json
{"status":"ok","timestamp":1234567890.123}
```

---

## Viewing API Documentation

Open your browser:
- **Interactive Docs (Swagger)**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc
- **Health Check**: http://localhost:8000/health

---

## Troubleshooting

### "ModuleNotFoundError: No module named 'fastapi'"

**Problem**: You're using system Python instead of the venv.

**Solution**: Use Method 1 or Method 2 (direct path).

### "Cannot find uvicorn"

**Problem**: uvicorn is not in your system PATH.

**Solution**: Use the full path: `.\backend\venv\Scripts\uvicorn.exe`

### Server won't start / Port already in use

**Problem**: Another process is using port 8000.

**Solution**:
```powershell
# Find what's using port 8000
netstat -ano | findstr :8000

# Kill the process (replace PID with actual process ID)
taskkill /PID <PID> /F

# Or use a different port
.\backend\venv\Scripts\uvicorn.exe backend.api:app --reload --port 8001
```

---

## Environment Variables

Before running the backend for the first time, ensure you have a `.env` file in `backend/` with:

```env
# Required
NEON_DATABASE_URL=postgresql://...
GROQ_API_KEY=your-groq-key
COHERE_API_KEY=your-cohere-key
QDRANT_URL=https://...
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION_NAME=humanoid-robotics-textbook
FRONTEND_URL=http://localhost:3000
```

See `backend/.env.example` for a template.

---

## Next Steps

1. ✅ Start backend with `.\start-backend.ps1`
2. ✅ Test with `curl http://localhost:8000/health`
3. ✅ View docs at http://localhost:8000/docs
4. ➡️ Start frontend (see frontend/README.md)
5. ➡️ Test chat widget end-to-end
