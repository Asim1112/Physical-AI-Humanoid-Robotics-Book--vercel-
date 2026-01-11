# Quick Start Guide

## ✅ Backend is Already Running!

Your backend is currently running at: **http://localhost:8000**

---

## 🚀 Test Backend (2 minutes)

### Option 1: Use PowerShell Test Script
```powershell
.\test-backend.ps1
```

### Option 2: Manual Tests

**1. Health Check:**
```powershell
Invoke-WebRequest -Uri 'http://localhost:8000/health' -UseBasicParsing
```

**2. Visit API Documentation:**
Open in browser: http://localhost:8000/docs

**3. Test Chat Endpoint:**
- Go to http://localhost:8000/docs
- Find `/api/chat` endpoint
- Click "Try it out"
- Enter JSON:
  ```json
  {
    "message": "What is ROS2?"
  }
  ```
- Click "Execute"
- You should get a response with answer and sources!

---

## 🎨 Start Frontend (1 minute)

### Open a NEW PowerShell terminal and run:

```powershell
cd frontend
npm start
```

### What to expect:
- Build process takes 30-60 seconds
- Browser automatically opens to http://localhost:3000
- You'll see the Humanoid Robotics Book homepage

---

## 💬 Test the Chat

1. **Navigate to the chat page** (wherever it's located in your frontend)

2. **Type a question**, for example:
   - "What is ROS2?"
   - "How do I set up Gazebo?"
   - "Explain VLA models"

3. **Check that:**
   - ✅ Your message appears
   - ✅ Backend responds (may take 2-5 seconds)
   - ✅ Answer is relevant
   - ✅ Sources are shown (if implemented)

---

## 🐛 If Something Goes Wrong

### Backend not responding?
```powershell
# Check if backend is running:
Invoke-WebRequest -Uri 'http://localhost:8000/health'

# If not running, start it:
.\start-backend.ps1
```

### Frontend won't start?
```powershell
# Make sure you're in the frontend directory:
cd frontend

# Try clearing cache:
npm run clear

# Then start again:
npm start
```

### Chat not working?
1. Check browser console (F12 → Console tab)
2. Check Network tab (F12 → Network tab)
3. Look for failed requests to http://localhost:8000

---

## 📋 Complete Testing Checklist

For comprehensive testing instructions, see: **[TESTING_GUIDE.md](./TESTING_GUIDE.md)**

---

## 🎯 Current Status

✅ **Backend**: Running at http://localhost:8000
⏳ **Frontend**: Ready to start (run `cd frontend && npm start`)
⏳ **Integration**: Ready to test once frontend is running

---

## 📝 Quick Commands Reference

```powershell
# Backend
.\start-backend.ps1                    # Start backend
.\test-backend.ps1                     # Test backend

# Frontend
cd frontend                            # Go to frontend directory
npm start                              # Start dev server
npm run build                          # Build for production

# Open in Browser
start http://localhost:8000/docs       # Backend API docs
start http://localhost:3000            # Frontend
```

---

**🎉 Ready to test! Start with the frontend: `cd frontend && npm start`**
