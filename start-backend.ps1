# Start Backend Server Script
# This script starts the FastAPI backend using the virtual environment directly

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Humanoid Robotics RAG Backend" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Using Python from venv..." -ForegroundColor Green

# Show Python version
$pythonVersion = & .\backend\venv\Scripts\python.exe --version
Write-Host "  $pythonVersion" -ForegroundColor Yellow

Write-Host ""
Write-Host "Starting FastAPI backend..." -ForegroundColor Green
Write-Host "  URL: http://localhost:8000" -ForegroundColor Yellow
Write-Host "  Docs: http://localhost:8000/docs" -ForegroundColor Yellow
Write-Host ""
Write-Host "Press Ctrl+C to stop the server" -ForegroundColor Gray
Write-Host ""

# Start uvicorn directly from venv (no activation needed)
& .\backend\venv\Scripts\uvicorn.exe backend.api:app --reload --host 0.0.0.0 --port 8000
