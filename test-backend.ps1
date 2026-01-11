# Quick Backend Test Script

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Testing Backend API" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Test 1: Health Check
Write-Host "[1/3] Testing health endpoint..." -ForegroundColor Yellow
try {
    $health = Invoke-WebRequest -Uri 'http://localhost:8000/health' -UseBasicParsing
    Write-Host "✓ Health check: " -ForegroundColor Green -NoNewline
    Write-Host $health.Content
} catch {
    Write-Host "✗ Health check failed: $_" -ForegroundColor Red
    exit 1
}

Write-Host ""

# Test 2: Root Endpoint
Write-Host "[2/3] Testing root endpoint..." -ForegroundColor Yellow
try {
    $root = Invoke-WebRequest -Uri 'http://localhost:8000/' -UseBasicParsing
    Write-Host "✓ Root endpoint: " -ForegroundColor Green -NoNewline
    Write-Host $root.Content
} catch {
    Write-Host "✗ Root endpoint failed: $_" -ForegroundColor Red
    exit 1
}

Write-Host ""

# Test 3: Chat Endpoint
Write-Host "[3/3] Testing chat endpoint (this may take a few seconds)..." -ForegroundColor Yellow
try {
    $chatBody = @{
        message = "What is ROS2?"
    } | ConvertTo-Json

    Write-Host "Sending query: 'What is ROS2?'" -ForegroundColor Gray

    $chatResponse = Invoke-WebRequest `
        -Uri 'http://localhost:8000/api/chat' `
        -Method POST `
        -ContentType 'application/json' `
        -Body $chatBody `
        -UseBasicParsing

    $jsonResponse = $chatResponse.Content | ConvertFrom-Json

    Write-Host "SUCCESS: Chat endpoint working!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Response preview:" -ForegroundColor Cyan
    Write-Host "  Session ID: $($jsonResponse.session_id)" -ForegroundColor White
    Write-Host "  Response time: $([math]::Round($jsonResponse.response_time_ms, 2))ms" -ForegroundColor White

    $previewLength = [Math]::Min(150, $jsonResponse.response.Length)
    Write-Host "  Answer: $($jsonResponse.response.Substring(0, $previewLength))..." -ForegroundColor White

    if ($jsonResponse.sources) {
        Write-Host "  Sources: $($jsonResponse.sources.Count) documents retrieved" -ForegroundColor White
    }

} catch {
    Write-Host "FAILED: Chat endpoint error" -ForegroundColor Red
    Write-Host "Error details: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  All backend tests passed! ✓" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "API Documentation: http://localhost:8000/docs" -ForegroundColor Cyan
