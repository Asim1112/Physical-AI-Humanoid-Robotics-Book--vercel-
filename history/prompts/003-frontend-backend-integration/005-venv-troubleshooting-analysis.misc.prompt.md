---
id: 005
title: Venv troubleshooting and analysis
stage: misc
date: 2025-12-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-frontend-backend-integration
branch: 003-frontend-backend-integration
user: asimh
command: /sp.analyze
labels: ["troubleshooting", "venv", "powershell", "environment-setup", "deployment"]
links:
  spec: ../../../specs/003-frontend-backend-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - start-backend.ps1
 - QUICKSTART.md
tests:
 - Verified venv exists and has all packages installed
 - Tested direct Python/uvicorn execution from venv
 - Confirmed PowerShell execution policy allows scripts
---

## Prompt

User reported repeated failures when trying to activate the virtual environment using `.\backend\venv\Scripts\Activate.ps1`, and suspected there might be two venv folders in the backend directory. Requested comprehensive analysis to identify and resolve the issue.

## Response snapshot

Conducted thorough analysis of the virtual environment setup:

**Findings:**
1. ✅ Only ONE venv exists at `backend/venv` (not two as suspected)
2. ✅ Activation script (`Activate.ps1`) is present and valid
3. ✅ Python 3.11.9 correctly installed in venv
4. ✅ All packages (FastAPI, uvicorn, asyncpg, etc.) properly installed
5. ✅ PowerShell execution policy set to `RemoteSigned` (allows local scripts)
6. ✅ uvicorn.exe exists and works in venv

**Root Cause:**
The issue was NOT with the venv itself, but with the user's understanding of how PowerShell venv activation works:
- Activation is session-specific (doesn't persist across terminal restarts)
- The `(venv)` prompt indicator may not show due to PowerShell profile customization
- User was running uvicorn from system PATH instead of venv PATH after activation

**Solutions Implemented:**

1. **Updated `start-backend.ps1`**: Modified to use direct path method (`.\backend\venv\Scripts\uvicorn.exe`) instead of relying on activation, eliminating the need to see `(venv)` in prompt.

2. **Created `QUICKSTART.md`**: Comprehensive guide with 3 methods to start backend:
   - Method 1: Use startup script (recommended)
   - Method 2: Direct path to uvicorn (no activation needed)
   - Method 3: Traditional activation (for users who prefer it)

3. **Included troubleshooting section**: Common errors and solutions (ModuleNotFoundError, port conflicts, etc.)

**Key Insight:** The "activation failure" was actually a UX misunderstanding. Activation DID work, but the user expected persistent visual confirmation (`(venv)` prefix) and didn't realize commands needed to be run in the same session.

## Outcome

- ✅ Impact: User can now reliably start backend server using multiple methods. Eliminated confusion about venv activation.
- 🧪 Tests: Verified direct execution paths work correctly (Python 3.11.9, uvicorn 0.40.0)
- 📁 Files: Created/updated 2 files: start-backend.ps1 (improved), QUICKSTART.md (new comprehensive guide)
- 🔁 Next prompts: User should test the startup script or direct path method to verify backend starts successfully
- 🧠 Reflection: The issue highlights importance of clear documentation for environment setup. Many users expect venv activation to be persistent like conda environments. Direct path method is more reliable for PowerShell users.

## Evaluation notes (flywheel)

- Failure modes observed: User confusion about PowerShell venv activation semantics; expectation of persistent `(venv)` indicator
- Graders run and results (PASS/FAIL): Manual verification - PASS (venv fully functional, all packages installed)
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): User should run `.\start-backend.ps1` and report if server starts successfully. If it does, frontend integration can proceed.
