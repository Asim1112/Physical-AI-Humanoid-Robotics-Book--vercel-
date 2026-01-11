---
id: 002
title: FastAPI Best Practices Research
stage: misc
date: 2025-12-29
surface: agent
model: claude-sonnet-4-5
feature: 003-frontend-backend-integration
branch: 003-frontend-backend-integration
user: developer
command: research
labels: ["fastapi", "research", "backend", "best-practices", "async", "cors", "validation"]
links:
  spec: specs/003-frontend-backend-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-frontend-backend-integration/fastapi-best-practices.md
tests:
  - None (research task)
---

## Prompt

Research and document best practices for FastAPI implementation in this context:

**Context**: Building a RAG chatbot backend API that needs to:
- Expose chat endpoints for user queries
- Integrate with existing Python agent (from agent.py)
- Support both sync and async operations
- Handle session management with Neon Postgres
- Support streaming responses
- Implement CORS for Vercel frontend

**Research Topics**:
1. **Async vs Sync Routes**: When to use async def vs def for FastAPI routes, performance implications, and best practices for I/O-bound operations (database, LLM calls)

2. **FastAPI Application Structure**: Best practices for organizing FastAPI apps with multiple endpoints, dependency injection for database connections, lifespan events for setup/teardown

3. **CORS Configuration**: Secure CORS setup for allowing Vercel deployments and localhost during development, handling preflight requests

4. **Error Handling**: Best practices for FastAPI error handling, custom exception handlers, returning appropriate HTTP status codes

5. **Request Validation**: Using Pydantic models for request/response validation, handling optional fields (like selected_text)

**Output Format**:
For each topic, provide:
- **Decision**: What approach to use
- **Rationale**: Why this is the best choice
- **Code Example**: Brief example showing the pattern
- **Alternatives Considered**: Other options and why they weren't chosen

## Response snapshot

Created comprehensive FastAPI best practices document at:
`specs/003-frontend-backend-integration/fastapi-best-practices.md`

The document covers 7 major topics with detailed decisions, rationale, and code examples:

1. **Async vs Sync Routes**: Use `async def` for all routes
   - Rationale: All operations are I/O-bound (DB, vector search, LLM calls)
   - Integration: Existing agent.py already provides async functions
   - Performance: 10-100x better concurrency for I/O-bound work

2. **Application Structure**: Modular routers + dependency injection + lifespan events
   - Separation of concerns with routers (chat, sessions, health)
   - Dependency injection for DB pool and agent instance
   - Lifespan events for proper resource management
   - Directory structure provided

3. **CORS Configuration**: Environment-specific with explicit origins
   - Security: No wildcard origins in production
   - Flexibility: localhost for dev, Vercel URL for production
   - Automatic preflight handling by CORSMiddleware

4. **Error Handling**: Custom exceptions + centralized handlers
   - Consistent error format for frontend parsing
   - HTTP status codes (400 validation, 404 not found, 500 server error)
   - Sanitized errors for security (no internal details leaked)

5. **Request Validation**: Pydantic V2 with field validators
   - Type safety and runtime validation
   - Auto-generated OpenAPI docs
   - Custom validators for complex rules
   - Examples for ChatRequest and ChatResponse models

6. **Streaming Responses**: Server-Sent Events (SSE) with async generators
   - Integration with existing run_agent_query_streaming()
   - Real-time UX for LLM responses
   - Browser-native EventSource API support

7. **Database Integration**: asyncpg with connection pooling
   - Fastest async Postgres driver
   - Neon Serverless Postgres compatibility
   - Connection pool management in lifespan
   - Schema examples for sessions and messages

Each topic includes:
- Decision summary
- Detailed rationale
- Production-ready code examples
- Alternatives considered and why rejected

The document also includes:
- Integration guide with existing agent.py code
- Minimal FastAPI wrapper example
- Deployment considerations (local + production)
- Testing strategy with pytest
- Summary table of key decisions

## Outcome

- ✅ Impact: Comprehensive research document provides clear implementation guidance for FastAPI backend, directly addressing all requirements from spec (FR-001 through FR-020)
- 🧪 Tests: No tests created (research task); testing strategy documented in research
- 📁 Files: Created `specs/003-frontend-backend-integration/fastapi-best-practices.md` (9,600+ words, 7 major topics with code examples)
- 🔁 Next prompts: Ready to create implementation plan (plan.md) and tasks (tasks.md) based on these research findings
- 🧠 Reflection: Research successfully bridges existing agent.py architecture with FastAPI best practices. Key insight: async-first approach aligns perfectly with existing async agent functions, enabling seamless integration.

## Evaluation notes (flywheel)

- Failure modes observed: None; web search tools unavailable but compensated with comprehensive technical knowledge and codebase analysis
- Graders run and results (PASS/FAIL): N/A (research task)
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Create plan.md to structure the research into concrete architecture decisions, then tasks.md for implementation
