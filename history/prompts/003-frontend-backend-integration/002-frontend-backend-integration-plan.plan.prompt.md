---
id: 002
title: Frontend-Backend Integration Implementation Plan
stage: plan
date: 2025-12-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-frontend-backend-integration
branch: 003-frontend-backend-integration
user: user
command: /sp.plan
labels: ["planning", "architecture", "frontend", "backend", "fastapi", "docusaurus", "neon-postgres", "chatkit"]
links:
  spec: specs/003-frontend-backend-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-frontend-backend-integration/plan.md
  - specs/003-frontend-backend-integration/research.md
  - specs/003-frontend-backend-integration/data-model.md
  - specs/003-frontend-backend-integration/quickstart.md
  - specs/003-frontend-backend-integration/contracts/openapi.yaml
tests:
  - No tests created during planning phase (planning artifacts only)
---

## Prompt

Create: integration architecture overview, API endpoint designs, database schema for Neon, frontend widget component, deployment configs, testing scripts, and end-to-end validation reports.

Decisions needing documentation: choices for FastAPI route structures (e.g., async vs. sync) with tradeoffs on performance; Neon schema design (e.g., sessions table fields) balancing simplicity and extensibility; ChatKit configuration options for UI customization vs. defaults; CORS policies for secure cross-origin requests.

Testing strategy: unit tests for API endpoints and DB operations; integration tests for agent calls via API with sample queries; frontend tests for chat widget rendering and selected-text capture; end-to-end tests with at least 5 UI-driven queries verifying responses, session persistence in Neon, and selected-text mode; deployment verification on Vercel (frontend) and a host like Hugging Face (backend).

Technical details:
- Build upon prior specs: Import and expose the agent from agent.py, retrieval from retrieve.py, and ingestion from main.py as needed in the FastAPI app.
- In backend, create api.py for FastAPI implementation (app setup, routes like /chat POST for queries with optional selected_text, integration with agent and retrieval); create db.py for Neon Postgres setup (connection, schema creation for sessions table with columns like id, user_id, query, response, timestamp, and CRUD functions).
- In frontend, install OpenAI ChatKit SDK via npm, create a custom React component (e.g., src/components/ChatWidget.jsx) for the chat interface, add logic to capture window.getSelection() for selected text, and integrate into docusaurus.config.js or pages for embedding as a floating widget.
- Organize by phases: Backend Setup (FastAPI + Neon) → API Development → Frontend Integration → Deployment → Testing → Validation
- Main function: In api.py, define the FastAPI app with lifespan for DB connection, endpoints that handle queries by calling the agent (embed with Cohere, retrieve from Qdrant if not selected_text, augment and respond), store sessions in Neon, handle environment variables for API keys and DB connection string, and include a root endpoint for health checks.

## Response snapshot

Successfully created comprehensive implementation plan with complete architecture, design decisions, and phased approach:

**Plan Structure**:
- Executive Summary with 5 key architectural decisions
- Detailed architecture overview with system components diagram
- Technology stack breakdown (backend, frontend, infrastructure)
- 6 key architectural decisions with options analysis, rationale, and trade-offs:
  1. Async vs Sync FastAPI Routes → **Decision: Async-first** (handles I/O-bound ops efficiently)
  2. Database Schema Design → **Decision: Two-table normalized schema** (sessions + messages)
  3. CORS Configuration Strategy → **Decision: Whitelist specific origins** with environment config
  4. ChatKit SDK vs Custom Component → **Decision: Use ChatKit SDK** (faster time to MVP)
  5. Selected-Text Capture Mechanism → **Decision: window.getSelection()** (native API, no dependencies)
  6. Connection Pooling for Neon → **Decision: asyncpg pool with lifespan** (efficient reuse)

**Implementation Phases (6 total)**:
- Phase 0: Prerequisites & Setup (1 hour) - Environment prep and dependency verification
- Phase 1: Database Layer (2 hours) - Schema and CRUD operations
- Phase 2: API Layer (3 hours) - FastAPI endpoints and middleware
- Phase 3: Frontend ChatWidget (3 hours) - React component with ChatKit SDK
- Phase 4: Integration & Testing (2 hours) - E2E tests and edge cases
- Phase 5: Deployment (2 hours) - Hugging Face Spaces (backend) + Vercel (frontend)
- Phase 6: Documentation (1 hour) - README, deployment guide, demo materials

**Supporting Artifacts Created**:
- **research.md** (126KB): Comprehensive architectural research with FastAPI patterns, Neon schema design, ChatKit integration
- **data-model.md**: Complete database schemas (SQL), API contracts (Pydantic models), frontend state models (React)
- **contracts/openapi.yaml**: Full OpenAPI 3.0 specification with 7 endpoints, request/response schemas, examples
- **quickstart.md**: Step-by-step developer guide with setup instructions, test procedures, troubleshooting

**Risk Analysis**: 5 risks identified with mitigation strategies:
1. Database connection pool exhaustion → Monitor + retry logic
2. LLM API rate limiting → Exponential backoff + user-friendly messages
3. CORS misconfiguration → Environment variables + pre-demo testing
4. Session data loss → Neon backups + soft delete pattern
5. ChatKit SDK load failure → Bundle with Webpack + error boundaries

**Testing Strategy**:
- Unit tests: 80% backend coverage, 70% frontend coverage
- Integration tests: Full chat flow, session persistence, selected-text mode, CORS verification
- E2E tests: 5 required scenarios (basic query, multi-turn, selected-text, persistence, errors)
- Performance tests: 50 concurrent users, p95 < 5s response time

**Success Metrics**: 10 measurable targets (SC-001 to SC-010) from specification validated in plan

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation via /sp.tasks
- 🧪 Tests: No tests created during planning (planning phase only)
- 📁 Files: Created plan.md (1027 lines), research.md (comprehensive), data-model.md, quickstart.md, openapi.yaml
- 🔁 Next prompts: `/sp.tasks` (required next step to generate atomic tasks from this plan)
- 🧠 Reflection: Successfully translated user's technical requirements into a structured, phased implementation plan with clear architectural decisions, risk analysis, and acceptance criteria. All 6 key architectural decisions documented with rationale and trade-offs. Plan includes comprehensive testing strategy, deployment architecture, and success metrics aligned with specification.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan generation completed successfully without clarification needs
- Graders run and results (PASS/FAIL): N/A (planning phase does not include graders)
- Prompt variant (if applicable): Standard /sp.plan workflow with detailed technical requirements provided by user
- Next experiment (smallest change to try): Proceed directly to /sp.tasks to generate atomic, testable tasks from the implementation plan. Verify that tasks map 1:1 with phase checklist items in plan.md.
