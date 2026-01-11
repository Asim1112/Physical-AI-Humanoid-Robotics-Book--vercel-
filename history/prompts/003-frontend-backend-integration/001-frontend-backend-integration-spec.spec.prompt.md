---
id: 001
title: Frontend-Backend Integration Specification
stage: spec
date: 2025-12-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-frontend-backend-integration
branch: 003-frontend-backend-integration
user: user
command: /sp.specify
labels: ["specification", "frontend", "backend", "chat-widget", "api", "deployment"]
links:
  spec: specs/003-frontend-backend-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-frontend-backend-integration/spec.md
  - specs/003-frontend-backend-integration/checklists/requirements.md
tests:
  - Specification quality checklist validation (PASSED)
---

## Prompt

Integrate backend with frontend for RAG chatbot in Physical AI & Humanoid Robotics textbook

Target audience: Hackathon participants and evaluators for the textbook project

Focus: Establish FastAPI backend endpoints to serve the OpenAI agent with retrieval, integrate Neon Serverless Postgres for session management, and embed the chatbot UI in the Docusaurus frontend using OpenAI ChatKit SDK, enabling local and deployed connections for querying the book's content including selected-text mode

Success criteria:

- Successfully exposes FastAPI endpoints (e.g., /chat for agent queries) that handle full RAG and selected-text queries, returning responses from the agent

- Integrates Neon Serverless Postgres to store chat sessions (e.g., user queries, responses, timestamps) for multi-turn persistence

- Embeds ChatKit SDK in frontend as a widget, connecting to FastAPI API (local for dev, deployed URL for prod)

- Handles selected-text: Frontend captures user selection and sends as context to backend, overriding full retrieval

- Tests end-to-end: At least 5 queries via the UI, verifying responses, session persistence, and deployment on Vercel with backend on a host like Hugging Face

Constraints:

- Use FastAPI for backend API in the existing backend folder

- Use OpenAI ChatKit SDK for frontend chat interface (install via npm in frontend)

- Build upon prior specs: Expose the agent from agent.py via FastAPI routes, reuse retrieval and embedding logic

- Incorporate Neon Postgres for relational data (e.g., sessions table)

- Implement in Python (backend) and React/JS (frontend), updating docusaurus.config.js and src/components as needed

- Handle deployment: Update Vercel for frontend; note backend deployment separately

- Timeline: Complete as fourth and final spec in RAG sequence

## Response snapshot

Created comprehensive specification for frontend-backend integration feature with the following structure:

**User Stories (4 total)**:
1. P1 (MVP): Query Textbook via Chat Interface - Core chat functionality
2. P2: Session Persistence Across Page Navigation - Conversation history maintenance
3. P3: Selected-Text Mode for Contextual Queries - Enhanced precision for specific questions
4. P1 (MVP): Local Development and Production Deployment - Dev workflow and prod deployment

**Functional Requirements**: 20 requirements (FR-001 to FR-020) covering:
- REST API endpoints for chat queries, streaming, and session management
- Database integration for session storage
- Chat widget embedding and configuration
- CORS, validation, error handling
- Responsive design and error boundaries
- Reuse of existing agent and retrieval logic

**Key Entities**: 5 entities defined:
- ChatSession: Conversation state in database
- ChatMessage: Individual messages with metadata
- APIRequest/APIResponse: API contract structures
- ChatWidget: Frontend component state

**Success Criteria**: 10 measurable outcomes (SC-001 to SC-010):
- Response time <5 seconds for 95% of queries
- 24-hour session persistence
- 5+ end-to-end test queries
- 90% accuracy for selected-text mode
- 2-second widget load time
- 50 concurrent users support
- Successful production deployment
- User-friendly error handling
- Mobile responsiveness (375px+)
- Comprehensive module coverage testing

**Edge Cases**: 10 scenarios including API failures, network timeouts, database issues, browser limitations, mobile considerations, and concurrent user handling

**Validation**: Created specification quality checklist - all checks PASSED. No [NEEDS CLARIFICATION] markers. Specification ready for planning phase.

## Outcome

- ✅ Impact: Complete technology-agnostic specification created for RAG chatbot frontend-backend integration
- 🧪 Tests: Specification quality checklist validation passed (all 12 items)
- 📁 Files: Created spec.md (138 lines) and requirements checklist
- 🔁 Next prompts: `/sp.clarify` (optional) or `/sp.plan` (recommended next step)
- 🧠 Reflection: Successfully abstracted technical implementation details (FastAPI, ChatKit, Neon Postgres) from the specification while preserving user value and measurable outcomes. User stories properly prioritized with MVP markers. Success criteria are technology-agnostic and measurable.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification generated without clarification needs
- Graders run and results (PASS/FAIL): Specification quality checklist - PASS (12/12 items)
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Proceed directly to /sp.plan to create architectural design and technical implementation plan
