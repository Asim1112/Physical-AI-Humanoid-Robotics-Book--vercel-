# Tasks: Frontend-Backend Integration for RAG Chatbot

**Input**: Design documents from `/specs/003-frontend-backend-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL and NOT included in this task list. The specification does not explicitly request TDD approach. Testing will be performed manually using quickstart.md validation procedures.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/` and `frontend/` at repository root
- Backend: `backend/api.py`, `backend/db.py`, `backend/models.py`, etc.
- Frontend: `frontend/src/components/`, `frontend/src/theme/`, etc.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment preparation

- [ ] T001 Verify Neon Postgres account created and copy connection string to backend/.env as NEON_DATABASE_URL
- [ ] T002 Verify all API keys from spec 002 (GROQ_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY) present in backend/.env
- [ ] T003 [P] Install backend dependencies: fastapi, uvicorn[standard], asyncpg, python-dotenv in backend/
- [ ] T004 [P] Install frontend dependency: @openai/chatkit-react in frontend/
- [ ] T005 Verify existing modules (agent.py, retrieve.py, embedding.py) are importable in backend/
- [ ] T006 [P] Create backend/.dockerignore file for Docker deployment
- [ ] T007 [P] Add FRONTEND_URL environment variable to backend/.env (http://localhost:3000 for dev)
- [ ] T008 [P] Add REACT_APP_API_URL environment variable to frontend/.env (http://localhost:8000 for dev)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Create backend/db.py with asyncpg connection pool setup (lifespan pattern)
- [ ] T010 Implement init_database() function in backend/db.py to create chat_sessions table with schema from data-model.md
- [ ] T011 Implement init_database() continuation in backend/db.py to create chat_messages table with foreign key to chat_sessions
- [ ] T012 [P] Create database indexes in backend/db.py: idx_last_accessed, idx_user_sessions, idx_session_messages, idx_recent_messages, idx_archived_cleanup
- [ ] T013 [P] Implement get_or_create_session(session_id: Optional[UUID]) function in backend/db.py
- [ ] T014 [P] Implement save_message(session_id, role, content, selected_text, response_time_ms, error_message) function in backend/db.py
- [ ] T015 [P] Implement get_session_messages(session_id, limit, offset) function in backend/db.py
- [ ] T016 [P] Implement delete_session(session_id) function with soft delete (archived=True) in backend/db.py
- [ ] T017 Create backend/models.py with Pydantic models: ChatRequest, ChatResponse, SessionResponse, SessionDetailResponse, MessageDetail per data-model.md
- [ ] T018 [P] Add Pydantic validators to ChatRequest model in backend/models.py (query length, selected_text truncation)
- [ ] T019 Create backend/api.py with FastAPI app initialization and lifespan event for database pool
- [ ] T020 Configure CORS middleware in backend/api.py with CORS_ORIGINS from environment (localhost:3000, localhost:5173, FRONTEND_URL)
- [ ] T021 [P] Add error handling middleware in backend/api.py to catch exceptions and return structured ErrorResponse
- [ ] T022 Modify backend/agent.py to add async function run_agent_query_async(query_text, selected_text, session_id) that calls existing agent

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Textbook via Chat Interface (Priority: P1) 🎯 MVP

**Goal**: Enable users to interact with the RAG agent through an embedded chat widget, asking questions and receiving accurate, context-based answers.

**Independent Test**: Open Docusaurus frontend → Open ChatWidget → Type "What is Gazebo simulation?" → Verify response displays within 5 seconds with relevant content about Gazebo simulator.

### Implementation for User Story 1

- [ ] T023 [US1] Implement POST /api/chat endpoint in backend/api.py that accepts ChatRequest and returns ChatResponse
- [ ] T024 [US1] Add logic to POST /api/chat in backend/api.py: validate request, get_or_create_session from database
- [ ] T025 [US1] Add logic to POST /api/chat in backend/api.py: determine retrieval mode (if selected_text is null, call retrieve from Qdrant; else use selected_text as context)
- [ ] T026 [US1] Add logic to POST /api/chat in backend/api.py: call run_agent_query_async with query and context
- [ ] T027 [US1] Add logic to POST /api/chat in backend/api.py: save user message and assistant response to database via save_message
- [ ] T028 [US1] Add logic to POST /api/chat in backend/api.py: return ChatResponse with response text, session_id, message_id, retrieved_chunks, response_time_ms
- [ ] T029 [P] [US1] Implement GET /health endpoint in backend/api.py that returns basic health status
- [ ] T030 [P] [US1] Implement GET /api/health endpoint in backend/api.py that checks database, Qdrant, and LLM API connectivity
- [ ] T031 [US1] Create frontend/src/components/ChatWidget/index.jsx with imports for ChatUI from @openai/chatkit-react
- [ ] T032 [US1] Add state management in frontend/src/components/ChatWidget/index.jsx: messages (useState), sessionId (useState with localStorage persistence)
- [ ] T033 [US1] Implement handleSendMessage function in frontend/src/components/ChatWidget/index.jsx: capture selected text with window.getSelection()
- [ ] T034 [US1] Add POST request logic in handleSendMessage in frontend/src/components/ChatWidget/index.jsx: send to REACT_APP_API_URL/api/chat with query, session_id, selected_text
- [ ] T035 [US1] Add response handling in handleSendMessage in frontend/src/components/ChatWidget/index.jsx: update messages state, persist session_id to localStorage
- [ ] T036 [US1] Add error handling in frontend/src/components/ChatWidget/index.jsx: display user-friendly error messages for network failures
- [ ] T037 [US1] Add loading state in frontend/src/components/ChatWidget/index.jsx: show loading indicator while waiting for response
- [ ] T038 [US1] Render ChatUI component in frontend/src/components/ChatWidget/index.jsx with messages prop and onSendMessage handler
- [ ] T039 [US1] Run npm swizzle command to eject Root component: npm run swizzle @docusaurus/theme-classic Root -- --eject in frontend/
- [ ] T040 [US1] Edit frontend/src/theme/Root.jsx to import ChatWidget and render it globally after children
- [ ] T041 [US1] Start backend server with uvicorn api:app --reload --host 0.0.0.0 --port 8000 from backend/ directory
- [ ] T042 [US1] Start frontend server with npm start from frontend/ directory
- [ ] T043 [US1] Test query "What is Gazebo simulation?" in ChatWidget and verify response mentions 3D robotics simulator
- [ ] T044 [US1] Verify response time is under 5 seconds for the test query
- [ ] T045 [US1] Verify message appears in chat history with proper markdown rendering

**Checkpoint**: At this point, User Story 1 should be fully functional - users can query the textbook via chat widget and receive responses

---

## Phase 4: User Story 2 - Session Persistence Across Page Navigation (Priority: P2)

**Goal**: Users can navigate between different pages while maintaining conversation history, allowing them to reference previous questions and answers.

**Independent Test**: Open ChatWidget → Ask "What is ROS 2?" → Navigate to different page → Verify chat history persists → Ask "How do I install it?" → Verify agent understands "it" refers to ROS 2.

### Implementation for User Story 2

- [ ] T046 [P] [US2] Implement POST /api/sessions endpoint in backend/api.py that creates new session and returns SessionResponse
- [ ] T047 [P] [US2] Implement GET /api/sessions/{session_id} endpoint in backend/api.py that retrieves session with message history
- [ ] T048 [P] [US2] Add pagination support to GET /api/sessions/{session_id} endpoint in backend/api.py (limit and offset query parameters)
- [ ] T049 [P] [US2] Implement DELETE /api/sessions/{session_id} endpoint in backend/api.py that soft-deletes session (sets archived=True)
- [ ] T050 [US2] Add useEffect hook in frontend/src/components/ChatWidget/index.jsx to restore session_id from localStorage on component mount
- [ ] T051 [US2] Add logic in frontend/src/components/ChatWidget/index.jsx to fetch session history from GET /api/sessions/{session_id} on mount if session_id exists
- [ ] T052 [US2] Update messages state in frontend/src/components/ChatWidget/index.jsx with fetched session history
- [ ] T053 [US2] Add session_id update logic in frontend/src/components/ChatWidget/index.jsx to persist to localStorage after every API response
- [ ] T054 [US2] Add "Clear chat" button in frontend/src/components/ChatWidget/index.jsx that calls DELETE /api/sessions/{session_id} and clears localStorage
- [ ] T055 [US2] Test multi-page navigation: Ask question on homepage → Navigate to Module 1 page → Verify chat history visible
- [ ] T056 [US2] Test session restoration: Close browser → Reopen within 24 hours → Verify conversation history restored
- [ ] T057 [US2] Test multi-turn context: Ask "What is ROS 2?" → Ask "How do I install it?" → Verify agent maintains context

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - chat persists across navigation and sessions are restored

---

## Phase 5: User Story 3 - Selected-Text Mode for Contextual Queries (Priority: P3)

**Goal**: Users can highlight specific text from the textbook and ask questions directly about that selection for more precise answers.

**Independent Test**: Navigate to textbook page → Highlight paragraph about "Gazebo physics simulation" → Open ChatWidget → Type "Explain this in simpler terms" → Verify response references the selected text specifically.

### Implementation for User Story 3

- [ ] T058 [US3] Add getSelectedText() helper function in frontend/src/components/ChatWidget/index.jsx that uses window.getSelection() and returns trimmed text (max 5000 chars)
- [ ] T059 [US3] Update handleSendMessage in frontend/src/components/ChatWidget/index.jsx to call getSelectedText() before sending request
- [ ] T060 [US3] Pass selected_text field in POST /api/chat request body in frontend/src/components/ChatWidget/index.jsx
- [ ] T061 [US3] Add visual indicator in frontend/src/components/ChatWidget/index.jsx to show when selected text is included as context (e.g., badge or pill)
- [ ] T062 [US3] Clear selected text in frontend/src/components/ChatWidget/index.jsx after query is sent (setSelectedText(null))
- [ ] T063 [US3] Update retrieval logic in POST /api/chat endpoint in backend/api.py to skip Qdrant retrieval if selected_text is provided
- [ ] T064 [US3] Use selected_text as context directly in run_agent_query_async call in backend/api.py when selected_text is not null
- [ ] T065 [US3] Set retrieved_chunks to 0 in ChatResponse when selected_text mode is used in backend/api.py
- [ ] T066 [US3] Test selected-text mode: Highlight "ROS 2 nodes" → Ask "What does this mean?" → Verify response prioritizes selected content
- [ ] T067 [US3] Test selection truncation: Highlight text >5000 chars → Verify only first 5000 chars used and warning displayed
- [ ] T068 [US3] Test selection clearing: Ask question with selection → Clear selection → Ask another question → Verify new query processed without previous selection

**Checkpoint**: All three user stories (1, 2, 3) should now be independently functional - full chat with persistence and selected-text support

---

## Phase 6: User Story 4 - Local Development and Production Deployment (Priority: P1) 🎯 MVP

**Goal**: Developers can run the full stack locally and deploy to production with frontend on Vercel connecting to hosted backend.

**Independent Test**: Run backend locally → Start frontend dev server → Verify chat works → Deploy frontend to Vercel with backend URL configured → Verify chat works in production.

### Implementation for User Story 4

- [ ] T069 [P] [US4] Create backend/Dockerfile with Python 3.10-slim base image, pip install, EXPOSE 8000, CMD uvicorn
- [ ] T070 [P] [US4] Test Dockerfile locally: docker build -t rag-backend . && docker run -p 8000:8000 --env-file .env rag-backend from backend/
- [ ] T071 [US4] Create Hugging Face Space with Docker type at https://huggingface.co/spaces
- [ ] T072 [US4] Push backend code to Hugging Face Space repository
- [ ] T073 [US4] Configure environment variables in Hugging Face Space settings (Secrets): NEON_DATABASE_URL, GROQ_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, FRONTEND_URL
- [ ] T074 [US4] Verify Hugging Face Space builds successfully and test with curl https://<space>.hf.space/health
- [ ] T075 [US4] Update frontend/.env with production REACT_APP_API_URL=https://<space>.hf.space
- [ ] T076 [US4] Add REACT_APP_API_URL to Vercel project environment variables via Vercel dashboard
- [ ] T077 [US4] Push to main branch to trigger Vercel deployment from frontend/
- [ ] T078 [US4] Verify Vercel deployment completes at https://your-site.vercel.app
- [ ] T079 [US4] Test production ChatWidget end-to-end: Ask "What is Gazebo simulation?" → Verify response
- [ ] T080 [US4] Verify CORS configured correctly in production (no console errors in browser DevTools)
- [ ] T081 [US4] Test production session persistence: Ask query → Refresh page → Verify history restored

**Checkpoint**: All user stories complete and deployed - production system fully operational

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [ ] T082 [P] Run 5 required test queries from quickstart.md: "What is Gazebo simulation?", "What are the key features of ROS?", Highlight "Inverse kinematics" + "Explain this concept", "How does it relate to forward kinematics?", "What sensors are used in humanoid robots?"
- [ ] T083 [P] Verify all 5 test queries pass with responses under 5 seconds (SC-003)
- [ ] T084 [P] Test edge case: Empty query → Verify 400 error returned
- [ ] T085 [P] Test edge case: Query >2000 chars → Verify 400 error returned
- [ ] T086 [P] Test edge case: Invalid session_id → Verify 404 error returned
- [ ] T087 [P] Test edge case: Backend down → Verify user-friendly error message displayed
- [ ] T088 [P] Test edge case: Selected text >5000 chars → Verify automatic truncation
- [ ] T089 [P] Test mobile responsiveness: Open in DevTools with 375px width → Verify chat widget functional
- [ ] T090 [P] Benchmark response times: Run 10 queries and calculate p50, p95, p99 latencies
- [ ] T091 Update README.md in project root with quick start instructions and link to quickstart.md
- [ ] T092 [P] Create DEPLOYMENT.md with environment variable reference and deployment procedures
- [ ] T093 [P] Add troubleshooting section to DEPLOYMENT.md for common issues (CORS, database connection, etc.)
- [ ] T094 Prepare demo talking points: "Query textbook via natural language", "Highlight text for contextual questions", "Conversations persist across pages"
- [ ] T095 [P] Verify all functional requirements (FR-001 to FR-020) from spec.md are satisfied
- [ ] T096 [P] Verify all success criteria (SC-001 to SC-010) from spec.md are met

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational phase completion
- **User Story 2 (Phase 4)**: Depends on Foundational phase completion - Can run in parallel with US1 after foundation ready
- **User Story 3 (Phase 5)**: Depends on Foundational phase completion - Can run in parallel with US1/US2 after foundation ready
- **User Story 4 (Phase 6)**: Depends on Foundational phase completion - Deployment can happen after US1 is complete for MVP
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories - **REQUIRED FOR MVP**
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 endpoints but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Extends US1 with selected-text feature but independently testable
- **User Story 4 (P1)**: Can start after Foundational (Phase 2) - Deployment story, can be done after US1 for MVP - **REQUIRED FOR MVP**

### Within Each User Story

- Backend endpoints before frontend components that call them
- Database CRUD functions before API endpoints that use them
- Core implementation before edge case handling
- Local testing before production deployment
- Story complete before moving to next priority

### Parallel Opportunities

- **Phase 1 (Setup)**: T003, T004, T006, T007, T008 can all run in parallel (different files, no dependencies)
- **Phase 2 (Foundational)**: T012, T013, T014, T015, T016, T018, T021 can run in parallel after T009-T011 complete
- **After Foundational phase completes**: User Stories 1, 2, 3 can all start in parallel if team capacity allows
- **Within User Story 1**: T029, T030 (health endpoints) can run in parallel with T023-T028 (main chat endpoint)
- **Within User Story 2**: T046, T047, T048, T049 (session endpoints) can all run in parallel
- **Within User Story 4**: T069, T070 (Docker) can run in parallel with T075-T077 (Vercel config)
- **Phase 7 (Polish)**: T082-T090 (validation tests) can all run in parallel

---

## Parallel Example: User Story 1

```bash
# After Foundational phase (Phase 2) completes, launch User Story 1 tasks:

# Backend endpoints can be implemented in parallel:
Task T029: "Implement GET /health endpoint in backend/api.py"
Task T030: "Implement GET /api/health endpoint in backend/api.py"
# (while T023-T028 implement the main chat endpoint sequentially)

# Frontend widget can be built while backend is being developed:
Task T031-T038: "Build ChatWidget component in frontend/src/components/ChatWidget/"
# (can start after T023 POST /api/chat endpoint signature is defined)

# Deployment tasks can run in parallel:
Task T069: "Create Dockerfile"
Task T070: "Test Docker build locally"
```

---

## Parallel Example: After Foundational Phase

```bash
# Once Phase 2 (Foundational) completes, if you have multiple developers:

Developer A: Start on User Story 1 (Phase 3) - Chat interface
Developer B: Start on User Story 2 (Phase 4) - Session persistence endpoints
Developer C: Start on User Story 3 (Phase 5) - Selected-text mode
Developer D: Start on User Story 4 (Phase 6) - Deployment setup

# Stories can proceed independently and be tested independently
# Each developer can complete their story without blocking others
```

---

## Implementation Strategy

### MVP First (User Story 1 + User Story 4 Only)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T022) - CRITICAL
3. Complete Phase 3: User Story 1 (T023-T045) - Core chat functionality
4. Complete Phase 6: User Story 4 (T069-T081) - Deployment
5. **STOP and VALIDATE**: Run quickstart.md tests for basic chat functionality
6. Deploy MVP: Chat widget with basic query/response capability

**MVP Delivers**: Users can query textbook via chat widget in production. Sessions are temporary (no persistence or selected-text yet).

### Incremental Delivery

1. **MVP**: Setup → Foundational → User Story 1 → User Story 4 → Deploy
2. **v1.1**: Add User Story 2 (Session persistence) → Test independently → Deploy
3. **v1.2**: Add User Story 3 (Selected-text mode) → Test independently → Deploy
4. **v1.3**: Complete Phase 7 (Polish) → Full validation → Final deploy

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With 2-3 developers after completing Setup + Foundational:

**Week 1**:
- Dev 1: User Story 1 (Chat interface) - **CRITICAL PATH**
- Dev 2: User Story 4 (Deployment setup) - can start early
- Dev 3: User Story 2 (Session persistence) - parallel work

**Week 2**:
- Dev 1: User Story 3 (Selected-text mode)
- Dev 2: Polish & validation (Phase 7)
- Dev 3: Documentation and demo prep

Stories complete and integrate independently without blocking each other.

---

## Notes

- **[P] tasks** = different files, no dependencies, can run in parallel
- **[Story] label** maps task to specific user story for traceability
- **Each user story** should be independently completable and testable
- **Stop at any checkpoint** to validate story independently
- **Backend path**: `backend/api.py`, `backend/db.py`, `backend/models.py`, `backend/agent.py`
- **Frontend path**: `frontend/src/components/ChatWidget/`, `frontend/src/theme/Root.jsx`
- **Environment variables**: Must be set before running (see quickstart.md)
- **Database initialization**: Run `python -c "from db import init_database; import asyncio; asyncio.run(init_database())"` after T009-T012 complete
- **Testing**: Manual testing using quickstart.md procedures (no automated tests in this task list)
- **Commit frequently**: After each task or logical group
- **Avoid**: Vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Summary

**Total Tasks**: 96
- **Phase 1 (Setup)**: 8 tasks
- **Phase 2 (Foundational)**: 14 tasks (BLOCKS all user stories)
- **Phase 3 (User Story 1)**: 23 tasks - Query Textbook via Chat Interface (P1 MVP)
- **Phase 4 (User Story 2)**: 12 tasks - Session Persistence Across Page Navigation (P2)
- **Phase 5 (User Story 3)**: 11 tasks - Selected-Text Mode for Contextual Queries (P3)
- **Phase 6 (User Story 4)**: 13 tasks - Local Development and Production Deployment (P1 MVP)
- **Phase 7 (Polish)**: 15 tasks - Validation and documentation

**Parallel Opportunities**: 35 tasks marked [P] can run in parallel within their phase
**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (User Story 1) + Phase 6 (User Story 4) = 58 tasks
**Independent Stories**: Each user story (2, 3, 4) can be developed and tested independently after Foundational phase

**Estimated Timeline**:
- MVP (US1 + US4): ~8-10 hours (from plan.md: 1h setup + 2h database + 3h API + 3h frontend + 2h deployment)
- Full Feature (All stories): ~14 hours (from plan.md phases)
- With parallel team: ~6-8 hours (multiple stories simultaneously)
