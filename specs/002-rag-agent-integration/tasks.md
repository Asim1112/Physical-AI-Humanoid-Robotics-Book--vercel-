# Tasks: RAG Agent Integration

**Input**: Design documents from `/specs/002-rag-agent-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/agent_contract.md, quickstart.md

**Tests**: The spec explicitly requests comprehensive testing with at least 5 diverse queries, validation of multi-turn conversations, and error scenario tests.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for agent functionality

- [x] T001 Verify Python 3.13 and UV package manager are installed and configured
- [x] T002 [P] Verify existing backend structure (retrieve.py, embedding.py, storage.py from Spec 1-2)
- [x] T003 [P] Verify Qdrant collection contains embedded content from Spec 1 (1,278+ chunks)
- [x] T004 Verify .env file contains required credentials (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY)
- [x] T005 Install OpenAI Agents SDK and verify compatibility with existing backend structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core agent infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 [P] Create data classes for AgentRequest entity in backend/agent.py (query_text, selected_text, session_id, temperature, max_turns, top_k, score_threshold)
- [x] T007 [P] Create data classes for AgentResponse entity in backend/agent.py (response_text, retrieved_chunks, conversation_id, response_time_ms, query_tokens, response_tokens, retrieval_success)
- [x] T008 Create data class for AgentSession entity in backend/agent.py (session_id, created_at, last_accessed, conversation_history, current_turn)
- [x] T009 [P] Create data class for ConversationTurn entity in backend/agent.py (turn_number, user_input, retrieved_chunks, agent_response, timestamp, tool_calls)
- [x] T010 Create data class for AgentConfiguration entity in backend/agent.py (model_name, temperature, max_tokens, system_prompt, retrieval_enabled, max_turns, score_threshold)
- [x] T011 Create data class for RetrievalToolInput entity in backend/agent.py (query_text, top_k, score_threshold, selected_text)
- [x] T012 Create data class for RetrievalToolOutput entity in backend/agent.py (retrieved_chunks, query_embedding, search_time_ms, total_candidates, success)
- [x] T013 Verify retrieve.py exports retrieve_content() function for Qdrant vector search
- [x] T014 Verify embedding.py exports generate_embedding() function with search_query input type support
- [x] T015 Implement base run_agent_query() function in backend/agent.py with validation and logging

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Textbook with Agent Responses (Priority: P1) 🎯 MVP

**Goal**: Successfully create an AI agent that can process user queries by embedding them with Cohere, retrieve relevant chunks from Qdrant, and generate responses using OpenAI's agent capabilities

**Independent Test**: Submit query "How do I install ROS 2?" and verify the agent retrieves relevant chunks from Qdrant, augments its prompt with that context, and generates an accurate response that references the textbook content

### Tests for User Story 1 (Red Phase - Write Tests First)

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T016 [P] [US1] Unit test for AgentRequest data validation in backend/tests/test_agent.py (test_request_validation_success, test_request_validation_empty_query)
- [x] T017 [P] [US1] Unit test for AgentResponse data validation in backend/tests/test_agent.py (test_response_validation_success, test_response_validation_missing_fields)
- [x] T018 [P] [US1] Unit test for run_agent_query() function in backend/tests/test_agent.py (test_agent_query_success, test_agent_query_empty_query)
- [x] T019 [US1] Integration test for ROS2 query in backend/tests/test_agent_integration.py (test_ros2_installation_query with expected retrieval and response)
- [x] T020 [US1] Integration test for Gazebo query in backend/tests/test_agent_integration.py (test_gazebo_simulation_query with expected retrieval and response)

### Implementation for User Story 1 (Green Phase)

- [x] T021 [US1] Implement AgentRequest validation in run_agent_query() (check for empty/whitespace, length <1000 chars, parameter ranges)
- [x] T022 [US1] Implement OpenAI Agent initialization with Gemini 2.0 Flash via OpenAI-compatible API in backend/agent.py
- [x] T023 [US1] Implement RetrievalTool as function_tool wrapper around retrieve_content() in backend/agent.py
- [x] T024 [US1] Implement agent system prompt that instructs agent to use retrieval tool for textbook queries in backend/agent.py
- [x] T025 [US1] Implement response generation with retrieved context augmentation in backend/agent.py
- [x] T026 [US1] Implement response time measurement in run_agent_query() (start_time to end_time)
- [x] T027 [US1] Add logging for agent processing stages in run_agent_query() (tool calls, responses, retrieval results)
- [x] T028 [US1] Run all User Story 1 tests and verify they pass

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Multi-Turn Conversations with Context (Priority: P2)

**Goal**: Run comprehensive tests with multi-turn conversations where the agent remembers previous queries and answers, allowing for follow-up questions and clarifications about the textbook content

**Independent Test**: Submit a first query "What is ROS 2?", then a follow-up "How do I install it?" without re-stating the context. Verify the agent understands "it" refers to ROS 2 from the previous turn and retrieves installation-specific content.

### Tests for User Story 2 (Red Phase - Write Tests First)

- [x] T029 [P] [US2] Unit test for AgentSession data validation in backend/tests/test_agent.py (test_session_validation_success, test_session_validation_invalid_id)
- [x] T030 [P] [US2] Unit test for ConversationTurn data validation in backend/tests/test_agent.py (test_turn_validation_success, test_turn_validation_missing_fields)
- [x] T031 [US2] Integration test for VLA models conversation in backend/tests/test_agent_integration.py (test_vla_models_conversation with context maintenance)
- [x] T032 [US2] Integration test for humanoid robots conversation in backend/tests/test_agent_integration.py (test_humanoid_robots_conversation with context maintenance)
- [x] T033 [US2] Integration test for 10+ turn conversation in backend/tests/test_agent_integration.py (test_long_conversation with context maintenance)

### Implementation for User Story 2 (Green Phase)

- [x] T034 [US2] Implement create_agent_session() function in backend/agent.py with session ID generation
- [x] T035 [US2] Implement continue_agent_session() function in backend/agent.py with conversation history management
- [x] T036 [US2] Implement session persistence using OpenAI Agents SDK session capabilities in backend/agent.py
- [x] T037 [US2] Add conversation history to agent context for multi-turn processing in backend/agent.py
- [x] T038 [US2] Implement max_turns enforcement to prevent infinite conversations in backend/agent.py
- [x] T039 [US2] Add detailed logging for conversation turns and history in backend/agent.py
- [x] T040 [US2] Implement session management API endpoints in backend/agent.py (GET/DELETE /api/agent/session/{id})
- [x] T041 [US2] Run all User Story 2 tests and verify they pass

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Selected Text Mode for Targeted Queries (Priority: P3)

**Goal**: Handle selected-text queries where users can highlight specific text from the textbook interface and ask questions specifically about that selection, with the agent prioritizing the selected context in its retrieval and response

**Independent Test**: Pass a query "Explain this" along with selected text from Module 1 about ROS 2 nodes. Verify the agent retrieves additional context related to the selected text and generates a response that directly addresses the selection.

### Tests for User Story 3 (Red Phase - Write Tests First)

- [x] T042 [P] [US3] Unit test for selected-text mode validation in backend/tests/test_agent.py (test_selected_text_validation_success, test_selected_text_validation_invalid_input)
- [x] T043 [P] [US3] Unit test for selected-text mode processing in backend/tests/test_agent.py (test_selected_text_processing_success, test_selected_text_processing_empty_selection)
- [x] T044 [US3] Integration test for Gazebo physics simulation selected-text query in backend/tests/test_agent_integration.py (test_gazebo_selected_text_query with targeted retrieval)
- [x] T045 [US3] Integration test for ROS2 nodes selected-text query in backend/tests/test_agent_integration.py (test_ros2_nodes_selected_text_query with targeted retrieval)
- [x] T046 [US3] Integration test for no-results fallback in selected-text mode in backend/tests/test_agent_integration.py (test_selected_text_no_results_fallback)

### Implementation for User Story 3 (Green Phase)

- [x] T047 [P] [US3] Add selected_text parameter validation in AgentRequest validation (length limits, content checks)
- [x] T048 [P] [US3] Modify RetrievalTool to handle selected-text context in backend/agent.py
- [x] T049 [US3] Implement selected-text mode in run_agent_query() function with context augmentation
- [x] T050 [US3] Update agent system prompt to handle selected-text queries appropriately in backend/agent.py
- [x] T051 [US3] Add selected-text biasing to retrieval process to prioritize relevant source modules in backend/agent.py
- [x] T052 [US3] Implement fallback response when selected-text retrieval returns no results in backend/agent.py
- [x] T053 [US3] Add detailed logging for selected-text processing in backend/agent.py
- [x] T054 [US3] Run all User Story 3 tests and verify they pass

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [x] T055 [P] Add comprehensive docstrings to all functions in backend/agent.py (Google-style docstrings)
- [x] T056 [P] Add type hints to all function parameters and return values in backend/agent.py
- [x] T057 [P] Add inline comments for complex logic in agent functions and tool implementations
- [x] T058 Code cleanup and refactoring in backend/agent.py (remove duplicate code, improve readability)
- [x] T059 [P] Run pytest for all tests in backend/tests/ and verify 100% pass rate
- [x] T060 [P] Run linting checks (ruff or flake8) on backend/agent.py and fix any issues
- [x] T061 Performance optimization: Review and optimize agent initialization and tool call overhead
- [x] T062 Security hardening: Ensure API keys are never logged, sanitize error messages
- [x] T063 Implement error resilience with retry logic for API failures in backend/agent.py
- [x] T064 Run full quickstart.md validation following all steps in the guide
- [x] T065 Generate validation report with final performance metrics (response times, success rates, accuracy)
- [x] T066 Update backend/README.md with agent functionality documentation
- [x] T067 Implement health check endpoint GET /api/agent/health with dependency status
- [x] T068 Implement streaming response mode support in backend/agent.py
- [x] T069 Create validation script that runs 5+ sample queries covering all textbook modules
- [x] T070 Update main.py to include agent orchestration and CLI flags for agent functionality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 for conversation features but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Integrates with US1 for selected-text features but should be independently testable

### Within Each User Story

- Tests (Red Phase) MUST be written and FAIL before implementation (Green Phase)
- Data classes before functions that use them
- Core run_agent_query() before conversation/selected-text extensions
- Unit tests before integration tests
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002, T003, T004)
- All Foundational data class tasks marked [P] can run in parallel (T006, T007, T009)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can be written in parallel
- Different user stories can be worked on in parallel by different team members
- All polish tasks marked [P] can run in parallel (T055-T057, T059-T060)

### Parallel Example: User Story 1 Tests

```bash
# Launch all User Story 1 tests together (Red Phase):
Task: "Unit test for AgentRequest data validation in backend/tests/test_agent.py"
Task: "Unit test for AgentResponse data validation in backend/tests/test_agent.py"
Task: "Unit test for run_agent_query() function in backend/tests/test_agent.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Query Textbook with Agent Responses)
4. **STOP and VALIDATE**: Test User Story 1 independently with sample ROS2 and Gazebo queries
5. Verify response latency <5 seconds and accuracy of textbook content citations

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Validate with sample queries (MVP!)
3. Add User Story 2 → Test independently → Run multi-turn conversation tests
4. Add User Story 3 → Test independently → Verify selected-text mode functionality
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Basic agent functionality)
   - Developer B: User Story 2 (Multi-turn conversations)
   - Developer C: User Story 3 (Selected-text mode)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail (Red Phase) before implementing (Green Phase)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- All file paths use backend/ directory from existing Spec 1-2 structure
- Testing follows pytest conventions with test_*.py naming
- Integration tests require actual Qdrant, Cohere, and Gemini API connections (use fixtures for mocking if needed)