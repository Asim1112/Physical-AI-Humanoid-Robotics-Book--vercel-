# Tasks: Retrieve Textbook Content from Qdrant Vector Database

**Input**: Design documents from `/specs/001-retrieve-textbook-content/`
**Prerequisites**: plan.md, spec_retrieval.md, research.md, data-model.md, contracts/retrieval_contract.md, quickstart.md

**Tests**: The spec explicitly requests comprehensive testing with at least 5 diverse queries, performance benchmarks, and error scenario tests.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for retrieval functionality

- [x] T001 Verify Python 3.13 and UV package manager are installed and configured
- [x] T002 [P] Verify existing backend structure (embedding.py, storage.py, chunking.py, utils.py from Spec 1)
- [x] T003 [P] Verify Qdrant collection contains embedded content from Spec 1 (542+ chunks)
- [x] T004 Verify .env file contains required credentials (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core retrieval infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 [P] Create data classes for Query entity in backend/retrieve.py (query_text, query_embedding, query_type, created_at)
- [x] T006 [P] Create data classes for RetrievedChunk entity in backend/retrieve.py (chunk_id, similarity_score, content_text, source_file, module_name, section_heading, chunk_index, total_chunks, token_count)
- [x] T007 Create data class for RetrievalResult entity in backend/retrieve.py (query_id, original_query, retrieved_chunks, retrieval_time_ms, total_candidates)
- [x] T008 [P] Create data class for PerformanceMetric entity in backend/retrieve.py (metric_id, query_text, latency_ms, relevance_score, result_count, query_category, timestamp, success)
- [x] T009 Implement base retrieve_content() function in backend/retrieve.py with query embedding and Qdrant search
- [x] T010 Verify storage.py exports query_similar() function for Qdrant vector search
- [x] T011 Verify embedding.py exports generate_embedding() function with search_query input type support

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Successfully retrieve relevant chunks from Qdrant based on sample queries embedded with Cohere

**Independent Test**: Submit query "How do I install ROS 2?" and verify returned chunks have similarity scores >0.7 from ROS2-related modules

### Tests for User Story 1 (Red Phase - Write Tests First)

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T012 [P] [US1] Unit test for Query data validation in backend/tests/test_retrieve.py (test_query_validation_success, test_query_validation_empty_text)
- [x] T013 [P] [US1] Unit test for RetrievedChunk data validation in backend/tests/test_retrieve.py (test_chunk_validation_success, test_chunk_validation_invalid_score)
- [x] T014 [P] [US1] Unit test for retrieve_content() function in backend/tests/test_retrieve.py (test_retrieve_content_success, test_retrieve_content_empty_query)
- [x] T015 [US1] Integration test for ROS2 query in backend/tests/test_integration_retrieve.py (test_ros2_installation_query with expected similarity >0.7)
- [x] T016 [US1] Integration test for Gazebo query in backend/tests/test_integration_retrieve.py (test_gazebo_simulation_query with expected similarity >0.7)

### Implementation for User Story 1 (Green Phase)

- [x] T017 [US1] Implement query text validation in retrieve_content() (check for empty/whitespace, length <1000 chars)
- [x] T018 [US1] Implement query embedding generation with Cohere API in retrieve_content()
- [x] T019 [US1] Implement Qdrant vector search call in retrieve_content() with top_k=5 and score_threshold=0.7
- [x] T020 [US1] Implement conversion of Qdrant results to RetrievedChunk objects in retrieve_content()
- [x] T021 [US1] Implement retrieval time measurement in retrieve_content() (start_time to end_time)
- [x] T022 [US1] Add logging for query processing stages in retrieve_content() (embedding generation, search, results)
- [x] T023 [US1] Run all User Story 1 tests and verify they pass

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Validate Retrieval Pipeline (Priority: P2)

**Goal**: Run comprehensive tests with at least 5 diverse queries covering different book modules to validate the end-to-end pipeline

**Independent Test**: Run validation with 5+ diverse queries (ROS2, Gazebo, VLA, humanoid robots, robotics concepts) and verify 80%+ return relevant results with acceptable latency

### Tests for User Story 2 (Red Phase - Write Tests First)

- [x] T024 [P] [US2] Unit test for validate_retrieval_with_sample_queries() function in backend/tests/test_retrieve.py (test_validation_with_sample_queries)
- [x] T025 [P] [US2] Unit test for PerformanceMetric data validation in backend/tests/test_retrieve.py (test_performance_metric_validation)
- [x] T026 [US2] Integration test for VLA models query in backend/tests/test_integration_retrieve.py (test_vla_models_query)
- [x] T027 [US2] Integration test for humanoid robots query in backend/tests/test_integration_retrieve.py (test_humanoid_robots_query)
- [x] T028 [US2] Integration test for robotics concepts query in backend/tests/test_integration_retrieve.py (test_robotics_concepts_query)
- [x] T029 [US2] Performance benchmark test in backend/tests/test_integration_retrieve.py (test_retrieval_latency_under_2_seconds with 95th percentile check)

### Implementation for User Story 2 (Green Phase)

- [x] T030 [US2] Implement validate_retrieval_with_sample_queries() function in backend/retrieve.py with 5 sample queries
- [x] T031 [US2] Add performance metric collection in validate_retrieval_with_sample_queries() (latency, relevance scores, result counts)
- [x] T032 [US2] Implement success rate calculation in validate_retrieval_with_sample_queries() (count queries with similarity >0.5)
- [x] T033 [US2] Add detailed logging for each validation query result (similarity scores, retrieval times, source files)
- [x] T034 [US2] Implement validation report generation with summary statistics (successful_queries/total_queries, average latency, average relevance)
- [x] T035 [US2] Integrate validation into backend/main.py with run_retrieval_phase() function
- [x] T036 [US2] Add command-line flag to main.py to run retrieval validation independently
- [x] T037 [US2] Run all User Story 2 tests and verify they pass

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Handle Retrieval Errors (Priority: P3)

**Goal**: Handle query errors gracefully and provide meaningful feedback when issues occur during retrieval

**Independent Test**: Submit malformed queries and simulate Qdrant connection failures, verify appropriate error handling and logging

### Tests for User Story 3 (Red Phase - Write Tests First)

- [x] T038 [P] [US3] Unit test for empty query error handling in backend/tests/test_retrieve.py (test_empty_query_error)
- [x] T039 [P] [US3] Unit test for overly long query error handling in backend/tests/test_retrieve.py (test_long_query_error with >1000 chars)
- [x] T040 [P] [US3] Unit test for invalid top_k parameter in backend/tests/test_retrieve.py (test_invalid_top_k_error)
- [x] T041 [US3] Integration test for Cohere API failure handling in backend/tests/test_integration_retrieve.py (test_cohere_api_error with retry logic)
- [x] T042 [US3] Integration test for Qdrant connection failure in backend/tests/test_integration_retrieve.py (test_qdrant_connection_error)
- [x] T043 [US3] Integration test for no results scenario in backend/tests/test_integration_retrieve.py (test_no_relevant_results_found)

### Implementation for User Story 3 (Green Phase)

- [x] T044 [P] [US3] Add input validation for query_text in retrieve_content() (raise ValueError for empty/whitespace)
- [x] T045 [P] [US3] Add input validation for query length in retrieve_content() (raise ValueError if >1000 chars)
- [x] T046 [P] [US3] Add input validation for top_k parameter in retrieve_content() (raise ValueError if <1 or >10)
- [x] T047 [P] [US3] Add input validation for score_threshold parameter in retrieve_content() (raise ValueError if not between 0.0 and 1.0)
- [x] T048 [US3] Implement retry with exponential backoff for Cohere API calls using utils.retry_with_exponential_backoff
- [x] T049 [US3] Implement retry with exponential backoff for Qdrant search calls using utils.retry_with_exponential_backoff
- [x] T050 [US3] Add error logging for embedding generation failures (log error message and query text)
- [x] T051 [US3] Add error logging for Qdrant search failures (log error message and collection name)
- [x] T052 [US3] Handle no results scenario gracefully (return empty retrieved_chunks list instead of error)
- [x] T053 [US3] Add custom exception classes in backend/retrieve.py (QueryValidationError, EmbeddingError, SearchError)
- [x] T054 [US3] Update validate_retrieval_with_sample_queries() to handle and log errors for individual queries
- [x] T055 [US3] Run all User Story 3 tests and verify they pass

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [x] T056 [P] Add comprehensive docstrings to all functions in backend/retrieve.py (Google-style docstrings)
- [x] T057 [P] Add type hints to all function parameters and return values in backend/retrieve.py
- [x] T058 [P] Add inline comments for complex logic in retrieve_content() and validation functions
- [x] T059 Code cleanup and refactoring in backend/retrieve.py (remove duplicate code, improve readability)
- [x] T060 [P] Run pytest for all tests in backend/tests/ and verify 100% pass rate
- [x] T061 [P] Run linting checks (ruff or flake8) on backend/retrieve.py and fix any issues
- [x] T062 Performance optimization: Review and optimize query embedding caching if needed
- [x] T063 Security hardening: Ensure API keys are never logged, sanitize error messages
- [x] T064 Run full quickstart.md validation following all steps in the guide
- [x] T065 Generate validation report with final performance metrics (success rate, average latency, relevance scores)
- [x] T066 Update backend/README.md with retrieval functionality documentation (if README exists)

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 for validation but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Integrates with US1/US2 for error handling but should be independently testable

### Within Each User Story

- Tests (Red Phase) MUST be written and FAIL before implementation (Green Phase)
- Data classes before functions that use them
- Core retrieve_content() before validation functions
- Unit tests before integration tests
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002, T003, T004)
- All Foundational data class tasks marked [P] can run in parallel (T005, T006, T008)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can be written in parallel
- Different user stories can be worked on in parallel by different team members
- All polish tasks marked [P] can run in parallel (T056-T058, T060-T061)

---

## Parallel Example: User Story 1 Tests

```bash
# Launch all User Story 1 tests together (Red Phase):
Task: "Unit test for Query data validation in backend/tests/test_retrieve.py"
Task: "Unit test for RetrievedChunk data validation in backend/tests/test_retrieve.py"
Task: "Unit test for retrieve_content() function in backend/tests/test_retrieve.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Query Textbook Content)
4. **STOP and VALIDATE**: Test User Story 1 independently with sample ROS2 and Gazebo queries
5. Verify retrieval latency <2 seconds and similarity scores >0.7

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Validate with sample queries (MVP!)
3. Add User Story 2 → Test independently → Run full validation with 5+ diverse queries
4. Add User Story 3 → Test independently → Verify error handling with edge cases
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Query functionality)
   - Developer B: User Story 2 (Validation pipeline)
   - Developer C: User Story 3 (Error handling)
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
- All file paths use backend/ directory from existing Spec 1 structure
- Testing follows pytest conventions with test_*.py naming
- Integration tests require actual Qdrant and Cohere connections (use fixtures for mocking if needed)
