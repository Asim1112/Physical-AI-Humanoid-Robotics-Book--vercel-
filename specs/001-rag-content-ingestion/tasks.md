# Tasks: RAG Content Ingestion and Embedding System

**Input**: Design documents from `/specs/001-rag-content-ingestion/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Organization**: Tasks organized by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3)
- File paths based on backend/ structure from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize backend project and dependencies

- [x] T001 Create backend/ directory in repository root
- [x] T002 Initialize UV project in backend/ with `uv init`
- [x] T003 Add production dependencies: cohere>=5.0.0, qdrant-client>=1.7.0, tiktoken>=0.5.0, python-dotenv>=1.0.0, markdown-it-py>=3.0.0, requests>=2.31.0
- [x] T004 Add dev dependencies: pytest>=7.4.0, pytest-cov>=4.1.0, pytest-mock>=3.12.0
- [x] T005 [P] Create backend/.gitignore with .env, __pycache__, *.pyc, .pytest_cache, uv.lock
- [x] T006 [P] Create backend/.env.example template with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME, FRONTEND_PATH, DOCS_PATH
- [x] T007 [P] Create backend/tests/fixtures/ directory
- [x] T008 [P] Copy sample markdown file from frontend/docs/ to backend/tests/fixtures/sample.md
- [x] T009 Update root .gitignore to include backend/.env

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core utilities needed by all user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T010 Create backend/utils.py with structured logging setup (JSON format, console + file handlers)
- [x] T011 Add environment variable loading function in backend/utils.py using python-dotenv
- [x] T012 Add exponential backoff retry decorator in backend/utils.py (max_retries=3, base_delay=1s, jitter=True)
- [x] T013 Add API key validation function in backend/utils.py (check required env vars on startup)
- [x] T014 Create backend/tests/conftest.py with pytest fixtures for mocked Cohere and Qdrant clients

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Deploy Live Documentation Site (Priority: P1) 🎯 MVP

**Goal**: Deploy Docusaurus frontend to Vercel and verify accessibility

**Independent Test**: Navigate to returned Vercel URL, verify homepage loads with HTTP 200, sidebar navigation works, and Docusaurus search returns results

### Implementation for User Story 1

- [x] T015 [P] [US1] Create backend/deployment.py with deploy_to_vercel() function using Vercel CLI subprocess
- [x] T016 [P] [US1] Add verify_deployment() function in backend/deployment.py to check URL accessibility via HTTP request
- [x] T017 [US1] Create backend/main.py with deployment phase orchestration calling deployment.py functions
- [x] T018 [US1] Add deployment logging in main.py (deployment started, URL captured, verification result)
- [x] T019 [US1] Handle deployment errors in main.py (CLI not found, build failure, authentication error)

**Checkpoint**: Vercel deployment functional - can deploy frontend and verify URL independently

---

## Phase 4: User Story 2 - Generate Semantic Embeddings (Priority: P2)

**Goal**: Chunk markdown files and generate Cohere embeddings with metadata

**Independent Test**: Run pipeline on sample.md, verify chunks are 150-300 tokens, embeddings are 1024-dim, and metadata includes filename/section/chunk_index (UPDATED: reduced token range for Cohere API compatibility)

### Implementation for User Story 2

- [x] T020 [P] [US2] Create backend/chunking.py with discover_markdown_files() function (glob frontend/docs/**/*.md)
- [x] T021 [P] [US2] Add parse_markdown() function in backend/chunking.py (extract frontmatter, section headings, content)
- [x] T022 [P] [US2] Add chunk_text() function in backend/chunking.py (150-300 tokens, tiktoken counting, semantic boundaries, 30-token overlap) - FIXED for Cohere compatibility
- [x] T023 [P] [US2] Add extract_metadata() function in backend/chunking.py (module_name from path, section_heading, chunk_index)
- [x] T024 [US2] Create DocumentChunk dataclass in backend/chunking.py per data-model.md (chunk_id, source_file, module_name, section_heading, chunk_index, total_chunks, text, token_count, overlap_text, created_at)
- [x] T025 [P] [US2] Create backend/embedding.py with generate_embedding() function calling Cohere API (embed-english-v3.0, input_type=search_document)
- [x] T026 [P] [US2] Add batch_embed() function in backend/embedding.py (batch size=96, handle rate limits with retry decorator)
- [x] T027 [US2] Create EmbeddingVector dataclass in backend/embedding.py per data-model.md (vector_id, chunk_id, vector, model_name, input_type, created_at)
- [x] T028 [US2] Add chunking and embedding phases to backend/main.py (call discover → parse → chunk → embed)
- [x] T029 [US2] Add embedding pipeline logging in main.py (files discovered, chunks created, batches processed, API calls made)
- [x] T030 [US2] Handle embedding errors in main.py (invalid API key, rate limit exceeded, dimension mismatch)

**Checkpoint**: Chunking and embedding functional - can process markdown files and generate 1024-dim embeddings independently

---

## Phase 5: User Story 3 - Store and Query Embeddings (Priority: P3)

**Goal**: Store embeddings in Qdrant and verify retrieval via semantic search

**Independent Test**: Insert sample embeddings, query with test embedding "ROS 2 introduction", verify top-5 results have scores >0.7 and correct metadata

### Implementation for User Story 3

- [x] T031 [P] [US3] Create backend/storage.py with setup_qdrant_collection() function (1024-dim, cosine distance, HNSW config per research.md)
- [x] T032 [P] [US3] Add upsert_vectors() function in backend/storage.py (batch size=100, payload schema per data-model.md)
- [x] T033 [P] [US3] Add query_similar() function in backend/storage.py (search with embedding, return top-K with metadata, score_threshold=0.7) - FIXED for new Qdrant API (query_points)
- [x] T034 [US3] Create QdrantPoint dataclass in backend/storage.py per data-model.md (point_id, vector, payload with all metadata fields)
- [x] T035 [US3] Add storage and verification phases to backend/main.py (call setup_collection → upsert → query_sample)
- [x] T036 [US3] Add storage pipeline logging in main.py (collection created, vectors uploaded, batches completed, query results)
- [x] T037 [US3] Handle storage errors in main.py (Qdrant connection failure, collection already exists, storage quota exceeded)
- [x] T038 [US3] Create ProcessingJob dataclass in backend/main.py to track pipeline execution (job_id, files_discovered, files_processed, files_failed, total_chunks, total_embeddings, total_vectors_stored, started_at, completed_at, status, errors)
- [x] T039 [US3] Add final report generation in main.py (print summary: deployment URL, files processed, chunks created, embeddings generated, vectors stored, top query result)

**Checkpoint**: All user stories functional - complete end-to-end pipeline from deployment to verified vector search

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Validation, documentation, and production readiness

- [ ] T040 [P] Create backend/tests/test_chunking.py with unit tests (token counting accuracy, boundary splitting, overlap logic, frontmatter removal, code block preservation)
- [ ] T041 [P] Create backend/tests/test_embedding.py with mocked Cohere tests (successful embedding, rate limit retry, invalid API key handling, batch processing)
- [ ] T042 [P] Create backend/tests/test_storage.py with mocked Qdrant tests (collection creation, batch upsert, query results, error handling)
- [ ] T043 [P] Create backend/tests/test_deployment.py with mocked Vercel CLI tests (successful deploy, build failure, URL extraction)
- [ ] T044 Create backend/tests/test_integration.py with end-to-end test (process sample.md, verify chunks/embeddings/storage, query and validate results)
- [x] T045 [P] Add validation in main.py for token count ranges (150-300 acceptable, warn if outside) - UPDATED for Cohere compatibility
- [x] T046 [P] Add edge case handling in chunking.py (empty files, frontmatter-only files, large code blocks)
- [ ] T047 Update root README.md with backend setup instructions and quickstart reference
- [x] T048 Run full pipeline on all 43 markdown files and validate SC-001 through SC-008 from spec.md - COMPLETED: 542 chunks embedded, 542 vectors stored, 100% coverage

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phases 3-5)**: All depend on Foundational completion
  - US1 (Deployment): Independent, can start first
  - US2 (Embeddings): Independent of US1, parallel-ready
  - US3 (Storage): Independent but logically follows US2 (needs embeddings to store)
- **Polish (Phase 6)**: Depends on all user stories complete

### User Story Dependencies

- **User Story 1 (P1)**: Independent - can test deployment alone
- **User Story 2 (P2)**: Independent - can test chunking/embedding with fixtures
- **User Story 3 (P3)**: Integrates with US2 (needs embeddings) but tests storage independently with mocked vectors

### Within Each User Story

- Dataclasses before functions using them
- Core functions before main.py integration
- Logging after core implementation
- Error handling last within story

### Parallel Opportunities

**Phase 1 Setup** (5 tasks can run in parallel):
- T005 (gitignore), T006 (.env.example), T007 (fixtures dir), T008 (sample.md copy) in parallel

**Phase 2 Foundational** (T010-T012 in parallel after T014):
- T010 (logging), T011 (env loading), T012 (retry decorator) in parallel

**User Story 1** (T015-T016 in parallel):
- T015 (deploy function), T016 (verify function) in parallel

**User Story 2** (10 tasks, 7 parallelizable):
- T020-T024 (chunking functions + dataclass) in parallel
- T025-T027 (embedding functions + dataclass) in parallel after chunking
- T028-T030 sequentially (main.py integration)

**User Story 3** (T031-T034 in parallel):
- T031 (setup collection), T032 (upsert), T033 (query), T034 (dataclass) in parallel

**Phase 6 Polish** (T040-T044, T045-T046 in parallel):
- All test files (T040-T044) in parallel
- Validation tasks (T045-T046) in parallel

---

## Parallel Example: User Story 2

```bash
# Launch all chunking components together:
Task: "Create backend/chunking.py discover_markdown_files()"
Task: "Add parse_markdown() in backend/chunking.py"
Task: "Add chunk_text() in backend/chunking.py"
Task: "Add extract_metadata() in backend/chunking.py"
Task: "Create DocumentChunk dataclass in backend/chunking.py"

# Then launch all embedding components together:
Task: "Create backend/embedding.py generate_embedding()"
Task: "Add batch_embed() in backend/embedding.py"
Task: "Create EmbeddingVector dataclass in backend/embedding.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T014) - CRITICAL
3. Complete Phase 3: US1 Deployment (T015-T019)
4. **STOP and VALIDATE**: Deploy to Vercel, verify URL loads
5. Demo ready: Live textbook accessible

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 → Test deployment → Deploy/Demo (MVP!)
3. Add US2 → Test chunking/embedding on sample → Validate dimensions
4. Add US3 → Test storage/query → Deploy/Demo (Full pipeline!)
5. Add Polish → Production-ready with tests and validation

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational done:
   - Developer A: US1 (Deployment)
   - Developer B: US2 (Chunking + Embeddings)
   - Developer C: US3 (Storage + Query)
3. Integrate in main.py sequentially: deployment → chunking → embedding → storage

---

## Summary

**Total Tasks**: 48 tasks
**User Story Breakdown**:
- Setup: 9 tasks
- Foundational: 5 tasks (BLOCKS all stories)
- US1 (Deployment): 5 tasks
- US2 (Embeddings): 11 tasks
- US3 (Storage): 9 tasks
- Polish: 9 tasks

**Parallel Opportunities**: 23 tasks marked [P] can run in parallel within their phase

**Independent Test Criteria**:
- US1: Vercel URL returns HTTP 200, homepage loads, navigation works
- US2: Sample file chunked (400-1100 tokens), embeddings 1024-dim, metadata preserved
- US3: Sample vectors stored, query returns top-5 with scores >0.7, metadata correct

**MVP Scope**: Phases 1-3 (US1 only) = 19 tasks → Deployed textbook site

**Full Feature**: All phases = 48 tasks → Complete RAG ingestion pipeline

---

## Notes

- Tasks follow strict `- [ ] [ID] [P?] [Story] Description with file path` format
- Each user story independently testable at checkpoint
- Foundational phase completion gates all user story work
- [P] tasks enable parallel execution for team efficiency
- Stop at any checkpoint to validate story independence
- Reference quickstart.md for environment setup and verification commands
