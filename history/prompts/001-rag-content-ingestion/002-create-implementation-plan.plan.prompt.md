---
id: 002
title: Create Implementation Plan RAG Ingestion
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-content-ingestion
branch: 001-rag-content-ingestion
user: asimh
command: /sp.plan
labels: ["planning", "architecture", "rag", "embeddings", "cohere", "qdrant", "vercel", "backend"]
links:
  spec: specs/001-rag-content-ingestion/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-content-ingestion/plan.md
  - specs/001-rag-content-ingestion/research.md
  - specs/001-rag-content-ingestion/data-model.md
  - specs/001-rag-content-ingestion/contracts/cohere-api.md
  - specs/001-rag-content-ingestion/contracts/qdrant-api.md
  - specs/001-rag-content-ingestion/quickstart.md
tests:
  - Constitution compliance check (all gates passed)
  - Technical context validation (all unknowns resolved)
  - Cross-artifact consistency (spec → research → data model → plan)
---

## Prompt

Create: backend architecture overview, data flow diagram for ingestion pipeline, setup scripts, and verification steps.

Decisions needing documentation: choices for chunking strategy (e.g., token-based vs. semantic) with tradeoffs on retrieval accuracy vs. efficiency; Cohere model selection (e.g., embed-english-v3.0) based on dimensions and performance.

Testing strategy: unit tests for chunking and embedding functions; end-to-end validation by ingesting sample docs, querying Qdrant, and confirming vector storage; deployment check via accessing Vercel URL.

Technical details:

- Initial project setup: Create 'backend' folder in project root and initialize UV package manager with 'uv init' for dependency management.

- In backend, create main.py containing all functionalities: function to deploy frontend to Vercel and retrieve the live URL; function to get Markdown file paths from frontend/docs/; chunking function (split into 500-1000 token segments); embedding function using Cohere API; storage function to upsert vectors to Qdrant collection.

- Organize by phases: Setup → Deployment → Ingestion (chunk + embed + store) → Verification

- Main function: Orchestrate the pipeline by calling the above functions sequentially, handling API keys via environment variables, and logging progress.

## Response snapshot

Created comprehensive implementation plan with Phase 0 (Research) and Phase 1 (Design & Contracts) complete.

**Phase 0 - Research (research.md)**:
- **Embedding model**: Cohere embed-english-v3.0 (1024 dims) - best balance of quality, cost, free tier
- **Chunking strategy**: Semantic token-based (500-1000 tokens, 100-token overlap) - preserves context, respects API limits
- **Vector DB**: Qdrant Cloud Free Tier (cosine similarity, HNSW indexing) - sufficient for ~350 chunks
- **Deployment**: Vercel CLI for speed (GitHub Actions deferred to future)
- **Tooling**: UV package manager (10-100x faster than pip)
- **Error handling**: Exponential backoff with jitter, 3 max retries
- **Testing**: Unit + integration + deployment verification
- **Logging**: Structured JSON format for observability

**Phase 1 - Design & Contracts (data-model.md, contracts/, quickstart.md)**:
- **Data model**: 5 entities defined with validation rules, state transitions, relationships
  - MarkdownFile → DocumentChunk → EmbeddingVector → QdrantPoint → ProcessingJob
- **API contracts**:
  - Cohere API: Request/response schemas, rate limits (100 calls/min), batch processing (96 chunks)
  - Qdrant API: Collection config (1024-dim, cosine, HNSW), upsert/search operations
- **Quickstart guide**: Setup instructions, verification steps, troubleshooting for common issues

**Implementation Plan (plan.md)**:
- **Technical context**: Python 3.11+, 6 core dependencies, Qdrant storage, pytest testing
- **Architecture**: 6-phase pipeline (deployment → discovery → chunking → embedding → storage → verification)
- **Project structure**: Backend/ with 5 modules (main.py, chunking.py, embedding.py, storage.py, deployment.py, utils.py) + tests/
- **Performance**: Estimated ~100 seconds total (90s deployment, 10s ingestion of 350 chunks)
- **Constitution compliance**: All gates passed, documented Vercel variance (user-approved deviation from GitHub Pages)
- **Testing strategy**: Unit tests (mocked APIs), integration tests (real APIs), acceptance tests (manual verification)
- **Error handling**: Retryable (429, 500, 503) vs non-retryable (401, 400), partial failure handling
- **Security**: Environment variables (.env), secret redaction in logs, key rotation guidance

**Key Architectural Decisions**:
1. **Batch processing**: 96 chunks/batch for Cohere (API limit), 100 vectors/batch for Qdrant (optimal)
2. **Modular design**: 5 independent modules, each unit-testable, orchestrated by main.py
3. **Sequential pipeline**: No parallelization in v1 (complexity vs benefit tradeoff)
4. **Metadata preservation**: 8 fields per chunk for source attribution and filtering
5. **Verification built-in**: Sample semantic search query at end of pipeline confirms success

## Outcome

- ✅ Impact: Complete implementation plan with research, data model, API contracts, and quickstart guide. Ready for /sp.tasks to generate atomic implementation tasks.
- 🧪 Tests: Testing strategy defined for 3 levels (unit, integration, acceptance). Mock fixtures and sample data identified.
- 📁 Files: Created 6 planning artifacts (468-line plan.md, research.md, data-model.md, 2 API contracts, quickstart.md)
- 🔁 Next prompts: /sp.tasks to generate phase-based tasks, /sp.adr if chunking strategy or embedding model warrants ADR documentation
- 🧠 Reflection: Plan successfully balances technical detail with clarity. Constitution compliance documented with justified Vercel variance. All Phase 0/1 unknowns resolved. Module boundaries enable independent development and testing.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow followed template structure correctly
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates, documented variance approved by user input)
- Prompt variant (if applicable): N/A - first planning phase for this feature
- Next experiment (smallest change to try): During tasks generation, consider whether chunking overlap strategy (100 tokens) warrants ADR for future reference when extending to other content types
