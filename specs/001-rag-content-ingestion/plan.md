# Implementation Plan: RAG Content Ingestion and Embedding System

**Branch**: `001-rag-content-ingestion` | **Date**: 2025-12-25 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-rag-content-ingestion/spec.md`

## Summary

Build a RAG content ingestion pipeline that deploys the Docusaurus textbook to Vercel, generates semantic embeddings from all markdown content using Cohere embed-english-v3.0 model, and stores them in Qdrant Cloud vector database. The pipeline chunks markdown files into 500-1000 token segments, preserves metadata for source attribution, and enables semantic search for future RAG query capabilities.

**Technical Approach**: Python backend (UV package manager) with sequential pipeline: (1) Vercel deployment via CLI, (2) markdown file discovery and parsing, (3) semantic token-based chunking with overlap, (4) batch embedding generation via Cohere API, (5) batch vector upload to Qdrant, (6) verification via sample semantic search query.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere>=5.0.0, qdrant-client>=1.7.0, tiktoken>=0.5.0, python-dotenv>=1.0.0, markdown-it-py>=3.0.0, requests>=2.31.0
**Storage**: Qdrant Cloud (vector database) - Free Tier (1M vectors, 1GB RAM)
**Testing**: pytest>=7.4.0, pytest-cov>=4.1.0
**Target Platform**: Cross-platform (Windows/macOS/Linux) - local execution, cloud services (Cohere API, Qdrant Cloud, Vercel)
**Project Type**: Web application (backend ingestion pipeline + frontend deployment)
**Performance Goals**:
- Ingestion: ~350 chunks in <10 seconds (embedding + upload)
- Deployment: Vercel build + deploy <2 minutes
- Search latency: <50ms p95 for semantic queries (Qdrant)

**Constraints**:
- Cohere free tier: 100 API calls/min (batching required)
- Qdrant free tier: 1M vectors, 1GB RAM (sufficient for ~10K chunks)
- No modification of source files in frontend/docs/
- Token chunks: 400-1100 tokens (Cohere max 512 tokens/embedding)

**Scale/Scope**:
- ~50 markdown files in frontend/docs/
- Estimated ~350 chunks (avg 7 chunks/file at ~700 tokens)
- ~1.4MB vector storage (1024-dim × 350 vectors × 4 bytes)
- Single-run pipeline (no continuous updates in this spec)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Assessment

✅ **Structural Separation (Principle VI, Structural Rules)**
- Backend code in `/backend` directory (not `/frontend`)
- Frontend remains in `/frontend` (Docusaurus site unchanged)
- No Docusaurus files created outside `/frontend`
- Clear boundary: backend reads from frontend/docs/, does not modify

✅ **Spec-First Execution (Principle VI)**
- Specification created (spec.md) before planning
- Planning document (this file) created before implementation
- Research.md documents all technical decisions
- No implementation until tasks.md generated

✅ **Source-Aware Content (Principle III)**
- Cohere API: Official documentation referenced in contracts/cohere-api.md
- Qdrant API: Official documentation referenced in contracts/qdrant-api.md
- Vercel deployment: Official Docusaurus deployment guide
- No hallucinated API configurations

✅ **Modularity (Principle IV)**
- Pipeline organized into discrete modules: chunking.py, embedding.py, storage.py, deployment.py
- Each module testable independently
- Main.py orchestrates without tight coupling

✅ **Quality Gates (Deployment & Quality Gates section)**
- Testing strategy defined (unit + integration + deployment verification)
- Logging at each pipeline stage
- Verification step (sample query) confirms successful ingestion

### Documented Variance

⚠️ **Deployment Target Deviation**

| Constitution Requirement | Implementation | Justification |
|-------------------------|----------------|---------------|
| Deployment target: GitHub Pages | Vercel (via CLI) | User specified Vercel for faster hackathon iteration. Vercel provides instant preview deploys, better DX for demos. GitHub Pages remains viable post-hackathon. |

**Migration Path**: Vercel and GitHub Pages can coexist. Post-hackathon, add GitHub Actions workflow for GitHub Pages deployment. Vercel deployment is non-destructive and does not prevent future GitHub Pages setup.

**Approved By**: User input explicitly requested Vercel deployment.

### Re-Check Post-Design (Phase 1 Complete)

✅ **All gates pass** - No additional violations introduced during design phase.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-content-ingestion/
├── spec.md                  # Feature specification
├── plan.md                  # This file (implementation plan)
├── research.md              # Phase 0: Technical decisions and rationale
├── data-model.md            # Phase 1: Entity definitions and relationships
├── quickstart.md            # Phase 1: Setup and usage guide
├── contracts/
│   ├── cohere-api.md        # Cohere API contract and examples
│   └── qdrant-api.md        # Qdrant API contract and examples
├── checklists/
│   └── requirements.md      # Specification quality checklist
└── tasks.md                 # Phase 2: Generated by /sp.tasks (NOT YET CREATED)
```

### Source Code (repository root)

```text
backend/
├── .env                     # Environment variables (not committed)
├── .gitignore               # Ignore .env and cache files
├── pyproject.toml           # UV project configuration
├── uv.lock                  # Locked dependencies
├── main.py                  # Pipeline orchestration entry point
├── chunking.py              # Markdown chunking logic
├── embedding.py             # Cohere API integration
├── storage.py               # Qdrant operations (collection setup, upsert, query)
├── deployment.py            # Vercel deployment functions
├── utils.py                 # Shared utilities (logging, retry logic)
└── tests/
    ├── test_chunking.py     # Unit tests for chunking
    ├── test_embedding.py    # Unit tests for embedding (mocked Cohere)
    ├── test_storage.py      # Unit tests for storage (mocked Qdrant)
    ├── test_deployment.py   # Unit tests for deployment functions
    ├── test_integration.py  # End-to-end integration test
    ├── fixtures/
    │   └── sample.md        # Test markdown file
    └── conftest.py          # Pytest fixtures and config

frontend/
├── docs/                    # Textbook markdown content (source)
├── docusaurus.config.js     # Docusaurus configuration
├── package.json             # Node dependencies
└── ...                      # Other Docusaurus files

.github/
└── workflows/
    └── deploy.yml           # Future: GitHub Actions for automated deployment (out of scope)

.gitignore                   # Includes backend/.env
README.md                    # Updated with backend setup instructions
```

**Structure Decision**: Web application structure selected due to distinct frontend (Docusaurus static site) and backend (Python ingestion pipeline). Backend operates independently, reading from frontend/docs/ without modification. This aligns with constitution's structural separation requirements.

## Complexity Tracking

> No constitution violations requiring justification.

## Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────────────┐
│                     RAG Ingestion Pipeline                      │
└─────────────────────────────────────────────────────────────────┘

1. Deployment Phase
   ┌───────────┐
   │ Vercel CLI│──> Deploy frontend/ ──> Live URL
   └───────────┘

2. Discovery Phase
   ┌──────────────┐
   │ File Scanner │──> frontend/docs/**/*.md ──> List[MarkdownFile]
   └──────────────┘

3. Chunking Phase
   ┌──────────────┐
   │ Markdown     │──> Parse & Chunk ──> List[DocumentChunk]
   │ Parser       │    (500-1000 tokens, 100-token overlap)
   └──────────────┘

4. Embedding Phase
   ┌──────────────┐
   │ Cohere API   │──> Batch Embed ──> List[EmbeddingVector]
   │ Client       │    (96 chunks/batch, 1024-dim)
   └──────────────┘

5. Storage Phase
   ┌──────────────┐
   │ Qdrant Client│──> Batch Upsert ──> Qdrant Collection
   │              │    (100 vectors/batch)
   └──────────────┘

6. Verification Phase
   ┌──────────────┐
   │ Sample Query │──> Semantic Search ──> Top-K Results
   │              │    (query embedding + Qdrant search)
   └──────────────┘
```

### Data Flow

```
MarkdownFile ──> DocumentChunk ──> EmbeddingVector ──> QdrantPoint
     (raw)        (text+metadata)    (1024-dim vector)  (stored w/ payload)
```

See [data-model.md](data-model.md) for entity definitions.

### Module Responsibilities

| Module | Responsibilities | Inputs | Outputs |
|--------|-----------------|--------|---------|
| `deployment.py` | Vercel CLI integration, deployment verification | frontend/ path | Deployment URL |
| `chunking.py` | Markdown parsing, semantic chunking, metadata extraction | List[file paths] | List[DocumentChunk] |
| `embedding.py` | Cohere API calls, batch processing, retry logic | List[DocumentChunk] | List[EmbeddingVector] |
| `storage.py` | Qdrant collection setup, vector upsert, search queries | List[EmbeddingVector] | Success status, search results |
| `utils.py` | Logging, retry logic, environment variable loading | Config | N/A |
| `main.py` | Orchestration, error handling, reporting | CLI args/env vars | Exit code, summary report |

## Implementation Phases

### Phase 0: Research ✅ COMPLETE

**Output**: [research.md](research.md)

**Decisions Documented**:
1. Embedding model: Cohere embed-english-v3.0 (1024-dim)
2. Chunking strategy: Semantic token-based (500-1000 tokens, 100-token overlap)
3. Vector DB config: Qdrant Cloud (cosine similarity, HNSW indexing)
4. Deployment: Vercel CLI (manual initial, GitHub Actions future)
5. Tooling: UV for Python dependencies, structured JSON logging
6. Error handling: Exponential backoff with 3 max retries
7. Testing: Unit + integration + deployment verification

### Phase 1: Design & Contracts ✅ COMPLETE

**Output**: [data-model.md](data-model.md), [contracts/](contracts/), [quickstart.md](quickstart.md)

**Artifacts Created**:
- **Data Model**: 5 entities (MarkdownFile, DocumentChunk, EmbeddingVector, QdrantPoint, ProcessingJob)
- **API Contracts**:
  - [cohere-api.md](contracts/cohere-api.md): Request/response schemas, error handling, rate limits
  - [qdrant-api.md](contracts/qdrant-api.md): Collection config, upsert/search operations, testing
- **Quickstart**: Setup instructions, verification steps, troubleshooting

### Phase 2: Task Breakdown ⏳ PENDING

**Command**: `/sp.tasks` (NOT part of /sp.plan output)

**Expected Output**: tasks.md with phase-based implementation tasks:
- Phase 1 (Red): Setup backend, environment, dependencies
- Phase 2 (Red): Implement chunking module + tests
- Phase 3 (Red): Implement embedding module + tests
- Phase 4 (Red): Implement storage module + tests
- Phase 5 (Red): Implement deployment module + tests
- Phase 6 (Green): Integration testing
- Phase 7 (Green): End-to-end verification
- Phase 8 (Refactor): Logging, error handling improvements

## Testing Strategy

### Unit Tests (pytest)

| Module | Test Coverage | Key Test Cases |
|--------|---------------|----------------|
| `chunking.py` | Token counting, boundary splitting, overlap logic | Valid chunks (500-1000 tokens), final chunk (<400 allowed), code block preservation, frontmatter removal |
| `embedding.py` | API calls (mocked), batch processing, error handling | Successful embedding, rate limit retry, invalid API key, dimension validation (1024) |
| `storage.py` | Collection setup, upsert, search (mocked Qdrant) | Collection creation, batch upsert, query results, error handling (404, 503) |
| `deployment.py` | Vercel CLI calls (mocked), URL extraction | Successful deploy, build failure, authentication error |

### Integration Tests

| Test | Scenario | Validation |
|------|----------|------------|
| End-to-end pipeline | Ingest sample file (module-1-ros2/intro.md) | Chunks created, embeddings generated, vectors stored, search returns results |
| API integration | Real Cohere + Qdrant calls (requires env vars) | Embedding dimensions, Qdrant search scores, metadata preservation |
| Deployment verification | Deploy frontend, verify URL | HTTP 200, homepage content present |

### Acceptance Tests (Manual)

1. **Deployment smoke test**: Navigate to Vercel URL, verify textbook loads
2. **Embedding quality check**: Query "ROS 2 introduction", top result should be module-1-ros2/intro.md with score >0.8
3. **Storage confirmation**: Qdrant dashboard shows ~350 vectors in collection
4. **Error handling**: Simulate rate limit (burst requests), verify exponential backoff logs

### Test Data

- **Sample file**: `frontend/docs/module-1-ros2/intro.md` (~8500 tokens, ~12 expected chunks)
- **Edge cases**: Empty file, frontmatter-only file, file with large code block (>1000 tokens)
- **Mock responses**: See `tests/fixtures/` for mock API responses

## Error Handling & Retry Logic

### Retryable Errors (Exponential Backoff)

| Service | Error | Retry Strategy | Max Retries |
|---------|-------|----------------|-------------|
| Cohere | 429 (rate limit) | Exponential backoff (1s, 2s, 4s) | 3 |
| Cohere | 500 (internal error) | Exponential backoff | 3 |
| Qdrant | 503 (unavailable) | Exponential backoff | 3 |
| Vercel | Deployment timeout | Exponential backoff | 3 |

### Non-Retryable Errors (Immediate Failure)

| Service | Error | Action |
|---------|-------|--------|
| Cohere | 401 (invalid API key) | Log error, exit with code 1 |
| Cohere | 400 (malformed request) | Log chunk details, skip chunk, continue |
| Qdrant | 401 (unauthorized) | Log error, exit with code 1 |
| Qdrant | 400 (dimension mismatch) | Log error, exit with code 1 (indicates bug) |

### Partial Failure Handling

- **Chunking failure**: Log file path, continue to next file
- **Embedding failure (single chunk)**: Retry 3 times, then log and skip
- **Storage failure (batch)**: Retry batch 3 times, then log failed IDs and continue
- **Final report**: Log summary of successes/failures with file paths

## Logging Strategy

### Log Levels

- **DEBUG**: Token counts, chunk boundaries, API request payloads
- **INFO**: Pipeline progress, file processing, API call counts
- **WARNING**: Retry attempts, skipped files (empty/invalid)
- **ERROR**: API failures after retries, storage errors

### Structured Logging (JSON)

```json
{
  "timestamp": "2025-12-25T10:00:00Z",
  "level": "INFO",
  "stage": "embedding",
  "message": "Generated embeddings for batch",
  "metadata": {
    "batch_num": 2,
    "chunks_processed": 96,
    "api_calls": 1,
    "duration_sec": 0.8
  }
}
```

### Log Output

- **Console**: Human-readable format (colorized if terminal supports)
- **File**: JSON format in `backend/logs/ingestion-{timestamp}.log`
- **Retention**: Keep last 10 log files, auto-rotate

## Performance Considerations

### Estimated Execution Time

| Phase | Operation | Time Estimate |
|-------|-----------|---------------|
| Deployment | Vercel build + deploy | ~90 seconds |
| Discovery | Scan 50 markdown files | <1 second |
| Chunking | Parse 50 files → 350 chunks | ~2 seconds |
| Embedding | 4 batches × 96 chunks | ~3 seconds (+ rate limit buffer) |
| Storage | 4 batches × 100 vectors | ~2 seconds |
| Verification | Sample query | <0.1 second |
| **Total** | | **~100 seconds (~1.5 minutes)** |

### Optimization Opportunities (Future)

- **Parallel processing**: Chunk files concurrently (currently sequential)
- **Caching**: Skip re-embedding unchanged files (requires file hash tracking)
- **Batch size tuning**: Optimize Cohere batch size (currently 96, could test 50-96 range)
- **Incremental updates**: Only process new/modified files (requires change detection)

## Security & Secrets Management

### Environment Variables (.env)

```bash
COHERE_API_KEY=<secret>
QDRANT_URL=<cluster-url>
QDRANT_API_KEY=<secret>
VERCEL_TOKEN=<secret>  # Optional (CLI prompts if missing)
```

**Storage**: Backend/.env (git-ignored)
**Access**: `python-dotenv` loads at runtime
**Validation**: Check all required vars on startup, fail fast if missing

### Secret Handling Rules

1. **Never log secrets**: Redact API keys in logs (show first 6 chars only)
2. **Never commit secrets**: .gitignore includes backend/.env
3. **Rotate regularly**: Especially after public demos/hackathon
4. **Use environment-specific keys**: Separate keys for dev/test/prod

## Dependencies

### Production Dependencies

```toml
[project.dependencies]
cohere = ">=5.0.0"              # Cohere SDK
qdrant-client = ">=1.7.0"       # Qdrant Python client
tiktoken = ">=0.5.0"            # Token counting (GPT tokenizer)
python-dotenv = ">=1.0.0"       # Environment variable management
markdown-it-py = ">=3.0.0"      # Markdown parsing
requests = ">=2.31.0"           # HTTP for Vercel API checks
```

### Development Dependencies

```toml
[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",            # Testing framework
    "pytest-cov>=4.1.0",        # Coverage reporting
    "pytest-mock>=3.12.0",      # Mocking utilities
    "black>=23.0.0",            # Code formatting
    "ruff>=0.1.0",              # Linting
]
```

### External Services

- **Cohere API**: Free tier (100 calls/min, 100K/month)
- **Qdrant Cloud**: Free tier (1M vectors, 1GB RAM)
- **Vercel**: Free tier (100GB bandwidth/month, unlimited deploys)

## Deployment & Verification Checklist

### Pre-Deployment

- [ ] `.env` file configured with all API keys
- [ ] Frontend builds successfully (`cd frontend && npm run build`)
- [ ] Vercel CLI authenticated (`vercel login`)
- [ ] Cohere API key validated (test embed call)
- [ ] Qdrant cluster accessible (test connection)

### Post-Deployment

- [ ] Vercel URL returns HTTP 200
- [ ] Homepage content loads correctly
- [ ] Docusaurus search functional
- [ ] Qdrant collection contains ~350 vectors
- [ ] Sample query returns relevant results (score >0.7)
- [ ] Logs show no errors or warnings

### Acceptance Criteria (from spec.md)

- [x] SC-001: Docusaurus site deployed to Vercel
- [ ] SC-002: All markdown files processed (100%)
- [ ] SC-003: Chunks 400-1100 tokens (except final chunks ≥100)
- [ ] SC-004: Qdrant collection created (1024-dim, cosine)
- [ ] SC-005: Verification query top result >0.8 similarity
- [ ] SC-006: Embedding generation completes in reasonable time
- [ ] SC-007: No source files modified
- [ ] SC-008: Logs capture all stages with timestamps

## Next Steps

1. **Generate tasks** (`/sp.tasks`): Break down implementation into atomic, testable tasks
2. **Implement backend modules**: Follow TDD approach (test → implement → refactor)
3. **Integration testing**: Validate end-to-end pipeline with sample data
4. **Production run**: Ingest all ~50 markdown files
5. **Document results**: Capture deployment URL, sample queries, performance metrics
6. **Proceed to Spec 2**: RAG query pipeline for chatbot integration

## References

- **Specification**: [spec.md](spec.md)
- **Research**: [research.md](research.md)
- **Data Model**: [data-model.md](data-model.md)
- **API Contracts**: [contracts/cohere-api.md](contracts/cohere-api.md), [contracts/qdrant-api.md](contracts/qdrant-api.md)
- **Setup Guide**: [quickstart.md](quickstart.md)
- **Constitution**: [.specify/memory/constitution.md](../../.specify/memory/constitution.md)
