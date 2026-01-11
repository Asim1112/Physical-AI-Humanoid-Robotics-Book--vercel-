# Research: RAG Content Ingestion and Embedding System

**Feature**: 001-rag-content-ingestion
**Date**: 2025-12-25
**Purpose**: Resolve technical unknowns and document key architectural decisions

## 1. Embedding Model Selection

### Decision
Use **Cohere embed-english-v3.0** model for generating embeddings.

### Rationale
- **Vector dimensions**: 1024 dimensions, optimal balance between semantic richness and storage efficiency
- **Free tier availability**: Cohere provides generous free tier with 100 API calls/minute
- **Performance**: State-of-the-art retrieval performance on English text, well-suited for technical documentation
- **API simplicity**: Clean SDK with straightforward integration, minimal boilerplate
- **Compatibility**: Widely supported by vector databases including Qdrant

### Alternatives Considered
- **Cohere embed-english-light-v3.0**: 384 dimensions, faster but lower semantic quality - rejected due to reduced retrieval accuracy for technical content
- **OpenAI text-embedding-3-small**: 1536 dimensions, higher cost on free tier, more complex rate limiting - rejected due to cost constraints
- **Sentence-Transformers (self-hosted)**: No API costs but requires infrastructure - rejected as out of scope for hackathon timeline

### Technical Specifications
- Model ID: `embed-english-v3.0`
- Vector size: 1024 dimensions
- Input type: `search_document` for content, `search_query` for retrieval queries
- Max tokens per request: 512 tokens (chunking required for longer content)
- Rate limits: 100 calls/min on free tier

## 2. Chunking Strategy

### Decision
Use **semantic token-based chunking** with 500-1000 token segments and 100-token overlap.

### Rationale
- **Retrieval precision**: Smaller chunks (500-1000 tokens) increase likelihood of precise matches between queries and content
- **Context preservation**: 100-token overlap ensures concepts spanning chunk boundaries remain retrievable
- **API compatibility**: Respects Cohere's 512-token max per embedding call
- **Semantic coherence**: Split at sentence/paragraph boundaries within token range to avoid mid-sentence cuts
- **Computational efficiency**: Balances embedding API costs with retrieval quality

### Chunking Algorithm
1. Parse markdown to extract structure (headings, paragraphs, code blocks)
2. Count tokens using `tiktoken` with cl100k_base encoding (approximates Cohere tokenization)
3. Accumulate content until 500-1000 token range
4. Split at nearest sentence/paragraph boundary within range
5. Add 100-token overlap from previous chunk to maintain context
6. Preserve metadata: source file, section heading, chunk index

### Alternatives Considered
- **Fixed 512-token chunks**: Simpler but often splits mid-concept - rejected for poor semantic coherence
- **Paragraph-based chunking**: Variable size, some paragraphs too large (>2000 tokens) - rejected for inconsistent retrieval granularity
- **Recursive character splitting**: No semantic awareness - rejected for context fragmentation

### Handling Edge Cases
- **Code blocks**: Keep intact within chunks when possible; if >1000 tokens, split at logical boundaries (function definitions)
- **Tables**: Preserve as single units; if exceeds limit, include table header with each chunk
- **Diagrams (text descriptions)**: Keep with associated explanation text
- **Frontmatter/metadata**: Extract separately, do not embed

## 3. Vector Database Configuration (Qdrant)

### Decision
Use **Qdrant Cloud Free Tier** with optimized collection configuration.

### Collection Configuration
```python
collection_config = {
    "vectors": {
        "size": 1024,  # Matches Cohere embed-english-v3.0
        "distance": "Cosine"  # Standard for semantic similarity
    },
    "optimizers_config": {
        "indexing_threshold": 20000  # Free tier supports up to 1M vectors
    },
    "hnsw_config": {
        "m": 16,  # Connections per node (balance speed/accuracy)
        "ef_construct": 100  # Quality of index construction
    }
}
```

### Rationale
- **Cosine similarity**: Standard for embedding-based retrieval, normalized comparison
- **HNSW indexing**: Fast approximate nearest neighbor search, suitable for free tier scale
- **Free tier limits**: 1M vectors, 1GB RAM - sufficient for estimated 5000-10000 chunks from textbook
- **Metadata filtering**: Qdrant supports payload filtering for module/section scoping

### Metadata Schema
```python
payload = {
    "document_id": "frontend/docs/module-1-ros2/intro.md",
    "module_name": "module-1-ros2",
    "section_heading": "Introduction to ROS 2",
    "chunk_index": 0,
    "total_chunks": 12,
    "text": "Full chunk text...",
    "created_at": "2025-12-25T10:00:00Z"
}
```

### Alternatives Considered
- **Pinecone**: Better managed service but free tier limited to 1 project - rejected for future scalability
- **Weaviate**: Rich features but heavier setup - rejected for hackathon simplicity
- **Chroma**: Lightweight but less mature cloud offering - rejected for production readiness

## 4. Deployment Strategy (Vercel)

### Decision
**Manual Vercel deployment** via CLI for initial setup, GitHub integration for future updates.

### Rationale
- **Speed**: Vercel CLI deploys in <2 minutes vs. GitHub Actions setup time
- **Free tier**: Generous limits for static sites (100GB bandwidth/month)
- **Docusaurus compatibility**: Official Vercel deployment guide for Docusaurus
- **Future-ready**: Easy migration to GitHub Actions for automated deployments in later specs

### Deployment Steps
1. Install Vercel CLI: `npm install -g vercel`
2. Authenticate: `vercel login`
3. Deploy from frontend/: `cd frontend && vercel --prod`
4. Capture deployment URL from output
5. Verify accessibility by fetching homepage HTML

### Alternatives Considered
- **GitHub Pages**: Specified in constitution for book deployment but Vercel provides better preview environments for hackathon - documented as constitutional variance
- **Netlify**: Similar to Vercel but Vercel has better Docusaurus integration - rejected for optimization
- **Manual GitHub Actions**: More setup overhead for initial deployment - deferred to future iteration

## 5. Python Environment and Dependency Management

### Decision
Use **uv** (ultra-fast Python package manager) for dependency management.

### Rationale
- **Speed**: 10-100x faster than pip for installs and resolution
- **Modern tooling**: Built in Rust, designed for reproducible environments
- **Simplicity**: Single command initialization (`uv init`)
- **Compatibility**: Drop-in replacement for pip, works with standard requirements.txt
- **Lockfile support**: Automatic dependency locking for reproducibility

### Project Structure
```
backend/
├── pyproject.toml      # UV project config
├── uv.lock             # Locked dependencies
├── main.py             # Orchestration pipeline
├── chunking.py         # Markdown chunking logic
├── embedding.py        # Cohere API integration
├── storage.py          # Qdrant operations
├── deployment.py       # Vercel deployment functions
└── tests/
    ├── test_chunking.py
    ├── test_embedding.py
    └── test_integration.py
```

### Core Dependencies
```toml
[project]
dependencies = [
    "cohere>=5.0.0",           # Cohere SDK
    "qdrant-client>=1.7.0",    # Qdrant Python client
    "tiktoken>=0.5.0",         # Token counting
    "python-dotenv>=1.0.0",    # Environment variable management
    "markdown-it-py>=3.0.0",   # Markdown parsing
    "requests>=2.31.0",        # HTTP for Vercel API
]

[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",
    "pytest-cov>=4.1.0",
]
```

### Alternatives Considered
- **Poetry**: Feature-rich but slower and heavier - rejected for speed priority
- **pip + venv**: Standard but manual dependency management - rejected for reproducibility concerns
- **Conda**: Overkill for Python-only project - rejected for simplicity

## 6. Error Handling and Retry Strategy

### Decision
Implement **exponential backoff with jitter** for API retries.

### Retry Configuration
```python
retry_config = {
    "max_retries": 3,
    "base_delay": 1,  # seconds
    "max_delay": 60,  # seconds
    "exponential_base": 2,
    "jitter": True,  # Randomize delays to avoid thundering herd
    "retryable_errors": [
        "rate_limit_exceeded",
        "service_unavailable",
        "timeout"
    ]
}
```

### Rationale
- **API rate limits**: Cohere free tier: 100 calls/min, Qdrant Cloud: reasonable defaults
- **Transient failures**: Network issues, temporary service outages
- **Batch processing**: Process 100 chunks, pause 60s if rate limit hit
- **Logging**: Capture retry attempts, final failures with context

### Error Categories
1. **Retryable**: Rate limits, timeouts, 5xx errors → exponential backoff
2. **Non-retryable**: Invalid API keys, malformed requests → immediate failure with clear message
3. **Partial failures**: Some chunks succeed, some fail → log failures, continue processing, report at end

## 7. Testing Strategy

### Unit Tests
- **Chunking**: Verify token counts, overlap logic, boundary splitting
- **Embedding**: Mock Cohere API, test batch processing, error handling
- **Storage**: Mock Qdrant client, test upsert logic, metadata formatting

### Integration Tests
- **End-to-end pipeline**: Ingest sample docs/module-1-ros2/intro.md
- **Verify**: Chunks created, embeddings generated, vectors in Qdrant
- **Query test**: Search for "ROS 2 introduction" and validate top result

### Deployment Verification
- **Smoke test**: HTTP GET to Vercel URL, verify 200 status and content
- **Search test**: Verify Docusaurus search returns results

### Test Data
- **Sample files**: Use existing frontend/docs/module-1-ros2/ for realistic testing
- **Edge cases**: Empty file, large file (>5000 tokens), file with code blocks

## 8. Logging and Observability

### Decision
Use Python's **structu

red logging** with JSON output for pipeline observability.

### Log Schema
```python
{
    "timestamp": "2025-12-25T10:00:00Z",
    "level": "INFO",
    "stage": "chunking|embedding|storage|deployment",
    "message": "Processed file: module-1-ros2/intro.md",
    "metadata": {
        "file": "frontend/docs/module-1-ros2/intro.md",
        "chunks_created": 12,
        "tokens_processed": 8450
    }
}
```

### Logging Levels
- **DEBUG**: Token counts, chunk boundaries, API request details
- **INFO**: File processing progress, API calls, deployment status
- **WARNING**: Retry attempts, skipped files (empty/invalid)
- **ERROR**: API failures, storage errors, deployment failures

## 9. Constitutional Compliance Review

### Alignment with Constitution
- ✅ **Structural separation**: Backend in /backend, frontend unchanged in /frontend
- ✅ **Spec-first execution**: All decisions documented in research.md before implementation
- ✅ **No frontend modification**: Ingestion reads from frontend/docs/, does not modify
- ✅ **Quality gates**: Testing strategy defined for pipeline validation

### Documented Variance
- **Deployment target**: Vercel (specified in user input) vs. GitHub Pages (constitution default)
  - **Justification**: Vercel provides faster iteration for hackathon; GitHub Pages deferred to post-hackathon
  - **Migration path**: Vercel deployment can coexist with GitHub Pages; switch post-demo

## Summary

All technical unknowns resolved. Key decisions:
1. **Embedding**: Cohere embed-english-v3.0 (1024-dim)
2. **Chunking**: 500-1000 tokens, 100-token overlap, semantic boundaries
3. **Storage**: Qdrant Cloud with cosine similarity, HNSW indexing
4. **Deployment**: Vercel CLI for speed, GitHub Actions future
5. **Tooling**: UV for Python deps, structured JSON logging
6. **Error handling**: Exponential backoff with 3 retries
7. **Testing**: Unit + integration + deployment verification

Ready for Phase 1: Data model and contracts design.
