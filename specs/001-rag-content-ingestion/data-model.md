# Data Model: RAG Content Ingestion Pipeline

**Feature**: 001-rag-content-ingestion
**Date**: 2025-12-25
**Source**: Derived from spec.md Key Entities and research.md decisions

## Entity Relationships

```
MarkdownFile (1) ──── (N) DocumentChunk
DocumentChunk (1) ──── (1) EmbeddingVector
EmbeddingVector (1) ──── (1) QdrantPoint
ProcessingJob (1) ──── (N) MarkdownFile
```

## 1. MarkdownFile

Represents a source markdown file from frontend/docs/.

### Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| file_path | str | Absolute path to markdown file | Must exist, must be .md or .mdx |
| relative_path | str | Path relative to frontend/docs/ | Used for metadata |
| module_name | str | Extracted from path (e.g., "module-1-ros2") | Regex: `module-\d+-[\w-]+` |
| content | str | Raw markdown content | Non-empty after frontmatter removal |
| frontmatter | dict | YAML frontmatter metadata | Optional, parsed if present |
| file_size_bytes | int | File size in bytes | > 0 |
| last_modified | datetime | File modification timestamp | ISO 8601 format |

### Extraction Logic

```python
def extract_markdown_file(file_path: str) -> MarkdownFile:
    """
    Parse markdown file and extract metadata.

    Validation:
    - File must exist and be readable
    - Must have .md or .mdx extension
    - Content must be non-empty after frontmatter removal
    """
    # Parse frontmatter if present (delimited by ---)
    # Extract module_name from path pattern
    # Capture file stats (size, modified time)
```

## 2. DocumentChunk

A segment of textbook content ready for embedding.

### Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| chunk_id | str | UUID v4 for unique identification | UUID format |
| source_file | str | Relative path to source markdown | Matches MarkdownFile.relative_path |
| module_name | str | Module this chunk belongs to | Inherited from MarkdownFile |
| section_heading | str | Heading text for the chunk's section | Extracted from markdown structure |
| chunk_index | int | Position in sequence (0-based) | >= 0 |
| total_chunks | int | Total chunks from this file | >= 1, >= chunk_index |
| text | str | Chunk content (plain text) | 400-1100 tokens |
| token_count | int | Exact token count (tiktoken) | 400-1100 (except final chunks: >=100) |
| overlap_text | str | Text overlapping with previous chunk | 0-100 tokens |
| created_at | datetime | Timestamp when chunk created | ISO 8601 format |

### Business Rules

1. **Token count**: 500-1000 tokens (target), 400-1100 (acceptable range)
2. **Final chunks**: Minimum 100 tokens (can be < 400 for last chunk)
3. **Overlap**: 100 tokens from previous chunk (except first chunk)
4. **Semantic boundaries**: Split at sentence/paragraph boundaries within token range
5. **Code block preservation**: Keep code blocks intact; split only if >1100 tokens

### State Transitions

```
Created → Validated → Embedded → Stored
```

- **Created**: Chunk generated from markdown
- **Validated**: Token count and semantic coherence verified
- **Embedded**: Cohere API returned embedding vector
- **Stored**: Vector uploaded to Qdrant

### Validation Rules

```python
def validate_chunk(chunk: DocumentChunk) -> bool:
    """
    Validate chunk meets requirements.

    Rules:
    - Token count: 400-1100 (or >=100 for final chunk)
    - Text is non-empty
    - chunk_index < total_chunks
    - section_heading extracted (warn if missing)
    """
```

## 3. EmbeddingVector

Semantic representation of a DocumentChunk.

### Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| vector_id | str | Same as chunk_id for 1:1 mapping | UUID format |
| chunk_id | str | Foreign key to DocumentChunk | Must exist |
| vector | List[float] | 1024-dimensional embedding | Length == 1024, all floats |
| model_name | str | Cohere model used | "embed-english-v3.0" |
| input_type | str | Embedding input type | "search_document" |
| created_at | datetime | Timestamp of embedding generation | ISO 8601 format |

### Generation Logic

```python
def generate_embedding(chunk: DocumentChunk) -> EmbeddingVector:
    """
    Call Cohere API to generate embedding.

    API Call:
    - Model: embed-english-v3.0
    - Input type: search_document
    - Text: chunk.text
    - Truncate: none (pre-validated token count)

    Returns:
    - 1024-dimensional float vector
    """
```

### Validation Rules

1. **Vector dimensions**: Exactly 1024 floats
2. **Value range**: Each float in [-1.0, 1.0] (post-normalization)
3. **Model consistency**: All embeddings use same model version

## 4. QdrantPoint

Representation of a vector stored in Qdrant collection.

### Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| point_id | str | Same as vector_id | UUID format |
| vector | List[float] | 1024-dimensional embedding | From EmbeddingVector |
| payload | dict | Metadata for filtering/display | See payload schema below |
| collection_name | str | Qdrant collection name | "humanoid-robotics-textbook" |

### Payload Schema

```python
{
    "document_id": str,        # source_file path
    "module_name": str,        # e.g., "module-1-ros2"
    "section_heading": str,    # e.g., "Introduction to ROS 2"
    "chunk_index": int,        # Position in file
    "total_chunks": int,       # Total from file
    "text": str,               # Full chunk text
    "token_count": int,        # Exact token count
    "created_at": str,         # ISO 8601 timestamp
    "model": str               # "embed-english-v3.0"
}
```

### Upsert Logic

```python
def upsert_to_qdrant(point: QdrantPoint) -> bool:
    """
    Upload vector to Qdrant collection.

    Collection config:
    - Vector size: 1024
    - Distance: Cosine
    - Indexing: HNSW (m=16, ef_construct=100)

    Returns:
    - True if successful, False on error
    """
```

### Query Operations

```python
def query_similar(query_text: str, top_k: int = 5) -> List[QdrantPoint]:
    """
    Find similar chunks by semantic search.

    Steps:
    1. Generate embedding for query_text (input_type="search_query")
    2. Search Qdrant with cosine similarity
    3. Return top_k results with scores and metadata
    """
```

## 5. ProcessingJob

Tracks ingestion pipeline execution.

### Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| job_id | str | UUID v4 for job tracking | UUID format |
| files_discovered | int | Total markdown files found | >= 0 |
| files_processed | int | Successfully processed | <= files_discovered |
| files_failed | int | Failed to process | <= files_discovered |
| total_chunks | int | Chunks created across all files | >= 0 |
| total_embeddings | int | Embeddings generated | == total_chunks (on success) |
| total_vectors_stored | int | Vectors uploaded to Qdrant | == total_embeddings (on success) |
| started_at | datetime | Job start timestamp | ISO 8601 format |
| completed_at | datetime | Job completion timestamp | >= started_at |
| status | str | Job status | "running", "completed", "failed", "partial" |
| errors | List[dict] | Error details for failed files | Each: {file, stage, error, timestamp} |

### Status Transitions

```
created → running → (completed | failed | partial)
```

- **created**: Job initialized
- **running**: Processing files
- **completed**: All files processed successfully
- **failed**: Job terminated due to critical error
- **partial**: Some files succeeded, some failed

### Completion Logic

```python
def finalize_job(job: ProcessingJob) -> str:
    """
    Determine final job status.

    Logic:
    - completed: files_failed == 0
    - partial: 0 < files_failed < files_discovered
    - failed: files_failed == files_discovered OR critical error
    """
```

## Validation Summary

### Cross-Entity Rules

1. **Chunk-Vector 1:1**: Each DocumentChunk has exactly one EmbeddingVector
2. **Vector-Point 1:1**: Each EmbeddingVector maps to one QdrantPoint
3. **Chunk sequence**: chunk_index values for a file must be sequential (0 to total_chunks-1)
4. **Token consistency**: Sum of chunk token_counts ≈ original file token count (±overlap)
5. **Module consistency**: All chunks from same file have same module_name

### Data Integrity Checks

```python
def validate_pipeline_output(job: ProcessingJob) -> List[str]:
    """
    Post-processing validation checks.

    Returns list of validation errors (empty if valid):
    - Total chunks == sum of chunks across all files
    - Total embeddings == total chunks
    - Total vectors stored == total embeddings
    - All chunk_ids have corresponding vector_ids
    - All vector_ids exist in Qdrant
    - Token counts within valid ranges
    """
```

## Example Data Flow

```python
# 1. MarkdownFile discovered
file = MarkdownFile(
    file_path="/path/to/frontend/docs/module-1-ros2/intro.md",
    relative_path="module-1-ros2/intro.md",
    module_name="module-1-ros2",
    content="# Introduction to ROS 2\n\n...",
    frontmatter={"title": "ROS 2 Intro"},
    token_count=8450
)

# 2. Chunked into DocumentChunks
chunks = [
    DocumentChunk(
        chunk_id="uuid-1",
        source_file="module-1-ros2/intro.md",
        module_name="module-1-ros2",
        section_heading="Introduction to ROS 2",
        chunk_index=0,
        total_chunks=12,
        text="# Introduction to ROS 2\n\nROS 2 is...",
        token_count=850,
        overlap_text="",  # First chunk
    ),
    # ... 11 more chunks
]

# 3. Embeddings generated
vectors = [
    EmbeddingVector(
        vector_id="uuid-1",
        chunk_id="uuid-1",
        vector=[0.123, -0.456, ..., 0.789],  # 1024 floats
        model_name="embed-english-v3.0",
        input_type="search_document"
    ),
    # ... 11 more vectors
]

# 4. Stored in Qdrant
points = [
    QdrantPoint(
        point_id="uuid-1",
        vector=[0.123, -0.456, ..., 0.789],
        payload={
            "document_id": "module-1-ros2/intro.md",
            "module_name": "module-1-ros2",
            "section_heading": "Introduction to ROS 2",
            "chunk_index": 0,
            "total_chunks": 12,
            "text": "# Introduction to ROS 2\n\nROS 2 is...",
            "token_count": 850,
            "model": "embed-english-v3.0"
        }
    ),
    # ... 11 more points
]
```

## Storage Estimates

Based on frontend/docs/ content:

- **Estimated files**: ~50 markdown files
- **Average file size**: ~5000 tokens
- **Chunks per file**: ~7 (at 700 tokens/chunk)
- **Total chunks**: ~350 chunks
- **Vector storage**: 350 vectors × 1024 dims × 4 bytes ≈ 1.4 MB (well under Qdrant free tier)
- **Metadata storage**: ~350 KB (text + metadata per chunk)
- **Total Qdrant usage**: <2 MB (far below 1 GB limit)

## Next Steps

With data model defined, proceed to:
1. Generate API contracts for Cohere and Qdrant
2. Create quickstart.md for setup instructions
3. Fill implementation plan with technical context
