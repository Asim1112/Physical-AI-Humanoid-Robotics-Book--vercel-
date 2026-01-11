"""Pytest configuration and fixtures for RAG ingestion tests."""

import pytest
from unittest.mock import Mock, MagicMock
from typing import List, Dict, Any


# ==============================================================================
# Cohere Mock Fixtures (T014)
# ==============================================================================

@pytest.fixture
def mock_cohere_client():
    """
    Mock Cohere client for testing embedding generation.

    Returns:
        Mock Cohere client with embed() method
    """
    client = Mock()

    # Mock embed response
    embed_response = Mock()
    embed_response.embeddings = [
        [0.1] * 1024  # Mock 1024-dimensional embedding
    ]
    embed_response.meta = {
        "billed_units": {"input_tokens": 100}
    }

    client.embed = Mock(return_value=embed_response)

    return client


@pytest.fixture
def mock_cohere_batch_embeddings():
    """
    Mock batch embeddings response for testing.

    Returns:
        Mock response with multiple embeddings
    """
    response = Mock()
    response.embeddings = [
        [0.1 + i * 0.01] * 1024  # Slightly different vectors
        for i in range(10)
    ]
    response.meta = {
        "billed_units": {"input_tokens": 500}
    }
    return response


# ==============================================================================
# Qdrant Mock Fixtures (T014)
# ==============================================================================

@pytest.fixture
def mock_qdrant_client():
    """
    Mock Qdrant client for testing vector storage operations.

    Returns:
        Mock Qdrant client with create_collection, upsert, search methods
    """
    client = Mock()

    # Mock collection operations
    client.create_collection = Mock(return_value=True)
    client.get_collections = Mock(return_value=Mock(collections=[]))

    # Mock upsert operation
    upsert_response = Mock()
    upsert_response.operation_id = 123
    upsert_response.status = "completed"
    client.upsert = Mock(return_value=upsert_response)

    # Mock search operation
    def mock_search(collection_name, query_vector, limit=5, **kwargs):
        """Mock search results with scored points."""
        results = []
        for i in range(min(limit, 5)):
            point = Mock()
            point.id = f"uuid-{i}"
            point.score = 0.95 - (i * 0.05)  # Decreasing scores
            point.payload = {
                "document_id": f"module-{i}/test.md",
                "module_name": f"module-{i}",
                "section_heading": f"Section {i}",
                "chunk_index": i,
                "total_chunks": 10,
                "text": f"Sample text chunk {i}",
                "token_count": 500 + i * 10,
                "model": "embed-english-v3.0"
            }
            results.append(point)
        return results

    client.search = Mock(side_effect=mock_search)

    return client


@pytest.fixture
def mock_qdrant_collection_info():
    """
    Mock Qdrant collection info for validation tests.

    Returns:
        Mock collection info with vector config
    """
    info = Mock()
    info.config = Mock()
    info.config.params = Mock()
    info.config.params.vectors = Mock()
    info.config.params.vectors.size = 1024
    info.config.params.vectors.distance = "Cosine"
    return info


# ==============================================================================
# Sample Data Fixtures
# ==============================================================================

@pytest.fixture
def sample_markdown_content():
    """
    Sample markdown content for testing chunking.

    Returns:
        String with markdown content
    """
    return """# Test Document

This is a test document for chunking validation.

## Section 1

This section contains enough text to create multiple chunks when processed.
We need to ensure the chunking logic properly handles markdown structure.

### Subsection 1.1

More content here to increase token count.

## Section 2

Different section with code blocks:

```python
def example_function():
    return "test"
```

And some more text after the code block.
"""


@pytest.fixture
def sample_document_chunk():
    """
    Sample DocumentChunk for testing.

    Returns:
        Dictionary representing a DocumentChunk
    """
    return {
        "chunk_id": "test-uuid-1",
        "source_file": "module-1-ros2/intro.md",
        "module_name": "module-1-ros2",
        "section_heading": "Introduction",
        "chunk_index": 0,
        "total_chunks": 3,
        "text": "Sample chunk text " * 50,  # ~100 tokens
        "token_count": 500,
        "overlap_text": "",
        "created_at": "2025-12-25T10:00:00Z"
    }


@pytest.fixture
def sample_embedding_vector():
    """
    Sample EmbeddingVector for testing.

    Returns:
        Dictionary representing an EmbeddingVector
    """
    return {
        "vector_id": "test-uuid-1",
        "chunk_id": "test-uuid-1",
        "vector": [0.1] * 1024,
        "model_name": "embed-english-v3.0",
        "input_type": "search_document",
        "created_at": "2025-12-25T10:00:00Z"
    }


# ==============================================================================
# Environment Variable Fixtures
# ==============================================================================

@pytest.fixture
def mock_env_vars(monkeypatch):
    """
    Set mock environment variables for testing.

    Args:
        monkeypatch: Pytest monkeypatch fixture

    Yields:
        Dictionary of mocked environment variables
    """
    env_vars = {
        "COHERE_API_KEY": "test_cohere_key_123",
        "QDRANT_URL": "https://test.cloud.qdrant.io:6333",
        "QDRANT_API_KEY": "test_qdrant_key_456",
        "QDRANT_COLLECTION_NAME": "test-collection",
        "FRONTEND_PATH": "../frontend",
        "DOCS_PATH": "docs"
    }

    for key, value in env_vars.items():
        monkeypatch.setenv(key, value)

    yield env_vars


# ==============================================================================
# File System Fixtures
# ==============================================================================

@pytest.fixture
def temp_markdown_files(tmp_path):
    """
    Create temporary markdown files for testing file discovery.

    Args:
        tmp_path: Pytest tmp_path fixture

    Returns:
        Path to temporary directory with markdown files
    """
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()

    # Create module directories
    for module_num in range(1, 4):
        module_dir = docs_dir / f"module-{module_num}"
        module_dir.mkdir()

        # Create markdown files
        for file_num in range(1, 3):
            md_file = module_dir / f"file-{file_num}.md"
            md_file.write_text(f"# Module {module_num} File {file_num}\n\nContent here.")

    return docs_dir
