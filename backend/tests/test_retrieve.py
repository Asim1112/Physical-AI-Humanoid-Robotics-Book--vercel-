"""Unit tests for retrieval functionality."""

import pytest
from datetime import datetime
from unittest.mock import Mock, patch, MagicMock

from retrieve import (
    Query,
    RetrievedChunk,
    RetrievalResult,
    PerformanceMetric,
    QueryValidationError,
    EmbeddingError,
    SearchError,
    retrieve_content,
    validate_retrieval_with_sample_queries
)


# ==============================================================================
# T012: Unit tests for Query data validation
# ==============================================================================

class TestQueryValidation:
    """Tests for Query dataclass validation."""

    def test_query_validation_success(self):
        """Test successful query creation with valid data."""
        query = Query(
            query_text="How do I install ROS 2?",
            query_embedding=[0.1] * 1024,
            query_type="ROS2",
            created_at="2025-12-25T10:00:00",
            embedding_model="embed-english-v3.0"
        )

        assert query.query_text == "How do I install ROS 2?"
        assert len(query.query_embedding) == 1024
        assert query.query_type == "ROS2"
        assert query.embedding_model == "embed-english-v3.0"

    def test_query_validation_empty_text(self):
        """Test query creation with empty text."""
        # Query dataclass allows empty text, but retrieve_content should validate
        query = Query(
            query_text="",
            query_embedding=[0.1] * 1024
        )

        assert query.query_text == ""


# ==============================================================================
# T013: Unit tests for RetrievedChunk data validation
# ==============================================================================

class TestRetrievedChunkValidation:
    """Tests for RetrievedChunk dataclass validation."""

    def test_chunk_validation_success(self):
        """Test successful chunk creation with valid data."""
        chunk = RetrievedChunk(
            chunk_id="chunk-123",
            similarity_score=0.85,
            content_text="ROS 2 installation guide...",
            source_file="module-1-ros2/installation.md",
            module_name="module-1-ros2",
            section_heading="Installing ROS 2",
            chunk_index=0,
            total_chunks=3,
            token_count=245,
            retrieved_at="2025-12-25T10:00:00"
        )

        assert chunk.chunk_id == "chunk-123"
        assert chunk.similarity_score == 0.85
        assert chunk.content_text == "ROS 2 installation guide..."
        assert chunk.source_file == "module-1-ros2/installation.md"
        assert chunk.module_name == "module-1-ros2"
        assert chunk.section_heading == "Installing ROS 2"
        assert chunk.chunk_index == 0
        assert chunk.total_chunks == 3
        assert chunk.token_count == 245

    def test_chunk_validation_invalid_score(self):
        """Test chunk creation with invalid similarity score."""
        # Dataclass allows any float, but application logic should validate
        chunk = RetrievedChunk(
            chunk_id="chunk-123",
            similarity_score=1.5,  # Invalid: should be 0.0-1.0
            content_text="Test content",
            source_file="test.md",
            module_name="test",
            section_heading="Test",
            chunk_index=0,
            total_chunks=1,
            token_count=10
        )

        # Dataclass creation succeeds, but score is invalid
        assert chunk.similarity_score == 1.5


# ==============================================================================
# T014: Unit tests for retrieve_content() function
# ==============================================================================

class TestRetrieveContent:
    """Tests for retrieve_content function."""

    @patch('retrieve.generate_embedding')
    @patch('retrieve.query_similar')
    def test_retrieve_content_success(self, mock_query_similar, mock_generate_embedding):
        """Test successful content retrieval."""
        # Mock embedding generation
        mock_generate_embedding.return_value = [0.1] * 1024

        # Mock Qdrant search results
        mock_query_similar.return_value = [
            ("chunk-1", 0.85, {
                "text": "ROS 2 installation instructions...",
                "source_file": "module-1-ros2/installation.md",
                "module_name": "module-1-ros2",
                "section_heading": "Installing ROS 2",
                "chunk_index": 0,
                "total_chunks": 3,
                "token_count": 245
            }),
            ("chunk-2", 0.75, {
                "text": "Prerequisites for ROS 2...",
                "source_file": "module-1-ros2/installation.md",
                "module_name": "module-1-ros2",
                "section_heading": "Prerequisites",
                "chunk_index": 1,
                "total_chunks": 3,
                "token_count": 180
            })
        ]

        # Mock Qdrant client
        mock_client = Mock()

        # Execute retrieval
        result = retrieve_content(
            query_text="How do I install ROS 2?",
            client=mock_client,
            collection_name="test-collection",
            top_k=5,
            score_threshold=0.7
        )

        # Verify result
        assert result.original_query == "How do I install ROS 2?"
        assert len(result.retrieved_chunks) == 2
        assert result.retrieved_chunks[0].similarity_score == 0.85
        assert result.retrieved_chunks[0].source_file == "module-1-ros2/installation.md"
        assert result.retrieval_time_ms > 0
        assert result.total_candidates == 2

        # Verify embedding was called correctly
        mock_generate_embedding.assert_called_once_with(
            "How do I install ROS 2?",
            api_key=None,
            input_type="search_query"
        )

        # Verify Qdrant search was called correctly
        mock_query_similar.assert_called_once()

    @patch('retrieve.generate_embedding')
    def test_retrieve_content_empty_query(self, mock_generate_embedding):
        """Test retrieval with empty query."""
        mock_generate_embedding.return_value = [0.1] * 1024

        mock_client = Mock()

        # Empty query should raise QueryValidationError (T017)
        with pytest.raises(QueryValidationError, match="Query text cannot be empty"):
            retrieve_content(
                query_text="",
                client=mock_client,
                collection_name="test-collection"
            )

    @patch('retrieve.generate_embedding')
    def test_retrieve_content_embedding_error(self, mock_generate_embedding):
        """Test handling of embedding generation failure."""
        # Mock embedding failure
        mock_generate_embedding.side_effect = Exception("Cohere API error")

        mock_client = Mock()

        # Should raise EmbeddingError
        with pytest.raises(EmbeddingError, match="Failed to generate embedding"):
            retrieve_content(
                query_text="Test query",
                client=mock_client,
                collection_name="test-collection"
            )

    @patch('retrieve.generate_embedding')
    @patch('retrieve.query_similar')
    def test_retrieve_content_search_error(self, mock_query_similar, mock_generate_embedding):
        """Test handling of Qdrant search failure."""
        mock_generate_embedding.return_value = [0.1] * 1024
        mock_query_similar.side_effect = Exception("Qdrant connection error")

        mock_client = Mock()

        # Should raise SearchError
        with pytest.raises(SearchError, match="Failed to search in Qdrant"):
            retrieve_content(
                query_text="Test query",
                client=mock_client,
                collection_name="test-collection"
            )


# ==============================================================================
# T024-T025: Unit tests for validation functions (User Story 2)
# ==============================================================================

class TestValidationFunctions:
    """Tests for validation-related functions."""

    def test_performance_metric_validation(self):
        """Test PerformanceMetric dataclass validation."""
        metric = PerformanceMetric(
            metric_id="metric-1",
            query_text="Test query",
            latency_ms=423.5,
            relevance_score=0.85,
            result_count=5,
            query_category="ROS2",
            timestamp="2025-12-25T10:00:00",
            success=True
        )

        assert metric.metric_id == "metric-1"
        assert metric.query_text == "Test query"
        assert metric.latency_ms == 423.5
        assert metric.relevance_score == 0.85
        assert metric.result_count == 5
        assert metric.query_category == "ROS2"
        assert metric.success is True

    @patch('retrieve.retrieve_content')
    def test_validation_with_sample_queries(self, mock_retrieve_content):
        """Test validate_retrieval_with_sample_queries function."""
        # Mock successful retrieval
        mock_result = RetrievalResult(
            query_id="query-1",
            original_query="How do I install ROS 2?",
            retrieved_chunks=[
                RetrievedChunk(
                    chunk_id="chunk-1",
                    similarity_score=0.85,
                    content_text="ROS 2 installation...",
                    source_file="module-1-ros2/installation.md",
                    module_name="module-1-ros2",
                    section_heading="Installing ROS 2",
                    chunk_index=0,
                    total_chunks=3,
                    token_count=245
                )
            ],
            retrieval_time_ms=423.5,
            total_candidates=1
        )
        mock_retrieve_content.return_value = mock_result

        mock_client = Mock()

        # Execute validation
        results = validate_retrieval_with_sample_queries(
            client=mock_client,
            collection_name="test-collection"
        )

        # Should have 5 sample queries
        assert len(results) == 5

        # All should be successful with mock
        for query, result, success in results:
            assert isinstance(query, str)
            assert result is not None
            assert success is True
