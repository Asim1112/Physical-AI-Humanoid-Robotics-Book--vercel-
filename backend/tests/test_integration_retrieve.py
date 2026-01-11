"""Integration tests for retrieval functionality.

These tests require actual Qdrant and Cohere connections.
Use fixtures for mocking if needed in CI/CD environments.
"""

import os
import pytest
from qdrant_client import QdrantClient

from retrieve import retrieve_content, validate_retrieval_with_sample_queries


# ==============================================================================
# Fixtures
# ==============================================================================

@pytest.fixture
def qdrant_client():
    """Create Qdrant client for integration tests."""
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")

    if not url or not api_key:
        pytest.skip("Qdrant credentials not available")

    return QdrantClient(url=url, api_key=api_key)


@pytest.fixture
def collection_name():
    """Get collection name from environment."""
    name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook")
    return name


@pytest.fixture
def cohere_api_key():
    """Get Cohere API key from environment."""
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        pytest.skip("Cohere API key not available")
    return api_key


# ==============================================================================
# T015: Integration test for ROS2 query
# ==============================================================================

@pytest.mark.integration
def test_ros2_installation_query(qdrant_client, collection_name, cohere_api_key):
    """Test retrieval with ROS2 installation query.

    Expected: Retrieved chunks should have similarity scores >0.7
    from ROS2-related modules.
    """
    query = "How do I install ROS 2?"

    result = retrieve_content(
        query_text=query,
        client=qdrant_client,
        collection_name=collection_name,
        top_k=5,
        score_threshold=0.7,
        api_key=cohere_api_key
    )

    # Verify we got results
    assert len(result.retrieved_chunks) > 0, "Should retrieve at least one chunk"

    # Verify similarity scores
    for chunk in result.retrieved_chunks:
        assert chunk.similarity_score >= 0.7, \
            f"Chunk {chunk.chunk_id} has similarity {chunk.similarity_score} < 0.7"

    # Verify ROS2 relevance (at least one chunk should mention ROS2)
    ros2_mentions = any(
        "ros" in chunk.content_text.lower() or
        "ros2" in chunk.module_name.lower()
        for chunk in result.retrieved_chunks
    )
    assert ros2_mentions, "At least one chunk should be related to ROS2"

    # Verify retrieval time
    assert result.retrieval_time_ms < 2000, \
        f"Retrieval took {result.retrieval_time_ms}ms, should be <2000ms"


# ==============================================================================
# T016: Integration test for Gazebo query
# ==============================================================================

@pytest.mark.integration
def test_gazebo_simulation_query(qdrant_client, collection_name, cohere_api_key):
    """Test retrieval with Gazebo simulation query.

    Expected: Retrieved chunks should have similarity scores >0.7
    from Gazebo-related modules.
    """
    query = "What is Gazebo simulation?"

    result = retrieve_content(
        query_text=query,
        client=qdrant_client,
        collection_name=collection_name,
        top_k=5,
        score_threshold=0.7,
        api_key=cohere_api_key
    )

    # Verify we got results
    assert len(result.retrieved_chunks) > 0, "Should retrieve at least one chunk"

    # Verify similarity scores
    for chunk in result.retrieved_chunks:
        assert chunk.similarity_score >= 0.7, \
            f"Chunk {chunk.chunk_id} has similarity {chunk.similarity_score} < 0.7"

    # Verify Gazebo relevance
    gazebo_mentions = any(
        "gazebo" in chunk.content_text.lower() or
        "gazebo" in chunk.module_name.lower()
        for chunk in result.retrieved_chunks
    )
    assert gazebo_mentions, "At least one chunk should be related to Gazebo"

    # Verify retrieval time
    assert result.retrieval_time_ms < 2000, \
        f"Retrieval took {result.retrieval_time_ms}ms, should be <2000ms"


# ==============================================================================
# T026-T028: Integration tests for User Story 2
# ==============================================================================

@pytest.mark.integration
def test_vla_models_query(qdrant_client, collection_name, cohere_api_key):
    """Test retrieval with VLA models query."""
    query = "Explain VLA models"

    result = retrieve_content(
        query_text=query,
        client=qdrant_client,
        collection_name=collection_name,
        top_k=5,
        score_threshold=0.5,  # Lower threshold for VLA (may not be in textbook)
        api_key=cohere_api_key
    )

    # Verify we got results (may be empty if VLA not in textbook)
    assert isinstance(result.retrieved_chunks, list)

    # If we got results, verify they meet minimum quality
    if len(result.retrieved_chunks) > 0:
        for chunk in result.retrieved_chunks:
            assert chunk.similarity_score >= 0.5


@pytest.mark.integration
def test_humanoid_robots_query(qdrant_client, collection_name, cohere_api_key):
    """Test retrieval with humanoid robots query."""
    query = "How to work with humanoid robots?"

    result = retrieve_content(
        query_text=query,
        client=qdrant_client,
        collection_name=collection_name,
        top_k=5,
        score_threshold=0.5,
        api_key=cohere_api_key
    )

    # Should get results for humanoid robotics textbook
    assert len(result.retrieved_chunks) > 0, "Should retrieve chunks for humanoid robots query"

    # Verify retrieval time
    assert result.retrieval_time_ms < 2000


@pytest.mark.integration
def test_robotics_concepts_query(qdrant_client, collection_name, cohere_api_key):
    """Test retrieval with general robotics concepts query."""
    query = "What are the key concepts in robotics?"

    result = retrieve_content(
        query_text=query,
        client=qdrant_client,
        collection_name=collection_name,
        top_k=5,
        score_threshold=0.5,
        api_key=cohere_api_key
    )

    # Should get results
    assert len(result.retrieved_chunks) > 0

    # Verify retrieval time
    assert result.retrieval_time_ms < 2000


# ==============================================================================
# T029: Performance benchmark test
# ==============================================================================

@pytest.mark.integration
@pytest.mark.benchmark
def test_retrieval_latency_under_2_seconds(qdrant_client, collection_name, cohere_api_key):
    """Test that retrieval latency is under 2 seconds for 95% of queries.

    This test runs multiple queries and checks the 95th percentile latency.
    """
    test_queries = [
        "How do I install ROS 2?",
        "What is Gazebo simulation?",
        "Explain VLA models",
        "How to work with humanoid robots?",
        "What are the key concepts in robotics?",
        "ROS 2 navigation setup",
        "Gazebo physics engine",
        "Robot perception systems",
        "Humanoid locomotion control",
        "Sensor fusion techniques"
    ]

    latencies = []

    for query in test_queries:
        result = retrieve_content(
            query_text=query,
            client=qdrant_client,
            collection_name=collection_name,
            api_key=cohere_api_key
        )
        latencies.append(result.retrieval_time_ms)

    # Calculate 95th percentile
    latencies.sort()
    p95_index = int(len(latencies) * 0.95)
    p95_latency = latencies[p95_index]

    # Verify 95th percentile is under 2 seconds
    assert p95_latency < 2000, \
        f"95th percentile latency is {p95_latency}ms, should be <2000ms"

    # Verify average latency
    avg_latency = sum(latencies) / len(latencies)
    assert avg_latency < 2000, \
        f"Average latency is {avg_latency}ms, should be <2000ms"


# ==============================================================================
# T041-T043: Error handling integration tests (User Story 3)
# ==============================================================================

@pytest.mark.integration
def test_cohere_api_error(qdrant_client, collection_name):
    """Test handling of Cohere API failure."""
    from retrieve import EmbeddingError

    # Use invalid API key to trigger error
    with pytest.raises(EmbeddingError):
        retrieve_content(
            query_text="Test query",
            client=qdrant_client,
            collection_name=collection_name,
            api_key="invalid_key_12345"
        )


@pytest.mark.integration
def test_qdrant_connection_error():
    """Test handling of Qdrant connection failure."""
    from retrieve import SearchError
    from qdrant_client import QdrantClient

    # Create client with invalid URL
    invalid_client = QdrantClient(url="http://invalid-url:6333")

    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        pytest.skip("Cohere API key not available")

    # Should raise SearchError
    with pytest.raises(SearchError):
        retrieve_content(
            query_text="Test query",
            client=invalid_client,
            collection_name="test",
            api_key=api_key
        )


@pytest.mark.integration
def test_no_relevant_results_found(qdrant_client, collection_name, cohere_api_key):
    """Test handling when no results meet the threshold."""
    # Use very high threshold to ensure no results
    result = retrieve_content(
        query_text="xyzabc nonsense query 12345",
        client=qdrant_client,
        collection_name=collection_name,
        top_k=5,
        score_threshold=0.95,  # Very high threshold
        api_key=cohere_api_key
    )

    # Should return empty chunks list, not error
    assert isinstance(result.retrieved_chunks, list)
    assert len(result.retrieved_chunks) == 0 or all(
        chunk.similarity_score >= 0.95 for chunk in result.retrieved_chunks
    )
