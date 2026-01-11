"""Module for retrieving textbook content from Qdrant vector database."""

import logging
import time
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

from qdrant_client import QdrantClient

from embedding import generate_embedding
from storage import query_similar
from utils import retry_with_exponential_backoff

logger = logging.getLogger("rag_retrieval")


# ==============================================================================
# T005: Query Entity
# ==============================================================================

@dataclass
class Query:
    """
    User input text that needs to be matched against embedded content.

    Attributes:
        query_text: The original user query text
        query_embedding: 1024-dimensional embedding vector from Cohere
        query_type: Category of query (e.g., "ROS2", "Gazebo", "VLA")
        created_at: Timestamp when query was processed
        embedding_model: Model used for embedding (default: embed-english-v3.0)
    """
    query_text: str
    query_embedding: List[float]
    query_type: Optional[str] = None
    created_at: str = ""
    embedding_model: str = "embed-english-v3.0"


# ==============================================================================
# T006: RetrievedChunk Entity
# ==============================================================================

@dataclass
class RetrievedChunk:
    """
    Content segment returned from Qdrant with similarity score and metadata.

    Attributes:
        chunk_id: Unique identifier for the content chunk
        similarity_score: Cosine similarity score (0.0-1.0)
        content_text: The actual text content of the retrieved chunk
        source_file: Original markdown file path
        module_name: Module name derived from file path
        section_heading: Section heading from original content
        chunk_index: Position of chunk within original document
        total_chunks: Total number of chunks in original document
        token_count: Number of tokens in the chunk
        retrieved_at: Timestamp when chunk was retrieved
    """
    chunk_id: str
    similarity_score: float
    content_text: str
    source_file: str
    module_name: str
    section_heading: str
    chunk_index: int
    total_chunks: int
    token_count: int
    retrieved_at: str = ""


# ==============================================================================
# T007: RetrievalResult Entity
# ==============================================================================

@dataclass
class RetrievalResult:
    """
    Collection of retrieved chunks for a single query.

    Attributes:
        query_id: Unique identifier for the query session
        original_query: The original query text
        retrieved_chunks: Top-K most relevant chunks
        retrieval_time_ms: Time taken to perform retrieval in milliseconds
        total_candidates: Total number of candidates considered
        retrieval_timestamp: When retrieval was performed
    """
    query_id: str
    original_query: str
    retrieved_chunks: List[RetrievedChunk]
    retrieval_time_ms: float
    total_candidates: int
    retrieval_timestamp: str = ""


# ==============================================================================
# T008: PerformanceMetric Entity
# ==============================================================================

@dataclass
class PerformanceMetric:
    """
    Measurement of retrieval performance for validation.

    Attributes:
        metric_id: Unique identifier for the metric
        query_text: The query that was measured
        latency_ms: Time taken to complete retrieval
        relevance_score: Average similarity score of top results
        result_count: Number of results returned
        query_category: Category of the query
        timestamp: When measurement was taken
        success: Whether retrieval completed successfully
    """
    metric_id: str
    query_text: str
    latency_ms: float
    relevance_score: float
    result_count: int
    query_category: str
    timestamp: str
    success: bool


# ==============================================================================
# Custom Exceptions (for T053 - User Story 3)
# ==============================================================================

class QueryValidationError(ValueError):
    """Raised when query validation fails."""
    pass


class EmbeddingError(Exception):
    """Raised when embedding generation fails."""
    pass


class SearchError(Exception):
    """Raised when vector search fails."""
    pass


# ==============================================================================
# T009: Base retrieve_content() Function
# ==============================================================================

def retrieve_content(
    query_text: str,
    client: QdrantClient,
    collection_name: str,
    top_k: int = 5,
    score_threshold: float = 0.7,
    api_key: Optional[str] = None
) -> RetrievalResult:
    """
    Retrieve relevant content chunks from Qdrant based on a query.

    Args:
        query_text: The text query to search for
        client: Qdrant client instance
        collection_name: Name of the Qdrant collection
        top_k: Number of top results to return (default: 5, max: 10)
        score_threshold: Minimum similarity score threshold (default: 0.7)
        api_key: Optional Cohere API key (uses COHERE_API_KEY env var if not provided)

    Returns:
        RetrievalResult containing the matched chunks and metadata

    Raises:
        QueryValidationError: If query validation fails
        EmbeddingError: If embedding generation fails
        SearchError: If vector search fails
    """
    start_time = time.time()

    # T017: Validate query text (empty/whitespace, length <1000 chars)
    if not query_text or query_text.strip() == "":
        raise QueryValidationError("Query text cannot be empty or whitespace only")

    if len(query_text) > 1000:
        raise QueryValidationError(f"Query text too long ({len(query_text)} chars), maximum is 1000 characters")

    # T046: Validate top_k parameter (User Story 3)
    if not isinstance(top_k, int) or top_k < 1 or top_k > 10:
        raise QueryValidationError(f"top_k must be an integer between 1 and 10, got {top_k}")

    # T047: Validate score_threshold parameter (User Story 3)
    if not isinstance(score_threshold, (int, float)) or score_threshold < 0.0 or score_threshold > 1.0:
        raise QueryValidationError(f"score_threshold must be between 0.0 and 1.0, got {score_threshold}")

    # Generate embedding for the query
    logger.info(f"Generating embedding for query: {query_text[:50]}...")
    try:
        query_embedding = generate_embedding(
            query_text,
            api_key=api_key,
            input_type="search_query"
        )
    except Exception as e:
        logger.error(f"Embedding generation failed: {e}")
        raise EmbeddingError(f"Failed to generate embedding: {e}") from e

    # Search in Qdrant
    logger.info(f"Searching for similar content in collection: {collection_name}")
    try:
        results = query_similar(
            client=client,
            collection_name=collection_name,
            query_vector=query_embedding,
            top_k=top_k,
            score_threshold=score_threshold
        )
    except Exception as e:
        logger.error(f"Qdrant search failed: {e}")
        raise SearchError(f"Failed to search in Qdrant: {e}") from e

    # Convert results to RetrievedChunk objects
    retrieved_chunks = []
    current_time = datetime.now().isoformat()

    for point_id, score, payload in results:
        chunk = RetrievedChunk(
            chunk_id=point_id,
            similarity_score=score,
            content_text=payload.get("text", ""),
            source_file=payload.get("source_file", ""),
            module_name=payload.get("module_name", ""),
            section_heading=payload.get("section_heading", ""),
            chunk_index=payload.get("chunk_index", 0),
            total_chunks=payload.get("total_chunks", 1),
            token_count=payload.get("token_count", 0),
            retrieved_at=current_time
        )
        retrieved_chunks.append(chunk)

    retrieval_time_ms = (time.time() - start_time) * 1000

    logger.info(f"Retrieved {len(retrieved_chunks)} chunks in {retrieval_time_ms:.2f}ms")

    return RetrievalResult(
        query_id=f"query_{int(time.time() * 1000)}",
        original_query=query_text,
        retrieved_chunks=retrieved_chunks,
        retrieval_time_ms=retrieval_time_ms,
        total_candidates=len(results),
        retrieval_timestamp=current_time
    )


# ==============================================================================
# Validation Function (for User Story 2)
# ==============================================================================

@retry_with_exponential_backoff(
    max_retries=3,
    base_delay=1.0,
    retryable_exceptions=(Exception,)
)
def validate_retrieval_with_sample_queries(
    client: QdrantClient,
    collection_name: str,
    api_key: Optional[str] = None
) -> List[Tuple[str, Optional[RetrievalResult], bool]]:
    """
    Validate retrieval system with sample queries.

    Args:
        client: Qdrant client instance
        collection_name: Name of the Qdrant collection
        api_key: Optional Cohere API key

    Returns:
        List of (query, result, is_successful) tuples
    """
    sample_queries = [
        "How do I install ROS 2?",
        "What is Gazebo simulation?",
        "Explain VLA models",
        "How to work with humanoid robots?",
        "What are the key concepts in robotics?"
    ]

    results = []
    for query in sample_queries:
        logger.info(f"Validating with sample query: {query}")
        try:
            retrieval_result = retrieve_content(
                query,
                client,
                collection_name,
                top_k=5,
                score_threshold=0.5,  # Use lower threshold for validation to capture more results
                api_key=api_key
            )
            # Check if we got relevant results
            success = len(retrieval_result.retrieved_chunks) > 0
            if success and len(retrieval_result.retrieved_chunks) > 0:
                # Check if at least one result has good similarity
                success = retrieval_result.retrieved_chunks[0].similarity_score >= 0.5
            results.append((query, retrieval_result, success))
        except Exception as e:
            logger.error(f"Validation failed for query '{query}': {e}")
            results.append((query, None, False))

    return results
