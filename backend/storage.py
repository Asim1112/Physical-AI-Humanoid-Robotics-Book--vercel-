"""Qdrant vector database integration for storing and querying embeddings."""

import logging
import os
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse

from utils import retry_with_exponential_backoff

logger = logging.getLogger("rag_ingestion")


# ==============================================================================
# T034: QdrantPoint Dataclass
# ==============================================================================

@dataclass
class QdrantPoint:
    """
    Represents a Qdrant point with vector and payload.

    Attributes per data-model.md specification.
    """
    point_id: str  # UUID for the point
    vector: List[float]  # 1024-dimensional embedding
    payload: Dict  # Metadata fields

    def to_qdrant_point(self) -> models.PointStruct:
        """Convert to Qdrant PointStruct for upsertion."""
        return models.PointStruct(
            id=self.point_id,
            vector=self.vector,
            payload=self.payload
        )


# ==============================================================================
# T031: Setup Qdrant Collection
# ==============================================================================

@retry_with_exponential_backoff(
    max_retries=3,
    base_delay=1.0,
    retryable_exceptions=(UnexpectedResponse,)
)
def setup_qdrant_collection(
    client: QdrantClient,
    collection_name: str,
    vector_size: int = 1024,
    distance: str = "Cosine",
    recreate: bool = False
) -> bool:
    """
    Setup Qdrant collection with proper configuration.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection
        vector_size: Dimension of vectors (1024 for embed-english-v3.0)
        distance: Distance metric (Cosine, Dot, Euclidean)
        recreate: If True, delete existing collection and recreate

    Returns:
        True if successful, False otherwise

    Configuration per research.md:
        - Vector size: 1024 (Cohere embed-english-v3.0)
        - Distance: Cosine similarity
        - HNSW config: m=16, ef_construct=100
    """
    try:
        # Check if collection exists
        collections = client.get_collections()
        collection_exists = any(c.name == collection_name for c in collections.collections)

        if collection_exists:
            if recreate:
                logger.info(f"Deleting existing collection: {collection_name}")
                client.delete_collection(collection_name)
            else:
                logger.info(f"Collection '{collection_name}' already exists, skipping creation")
                return True

        # Create collection with HNSW configuration
        logger.info(f"Creating collection '{collection_name}' with {vector_size}-dim {distance} vectors")

        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=models.Distance.COSINE if distance == "Cosine" else models.Distance.DOT
            ),
            hnsw_config=models.HnswConfigDiff(
                m=16,  # Number of edges per node
                ef_construct=100,  # Construction time/quality tradeoff
            )
        )

        logger.info(f"[OK] Collection '{collection_name}' created successfully")
        return True

    except UnexpectedResponse as e:
        logger.error(f"Qdrant API error during collection setup: {e}")
        raise
    except Exception as e:
        logger.error(f"Failed to setup collection: {e}", exc_info=True)
        return False


# ==============================================================================
# T032: Upsert Vectors with Batching
# ==============================================================================

def upsert_vectors(
    client: QdrantClient,
    collection_name: str,
    points: List[QdrantPoint],
    batch_size: int = 100
) -> int:
    """
    Upsert vectors to Qdrant collection in batches.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection
        points: List of QdrantPoint objects
        batch_size: Points per batch (default 100)

    Returns:
        Number of points successfully upserted

    Strategy per research.md:
        - Batch size: 100 points per request
        - Retry on failures with exponential backoff
        - Track success/failure counts
    """
    if not points:
        logger.warning("No points provided for upsert")
        return 0

    total_batches = (len(points) + batch_size - 1) // batch_size
    total_upserted = 0

    logger.info(
        f"Upserting {len(points)} points in {total_batches} batches "
        f"(batch_size={batch_size})"
    )

    for batch_idx in range(total_batches):
        start_idx = batch_idx * batch_size
        end_idx = min(start_idx + batch_size, len(points))
        batch = points[start_idx:end_idx]

        logger.info(f"Processing batch {batch_idx + 1}/{total_batches} ({len(batch)} points)...")

        try:
            # Convert to Qdrant PointStruct
            qdrant_points = [p.to_qdrant_point() for p in batch]

            # Upsert with retry
            _upsert_batch_with_retry(client, collection_name, qdrant_points)

            total_upserted += len(batch)
            logger.info(f"[OK] Batch {batch_idx + 1}/{total_batches} complete ({len(batch)} points)")

        except Exception as e:
            logger.error(f"Batch {batch_idx + 1} failed: {e}")
            # Continue with next batch instead of failing entire operation
            continue

    logger.info(f"[OK] Upsert complete: {total_upserted}/{len(points)} points stored")

    return total_upserted


@retry_with_exponential_backoff(
    max_retries=3,
    base_delay=1.0,
    retryable_exceptions=(UnexpectedResponse,)
)
def _upsert_batch_with_retry(
    client: QdrantClient,
    collection_name: str,
    points: List[models.PointStruct]
) -> None:
    """
    Internal function to upsert batch with retry logic.

    Args:
        client: Qdrant client instance
        collection_name: Collection name
        points: List of PointStruct objects
    """
    client.upsert(
        collection_name=collection_name,
        points=points,
        wait=True  # Wait for indexing to complete
    )


# ==============================================================================
# T033: Query Similar Vectors
# ==============================================================================

def query_similar(
    client: QdrantClient,
    collection_name: str,
    query_vector: List[float],
    top_k: int = 5,
    score_threshold: float = 0.7,
    filter_dict: Optional[Dict] = None
) -> List[Tuple[str, float, Dict]]:
    """
    Query Qdrant for similar vectors.

    Args:
        client: Qdrant client instance
        collection_name: Collection name
        query_vector: 1024-dimensional query embedding
        top_k: Number of results to return (default 5)
        score_threshold: Minimum similarity score (default 0.7)
        filter_dict: Optional filter conditions (e.g., {"module_name": "module-1-ros2"})

    Returns:
        List of (point_id, score, payload) tuples

    Example:
        results = query_similar(client, "textbook", query_embedding, top_k=5)
        for point_id, score, payload in results:
            print(f"Score: {score:.3f} - {payload['source_file']}")
    """
    try:
        # Build filter if provided
        query_filter = None
        if filter_dict:
            conditions = []
            for key, value in filter_dict.items():
                conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )
            query_filter = models.Filter(must=conditions)

        # Execute search using updated Qdrant API
        search_results = client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=top_k,
            score_threshold=score_threshold,
            query_filter=query_filter,
            with_payload=True
        ).points

        # Convert to tuples
        results = [
            (str(hit.id), hit.score, hit.payload)
            for hit in search_results
        ]

        logger.debug(
            f"Query returned {len(results)} results "
            f"(threshold={score_threshold}, top_k={top_k})"
        )

        return results

    except Exception as e:
        logger.error(f"Query failed: {e}", exc_info=True)
        return []


# ==============================================================================
# Helper Functions
# ==============================================================================

def create_qdrant_client(url: Optional[str] = None, api_key: Optional[str] = None) -> QdrantClient:
    """
    Create Qdrant client from environment variables or provided credentials.

    Args:
        url: Qdrant URL (defaults to QDRANT_URL env var)
        api_key: Qdrant API key (defaults to QDRANT_API_KEY env var)

    Returns:
        QdrantClient instance

    Example:
        client = create_qdrant_client()
    """
    if url is None:
        url = os.getenv("QDRANT_URL")
        if not url:
            raise ValueError("QDRANT_URL not found in environment")

    if api_key is None:
        api_key = os.getenv("QDRANT_API_KEY")
        if not api_key:
            raise ValueError("QDRANT_API_KEY not found in environment")

    # Extract host and port from URL
    # URL format: https://host:port or https://host
    if url.startswith("http://") or url.startswith("https://"):
        # Cloud or remote instance
        client = QdrantClient(
            url=url,
            api_key=api_key,
            timeout=30
        )
    else:
        # Local instance
        parts = url.split(":")
        host = parts[0]
        port = int(parts[1]) if len(parts) > 1 else 6333
        client = QdrantClient(host=host, port=port)

    logger.info(f"[OK] Qdrant client connected to {url}")

    return client


def convert_chunks_to_points(chunks: List, embeddings: List) -> List[QdrantPoint]:
    """
    Convert DocumentChunk and EmbeddingVector objects to QdrantPoint objects.

    Args:
        chunks: List of DocumentChunk objects
        embeddings: List of EmbeddingVector objects

    Returns:
        List of QdrantPoint objects

    Payload schema per data-model.md:
        - chunk_id: str
        - source_file: str
        - module_name: str
        - section_heading: str
        - chunk_index: int
        - total_chunks: int
        - text: str (full chunk text)
        - token_count: int
        - created_at: str (ISO 8601)
    """
    if len(chunks) != len(embeddings):
        raise ValueError(
            f"Chunk count ({len(chunks)}) must match embedding count ({len(embeddings)})"
        )

    points = []

    for chunk, embedding in zip(chunks, embeddings):
        # Validate chunk_id matches
        if chunk.chunk_id != embedding.chunk_id:
            raise ValueError(
                f"Chunk ID mismatch: {chunk.chunk_id} != {embedding.chunk_id}"
            )

        # Create payload with all metadata
        payload = {
            "chunk_id": chunk.chunk_id,
            "source_file": chunk.source_file,
            "module_name": chunk.module_name,
            "section_heading": chunk.section_heading,
            "chunk_index": chunk.chunk_index,
            "total_chunks": chunk.total_chunks,
            "text": chunk.text,
            "token_count": chunk.token_count,
            "created_at": chunk.created_at,
            "model_name": embedding.model_name,
            "input_type": embedding.input_type
        }

        # Create QdrantPoint
        point = QdrantPoint(
            point_id=embedding.vector_id,
            vector=embedding.vector,
            payload=payload
        )

        points.append(point)

    logger.debug(f"Converted {len(points)} chunks to Qdrant points")

    return points
