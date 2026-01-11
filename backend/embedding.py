"""Cohere API integration for generating embeddings."""

import logging
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional

import cohere
from cohere.core.api_error import ApiError

from utils import retry_with_exponential_backoff

logger = logging.getLogger("rag_ingestion")


# ==============================================================================
# T027: EmbeddingVector Dataclass
# ==============================================================================

@dataclass
class EmbeddingVector:
    """
    Represents an embedding vector with metadata.

    Attributes per data-model.md specification.
    """
    vector_id: str  # Same as chunk_id for 1:1 mapping
    chunk_id: str
    vector: List[float]  # 1024-dimensional for embed-english-v3.0
    model_name: str = "embed-english-v3.0"
    input_type: str = "search_document"
    created_at: str = field(default_factory=lambda: datetime.utcnow().isoformat() + "Z")

    def __post_init__(self):
        """Validate embedding after initialization."""
        if len(self.vector) != 1024:
            raise ValueError(
                f"Invalid vector dimension: {len(self.vector)} (expected 1024 for {self.model_name})"
            )

        # Check if all values are floats
        if not all(isinstance(v, (float, int)) for v in self.vector):
            raise TypeError("Vector must contain only numeric values")


# ==============================================================================
# T025: Generate Embedding (Single)
# ==============================================================================

@retry_with_exponential_backoff(
    max_retries=3,
    base_delay=1.0,
    retryable_exceptions=(ApiError,)
)
def generate_embedding(
    text: str,
    api_key: Optional[str] = None,
    model: str = "embed-english-v3.0",
    input_type: str = "search_document"
) -> List[float]:
    """
    Generate embedding for a single text using Cohere API.

    Args:
        text: Text to embed (must be <512 tokens)
        api_key: Cohere API key (defaults to COHERE_API_KEY env var)
        model: Embedding model name
        input_type: Type of embedding (search_document for content, search_query for queries)

    Returns:
        1024-dimensional embedding vector

    Raises:
        ApiError: If API call fails after retries
        ValueError: If API key is missing or text is too long

    Example:
        embedding = generate_embedding("Sample text for embedding")
        print(f"Embedding dimension: {len(embedding)}")
    """
    if not text.strip():
        raise ValueError("Cannot generate embedding for empty text")

    # Get API key
    if api_key is None:
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY not found in environment")

    # Initialize Cohere client
    client = cohere.Client(api_key)

    try:
        response = client.embed(
            texts=[text],
            model=model,
            input_type=input_type,
            truncate="NONE"  # Fail if text is too long (we pre-chunk to avoid this)
        )

        # Extract embedding
        embedding = response.embeddings[0]

        # Validate dimension
        if len(embedding) != 1024:
            raise ValueError(f"Unexpected embedding dimension: {len(embedding)} (expected 1024)")

        # Log embedding generation (skip meta details due to SDK changes)
        logger.debug(f"Generated embedding: {len(embedding)} dims for input_type={input_type}")

        return embedding

    except ApiError as e:
        logger.error(f"Cohere API error: {e}")
        raise


# ==============================================================================
# T026: Batch Embed with Rate Limit Handling
# ==============================================================================

def batch_embed(
    texts: List[str],
    api_key: Optional[str] = None,
    model: str = "embed-english-v3.0",
    input_type: str = "search_document",
    batch_size: int = 96,  # Cohere limit is 96
    rate_limit_delay: float = 0.6  # 600ms between batches (100 calls/min = 1 call per 600ms)
) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batches with rate limiting.

    Args:
        texts: List of texts to embed
        api_key: Cohere API key
        model: Embedding model name
        input_type: Type of embedding
        batch_size: Maximum texts per batch (Cohere limit: 96)
        rate_limit_delay: Delay between batches in seconds

    Returns:
        List of 1024-dimensional embedding vectors

    Strategy per research.md:
        - Batch size: 96 (Cohere limit)
        - Rate: 100 calls/min (free tier)
        - Delay: 600ms between batches
        - Retry on rate limit errors

    Example:
        chunks = ["text 1", "text 2", "text 3"]
        embeddings = batch_embed(chunks)
        print(f"Generated {len(embeddings)} embeddings")
    """
    if not texts:
        logger.warning("Empty text list provided for batch embedding")
        return []

    # Get API key
    if api_key is None:
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY not found in environment")

    # Initialize Cohere client
    client = cohere.Client(api_key)

    all_embeddings = []
    total_batches = (len(texts) + batch_size - 1) // batch_size

    logger.info(
        f"Batch embedding {len(texts)} texts in {total_batches} batches "
        f"(batch_size={batch_size})"
    )

    for batch_idx in range(total_batches):
        start_idx = batch_idx * batch_size
        end_idx = min(start_idx + batch_size, len(texts))
        batch = texts[start_idx:end_idx]

        logger.info(f"Processing batch {batch_idx + 1}/{total_batches} ({len(batch)} texts)...")

        try:
            # Use retry decorator by wrapping the call
            embeddings = _embed_batch_with_retry(client, batch, model, input_type)
            all_embeddings.extend(embeddings)

            logger.info(
                f"[OK] Batch {batch_idx + 1}/{total_batches} complete "
                f"({len(embeddings)} embeddings)"
            )

        except ApiError as e:
            logger.error(f"Batch {batch_idx + 1} failed after retries: {e}")
            raise

        # Rate limiting: delay between batches (except last batch)
        if batch_idx < total_batches - 1:
            logger.debug(f"Rate limit delay: {rate_limit_delay}s")
            time.sleep(rate_limit_delay)

    logger.info(f"[OK] Batch embedding complete: {len(all_embeddings)} total embeddings")

    return all_embeddings


@retry_with_exponential_backoff(
    max_retries=3,
    base_delay=1.0,
    retryable_exceptions=(ApiError,)
)
def _embed_batch_with_retry(
    client: cohere.Client,
    texts: List[str],
    model: str,
    input_type: str
) -> List[List[float]]:
    """
    Internal function to embed batch with retry logic.

    Args:
        client: Cohere client instance
        texts: Batch of texts
        model: Model name
        input_type: Input type

    Returns:
        List of embeddings
    """
    response = client.embed(
        texts=texts,
        model=model,
        input_type=input_type,
        truncate="NONE"
    )

    # Validate embeddings
    embeddings = response.embeddings

    if len(embeddings) != len(texts):
        raise ValueError(
            f"Embedding count mismatch: got {len(embeddings)}, expected {len(texts)}"
        )

    # Validate dimensions
    for idx, embedding in enumerate(embeddings):
        if len(embedding) != 1024:
            raise ValueError(
                f"Invalid dimension for embedding {idx}: {len(embedding)} (expected 1024)"
            )

    return embeddings


# ==============================================================================
# High-Level Embedding Function
# ==============================================================================

def embed_chunks(chunks: List, api_key: Optional[str] = None) -> List[EmbeddingVector]:
    """
    Generate embeddings for a list of DocumentChunk objects.

    Args:
        chunks: List of DocumentChunk objects
        api_key: Cohere API key

    Returns:
        List of EmbeddingVector objects

    This function orchestrates T025-T027:
        1. Extract text from chunks
        2. Filter chunks > 508 tokens (Cohere API limit)
        3. Batch embed using Cohere API (T026)
        4. Create EmbeddingVector objects (T027)
    """
    if not chunks:
        logger.warning("No chunks provided for embedding")
        return []

    # Filter chunks that exceed Cohere's 508-token limit
    # Use 350 as threshold to account for tokenizer differences between tiktoken and Cohere (up to 30% variance observed)
    # With new chunking max of 300, this should allow nearly all chunks through
    logger.info(f"Pre-filter: {len(chunks)} chunks")
    logger.info(f"Token counts: min={min(c.token_count for c in chunks)}, max={max(c.token_count for c in chunks)}, avg={sum(c.token_count for c in chunks)//len(chunks)}")

    filtered_chunks = [c for c in chunks if c.token_count <= 350]
    skipped_count = len(chunks) - len(filtered_chunks)

    logger.info(f"Post-filter: {len(filtered_chunks)} chunks (skipped {skipped_count})")

    if skipped_count > 0:
        logger.warning(f"Skipping {skipped_count} chunks that exceed 350-token safety threshold (Cohere limit: 508, ~30% tokenizer variance)")
        logger.warning(f"Processing {len(filtered_chunks)}/{len(chunks)} chunks for embedding")

    if not filtered_chunks:
        logger.error("No chunks remain after filtering for token limit")
        return []

    # Extract text from filtered chunks
    texts = [chunk.text for chunk in filtered_chunks]
    chunk_ids = [chunk.chunk_id for chunk in filtered_chunks]

    logger.info(f"Embedding {len(filtered_chunks)} chunks...")

    # T026: Batch embed
    embeddings = batch_embed(texts, api_key=api_key)

    if len(embeddings) != len(filtered_chunks):
        raise ValueError(
            f"Embedding count mismatch: {len(embeddings)} embeddings for {len(filtered_chunks)} chunks"
        )

    # T027: Create EmbeddingVector objects
    embedding_vectors = []
    for chunk_id, embedding in zip(chunk_ids, embeddings):
        vector = EmbeddingVector(
            vector_id=chunk_id,  # 1:1 mapping with chunk
            chunk_id=chunk_id,
            vector=embedding
        )
        embedding_vectors.append(vector)

    logger.info(f"[OK] Created {len(embedding_vectors)} EmbeddingVector objects")

    return embedding_vectors
