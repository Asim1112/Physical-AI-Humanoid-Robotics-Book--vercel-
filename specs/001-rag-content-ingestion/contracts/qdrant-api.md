# Qdrant API Contract

**Feature**: 001-rag-content-ingestion
**API**: Qdrant REST API v1
**Cloud URL**: `https://<cluster-id>.cloud.qdrant.io:6333`
**Documentation**: https://qdrant.tech/documentation/

## Authentication

**Method**: API Key (Header)

```http
api-key: <QDRANT_API_KEY>
```

**Environment Variables**:
- `QDRANT_URL`: Full cluster URL (e.g., `https://xyz.cloud.qdrant.io:6333`)
- `QDRANT_API_KEY`: API key for authentication

## Collection Setup

### 1. Create Collection

**Method**: `PUT`
**Path**: `/collections/{collection_name}`
**Content-Type**: `application/json`

```json
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "optimizers_config": {
    "indexing_threshold": 20000
  },
  "hnsw_config": {
    "m": 16,
    "ef_construct": 100
  }
}
```

#### Request Parameters

| Parameter | Type | Description | Value |
|-----------|------|-------------|-------|
| vectors.size | int | Vector dimensions | 1024 (Cohere embed-english-v3.0) |
| vectors.distance | string | Similarity metric | "Cosine" |
| optimizers_config.indexing_threshold | int | Vectors before indexing | 20000 (optimize for batch uploads) |
| hnsw_config.m | int | Graph connections per node | 16 (balance speed/accuracy) |
| hnsw_config.ef_construct | int | Index construction quality | 100 (good for free tier scale) |

#### Response

**Status**: `200 OK`

```json
{
  "result": true,
  "status": "ok",
  "time": 0.123
}
```

### 2. Upsert Points (Batch)

**Method**: `PUT`
**Path**: `/collections/{collection_name}/points`
**Content-Type**: `application/json`

```json
{
  "points": [
    {
      "id": "uuid-1",
      "vector": [0.123, -0.456, ..., 0.789],
      "payload": {
        "document_id": "module-1-ros2/intro.md",
        "module_name": "module-1-ros2",
        "section_heading": "Introduction to ROS 2",
        "chunk_index": 0,
        "total_chunks": 12,
        "text": "Full chunk text...",
        "token_count": 850,
        "created_at": "2025-12-25T10:00:00Z",
        "model": "embed-english-v3.0"
      }
    }
  ]
}
```

#### Request Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| points | array[object] | Yes | List of vectors to upsert |
| points[].id | string | Yes | UUID for vector |
| points[].vector | array[float] | Yes | 1024-dimensional embedding |
| points[].payload | object | Yes | Metadata for filtering/retrieval |

#### Payload Schema

| Field | Type | Description |
|-------|------|-------------|
| document_id | string | Source file path (relative to frontend/docs/) |
| module_name | string | Module name (e.g., "module-1-ros2") |
| section_heading | string | Section heading from markdown |
| chunk_index | int | Chunk position in file (0-based) |
| total_chunks | int | Total chunks from file |
| text | string | Full chunk text |
| token_count | int | Exact token count |
| created_at | string | ISO 8601 timestamp |
| model | string | Embedding model used |

#### Response

**Status**: `200 OK`

```json
{
  "result": {
    "operation_id": 123,
    "status": "completed"
  },
  "status": "ok",
  "time": 0.456
}
```

### 3. Search Similar Vectors

**Method**: `POST`
**Path**: `/collections/{collection_name}/points/search`
**Content-Type**: `application/json`

```json
{
  "vector": [0.111, -0.222, ..., 0.333],
  "limit": 5,
  "with_payload": true,
  "with_vector": false,
  "score_threshold": 0.7
}
```

#### Request Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| vector | array[float] | Yes | Query embedding (1024 dims) |
| limit | int | No | Top K results (default 10) |
| with_payload | bool | No | Include metadata in results (default false) |
| with_vector | bool | No | Include vectors in results (default false) |
| score_threshold | float | No | Minimum similarity score (0.0-1.0) |
| filter | object | No | Metadata filtering (optional) |

#### Example with Filter

```json
{
  "vector": [0.111, -0.222, ..., 0.333],
  "limit": 5,
  "with_payload": true,
  "filter": {
    "must": [
      {
        "key": "module_name",
        "match": {
          "value": "module-1-ros2"
        }
      }
    ]
  }
}
```

#### Response

**Status**: `200 OK`

```json
{
  "result": [
    {
      "id": "uuid-1",
      "version": 0,
      "score": 0.92,
      "payload": {
        "document_id": "module-1-ros2/intro.md",
        "module_name": "module-1-ros2",
        "section_heading": "Introduction to ROS 2",
        "chunk_index": 0,
        "total_chunks": 12,
        "text": "Full chunk text...",
        "token_count": 850,
        "created_at": "2025-12-25T10:00:00Z",
        "model": "embed-english-v3.0"
      }
    },
    {
      "id": "uuid-2",
      "score": 0.88,
      "payload": { /* ... */ }
    }
  ],
  "status": "ok",
  "time": 0.012
}
```

#### Response Fields

| Field | Type | Description |
|-------|------|-------------|
| result | array[object] | Search results ordered by score |
| result[].id | string | Vector ID |
| result[].score | float | Similarity score (0.0-1.0, higher is better) |
| result[].payload | object | Metadata if with_payload=true |
| time | float | Query execution time (seconds) |

## Error Responses

### 400 Bad Request

```json
{
  "status": {
    "error": "Wrong input: Vector dimension mismatch. Expected 1024, got 512"
  },
  "time": 0.001
}
```

**Handling**: Log error and terminate (indicates bug in embedding generation).

### 401 Unauthorized

```json
{
  "status": {
    "error": "Unauthorized"
  },
  "time": 0.001
}
```

**Handling**: Fail immediately, check QDRANT_API_KEY.

### 404 Not Found

```json
{
  "status": {
    "error": "Collection 'xyz' not found"
  },
  "time": 0.001
}
```

**Handling**: Create collection if missing, retry operation.

### 503 Service Unavailable

```json
{
  "status": {
    "error": "Service temporarily unavailable"
  },
  "time": 0.001
}
```

**Handling**: Retry with exponential backoff (max 3 retries).

## Rate Limits (Free Tier)

- **Cluster RAM**: 1 GB
- **Max vectors**: ~1M (1024-dim vectors with metadata)
- **Request rate**: No hard limit, but throttling on sustained high load
- **Concurrent connections**: ~10

### Batch Processing Strategy

```python
def batch_upsert(points: List[QdrantPoint], batch_size: int = 100) -> None:
    """
    Upload vectors in batches to avoid timeouts.

    Strategy:
    - Batch size: 100 points per request
    - Estimated: 350 points = 4 batches = ~2 seconds
    - No explicit delay needed (well under rate limits)
    """
```

## Example Usage (Python SDK)

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

def setup_qdrant_collection(url: str, api_key: str, collection_name: str) -> None:
    """
    Create Qdrant collection with proper configuration.

    Args:
        url: Qdrant cluster URL
        api_key: API key for authentication
        collection_name: Name for collection

    Raises:
        RuntimeError: Collection creation failed
    """
    client = QdrantClient(url=url, api_key=api_key)

    # Check if collection exists
    collections = client.get_collections().collections
    if collection_name in [c.name for c in collections]:
        print(f"Collection '{collection_name}' already exists")
        return

    # Create collection
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
        optimizers_config={
            "indexing_threshold": 20000
        },
        hnsw_config={
            "m": 16,
            "ef_construct": 100
        }
    )

def upsert_vectors(
    url: str,
    api_key: str,
    collection_name: str,
    points: List[dict]
) -> None:
    """
    Upload vectors to Qdrant in batches.

    Args:
        url: Qdrant cluster URL
        api_key: API key
        collection_name: Target collection
        points: List of {id, vector, payload} dicts

    Raises:
        RuntimeError: Upsert operation failed
    """
    client = QdrantClient(url=url, api_key=api_key)

    # Convert to PointStruct objects
    qdrant_points = [
        PointStruct(
            id=p["id"],
            vector=p["vector"],
            payload=p["payload"]
        )
        for p in points
    ]

    # Batch upsert
    batch_size = 100
    for i in range(0, len(qdrant_points), batch_size):
        batch = qdrant_points[i:i+batch_size]
        client.upsert(
            collection_name=collection_name,
            points=batch
        )

def search_similar(
    url: str,
    api_key: str,
    collection_name: str,
    query_vector: List[float],
    top_k: int = 5,
    score_threshold: float = 0.7
) -> List[dict]:
    """
    Search for similar vectors.

    Args:
        url: Qdrant cluster URL
        api_key: API key
        collection_name: Collection to search
        query_vector: 1024-dim query embedding
        top_k: Number of results
        score_threshold: Minimum similarity score

    Returns:
        List of {id, score, payload} dicts
    """
    client = QdrantClient(url=url, api_key=api_key)

    results = client.search(
        collection_name=collection_name,
        query_vector=query_vector,
        limit=top_k,
        score_threshold=score_threshold,
        with_payload=True,
        with_vectors=False
    )

    return [
        {
            "id": r.id,
            "score": r.score,
            "payload": r.payload
        }
        for r in results
    ]
```

## Testing Contract

### Unit Test Mock

```python
# Mock Qdrant search response
mock_search_results = [
    {
        "id": "uuid-1",
        "score": 0.92,
        "payload": {
            "document_id": "module-1-ros2/intro.md",
            "text": "Sample chunk text",
            "module_name": "module-1-ros2"
        }
    }
]
```

### Integration Test

```python
def test_qdrant_collection_setup():
    """
    Test Qdrant collection creation.

    Prerequisites:
    - QDRANT_URL and QDRANT_API_KEY environment variables set

    Validates:
    - Collection created successfully
    - Vector config correct (1024 dims, cosine)
    """
    setup_qdrant_collection(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        collection_name="test-collection"
    )

    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    info = client.get_collection("test-collection")

    assert info.config.params.vectors.size == 1024
    assert info.config.params.vectors.distance == Distance.COSINE
```

## Contract Compliance Checklist

- [ ] Collection created with 1024-dim vectors, cosine distance
- [ ] HNSW config set (m=16, ef_construct=100)
- [ ] All points include required payload fields
- [ ] Vector dimensions validated before upsert (1024 floats)
- [ ] Batch size ≤ 100 points per request
- [ ] Error handling for 400, 401, 404, 503
- [ ] API key and URL loaded from environment variables
- [ ] Search results include payload (with_payload=true)
- [ ] Score threshold applied (0.7 minimum for verification)

## Security Notes

- **Never log API keys** - redact in logs
- **Use environment variables** - never hardcode credentials
- **Monitor storage usage** - free tier has 1GB RAM limit
- **Backup strategy**: Export vectors periodically for disaster recovery

## Collection Name

**Production**: `humanoid-robotics-textbook`
**Testing**: `test-humanoid-robotics-textbook`

Use separate collections for testing to avoid polluting production data.
