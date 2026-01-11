# Cohere API Contract

**Feature**: 001-rag-content-ingestion
**API**: Cohere Embeddings API v1
**Base URL**: `https://api.cohere.ai/v1`
**Documentation**: https://docs.cohere.com/reference/embed

## Authentication

**Method**: Bearer Token (API Key)

```http
Authorization: Bearer <COHERE_API_KEY>
```

**Environment Variable**: `COHERE_API_KEY`

## Endpoint: Embed Text

### Request

**Method**: `POST`
**Path**: `/embed`
**Content-Type**: `application/json`

```json
{
  "texts": [
    "Text chunk 1 to embed",
    "Text chunk 2 to embed"
  ],
  "model": "embed-english-v3.0",
  "input_type": "search_document",
  "truncate": "NONE"
}
```

### Request Parameters

| Parameter | Type | Required | Description | Constraints |
|-----------|------|----------|-------------|-------------|
| texts | array[string] | Yes | List of text chunks to embed | Max 96 texts per request, max 512 tokens per text |
| model | string | Yes | Embedding model ID | "embed-english-v3.0" |
| input_type | string | Yes | Type of embedding | "search_document" for content, "search_query" for queries |
| truncate | string | No | Truncation strategy | "NONE" (we pre-validate token counts) |

### Response

**Status**: `200 OK`
**Content-Type**: `application/json`

```json
{
  "id": "req-xyz123",
  "texts": [
    "Text chunk 1 to embed",
    "Text chunk 2 to embed"
  ],
  "embeddings": [
    [0.123, -0.456, 0.789, ...],  // 1024 floats
    [0.321, -0.654, 0.987, ...]   // 1024 floats
  ],
  "meta": {
    "api_version": {
      "version": "1"
    },
    "billed_units": {
      "input_tokens": 100
    }
  }
}
```

### Response Fields

| Field | Type | Description |
|-------|------|-------------|
| id | string | Request ID for tracking |
| texts | array[string] | Echo of input texts |
| embeddings | array[array[float]] | Embedding vectors (1024 dimensions each) |
| meta.billed_units.input_tokens | int | Tokens billed for this request |

### Error Responses

#### 400 Bad Request

```json
{
  "message": "invalid request: text is too long. max tokens: 512, actual tokens: 600"
}
```

**Handling**: Should not occur (we pre-chunk to <512 tokens). Log and skip if encountered.

#### 429 Too Many Requests

```json
{
  "message": "rate limit exceeded"
}
```

**Handling**: Implement exponential backoff (see research.md retry strategy).

#### 401 Unauthorized

```json
{
  "message": "invalid api token"
}
```

**Handling**: Fail immediately, log error with API key format check.

#### 500 Internal Server Error

```json
{
  "message": "internal server error"
}
```

**Handling**: Retry with exponential backoff (max 3 retries).

## Rate Limits

### Free Tier

- **Requests per minute**: 100
- **Monthly API calls**: 100,000
- **Concurrent requests**: 5

### Batch Processing Strategy

```python
def batch_embed(chunks: List[str]) -> List[List[float]]:
    """
    Embed chunks in batches respecting rate limits.

    Strategy:
    - Batch size: 96 chunks (Cohere limit)
    - Rate: 100 requests/min
    - Processing: ~9600 chunks/min (optimal)
    - Delay: 600ms between batches (safety margin)
    - Estimated time: 350 chunks ≈ 4 batches ≈ 3 seconds
    """
```

## Example Usage (Python SDK)

```python
import cohere
from typing import List

def embed_chunks(chunks: List[str], api_key: str) -> List[List[float]]:
    """
    Generate embeddings for document chunks.

    Args:
        chunks: List of text chunks (pre-validated <512 tokens)
        api_key: Cohere API key

    Returns:
        List of 1024-dimensional embedding vectors

    Raises:
        ValueError: Invalid API key or request
        RuntimeError: API error after retries
    """
    co = cohere.Client(api_key)

    try:
        response = co.embed(
            texts=chunks,
            model="embed-english-v3.0",
            input_type="search_document",
            truncate="NONE"
        )
        return response.embeddings
    except cohere.CohereAPIError as e:
        # Handle specific error types
        if e.status_code == 429:
            # Rate limit - retry with backoff
            raise RuntimeError("Rate limit exceeded") from e
        elif e.status_code == 401:
            # Auth error - fail immediately
            raise ValueError("Invalid Cohere API key") from e
        else:
            # Other errors - log and retry
            raise RuntimeError(f"Cohere API error: {e}") from e
```

## Testing Contract

### Unit Test Mock

```python
# Mock Cohere response for testing
mock_response = {
    "embeddings": [
        [0.1] * 1024  # Mock 1024-dim vector
    ],
    "meta": {
        "billed_units": {"input_tokens": 100}
    }
}
```

### Integration Test

```python
def test_cohere_embedding_integration():
    """
    Test actual Cohere API call with sample text.

    Prerequisites:
    - COHERE_API_KEY environment variable set
    - Network connectivity

    Validates:
    - Response contains embeddings
    - Embedding dimension is 1024
    - All values are floats
    """
    chunks = ["This is a test chunk for embedding."]
    embeddings = embed_chunks(chunks, os.getenv("COHERE_API_KEY"))

    assert len(embeddings) == 1
    assert len(embeddings[0]) == 1024
    assert all(isinstance(v, float) for v in embeddings[0])
```

## Contract Compliance Checklist

- [ ] All requests include Authorization header
- [ ] `model` parameter is "embed-english-v3.0"
- [ ] `input_type` is "search_document" for content chunks
- [ ] Text chunks are <512 tokens (pre-validated)
- [ ] Batch size ≤ 96 chunks
- [ ] Rate limit handling implemented (exponential backoff)
- [ ] Error responses handled (400, 401, 429, 500)
- [ ] Response embeddings validated (length == 1024)
- [ ] API key loaded from environment variable

## Security Notes

- **Never log API keys** - redact in logs
- **Rotate keys regularly** - especially after public demos
- **Use environment variables** - never hardcode in source
- **Monitor usage** - track API calls to avoid unexpected charges on paid tier
