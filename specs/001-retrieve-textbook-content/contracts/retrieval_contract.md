# API Contract: Textbook Content Retrieval

**Feature**: 001-retrieve-textbook-content
**Date**: 2025-12-25

## Overview

API contract for retrieving textbook content from Qdrant vector database using Cohere embeddings.

## Endpoints

### POST /api/retrieve

Retrieve relevant textbook content based on a query.

#### Request

**Path**: `/api/retrieve`

**Method**: `POST`

**Headers**:
- `Content-Type: application/json`
- `Authorization: Bearer {cohere_api_key}` (if needed)

**Body**:
```json
{
  "query": "string (required) - The search query text",
  "top_k": "integer (optional, default: 5) - Number of results to return (max: 10)",
  "score_threshold": "number (optional, default: 0.7) - Minimum similarity score",
  "query_type": "string (optional) - Category of query for analytics"
}
```

**Example Request**:
```json
{
  "query": "How do I install ROS 2?",
  "top_k": 5,
  "score_threshold": 0.7
}
```

#### Response

**Success Response (200 OK)**:
```json
{
  "query_id": "string - Unique identifier for the query",
  "original_query": "string - The original query text",
  "retrieval_time_ms": "number - Time taken for retrieval in milliseconds",
  "total_candidates": "integer - Total number of candidates considered",
  "retrieved_chunks": [
    {
      "chunk_id": "string - Unique identifier for the content chunk",
      "similarity_score": "number - Cosine similarity score (0.0-1.0)",
      "content_text": "string - The actual content text",
      "source_file": "string - Original markdown file path",
      "module_name": "string - Module name derived from file path",
      "section_heading": "string - Section heading from original content",
      "chunk_index": "integer - Position of chunk in original document",
      "total_chunks": "integer - Total chunks in original document",
      "token_count": "integer - Number of tokens in the chunk"
    }
  ]
}
```

**Example Success Response**:
```json
{
  "query_id": "query_1703523900",
  "original_query": "How do I install ROS 2?",
  "retrieval_time_ms": 423.5,
  "total_candidates": 12,
  "retrieved_chunks": [
    {
      "chunk_id": "chunk-abc123",
      "similarity_score": 0.856,
      "content_text": "To install ROS 2, first ensure your system meets the requirements...",
      "source_file": "module-1-ros2/installation.md",
      "module_name": "module-1-ros2",
      "section_heading": "Installing ROS 2",
      "chunk_index": 0,
      "total_chunks": 3,
      "token_count": 245
    }
  ]
}
```

**Error Response (400 Bad Request)**:
```json
{
  "error": "string - Error message",
  "code": "string - Error code",
  "details": "object - Additional error details"
}
```

**Example Error Response**:
```json
{
  "error": "Query text is required",
  "code": "MISSING_QUERY",
  "details": {
    "field": "query"
  }
}
```

**Error Response (500 Internal Server Error)**:
```json
{
  "error": "string - Error message",
  "code": "string - Error code",
  "details": "object - Additional error details"
}
```

#### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| MISSING_QUERY | 400 | Query parameter is required |
| INVALID_QUERY_LENGTH | 400 | Query is too long (max 1000 characters) |
| INVALID_TOP_K | 400 | top_k must be between 1 and 10 |
| INVALID_SCORE_THRESHOLD | 400 | score_threshold must be between 0.0 and 1.0 |
| EMBEDDING_ERROR | 500 | Error occurred while generating embedding |
| SEARCH_ERROR | 500 | Error occurred while searching in Qdrant |
| CONNECTION_ERROR | 500 | Error connecting to Qdrant or Cohere |

## Validation Rules

### Request Validation
- `query` field is required and must not be empty
- `query` length must be between 1 and 1000 characters
- `top_k` must be an integer between 1 and 10
- `score_threshold` must be a number between 0.0 and 1.0
- `query_type` (if provided) must be a valid string

### Response Validation
- `query_id` must be a non-empty string
- `retrieval_time_ms` must be a positive number
- `total_candidates` must be a non-negative integer
- `retrieved_chunks` must be an array (can be empty)
- Each chunk must have valid similarity_score between 0.0 and 1.0
- Each chunk must have non-empty content_text

## Performance Requirements

- Response time should be under 2000ms for 95% of requests
- Must handle concurrent requests
- Should return results within 2 seconds even under load

## Security Requirements

- API keys must be properly validated
- Input must be sanitized to prevent injection attacks
- Sensitive information should not be exposed in error messages

## Compatibility

- Should work with Cohere embed-english-v3.0 model
- Should work with Qdrant Cloud and self-hosted instances
- Should maintain compatibility with existing embedding schema from Spec 1