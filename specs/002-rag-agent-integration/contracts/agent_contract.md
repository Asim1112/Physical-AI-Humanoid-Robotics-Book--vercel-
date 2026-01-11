# API Contract: RAG Agent Integration

## Agent Query Endpoint

### POST /api/agent/query

Initiates a query with the RAG agent, processing the user input through retrieval and response generation.

**Request**:
```json
{
  "query_text": "string (required, max 1000 chars)",
  "selected_text": "string (optional)",
  "session_id": "string (optional)",
  "parameters": {
    "temperature": "float (optional, 0.0-1.0, default 0.5)",
    "top_k": "int (optional, 1-10, default 5)",
    "score_threshold": "float (optional, 0.0-1.0, default 0.5)",
    "max_turns": "int (optional, 1-50, default 10)"
  }
}
```

**Response** (Success 200):
```json
{
  "conversation_id": "string",
  "response_text": "string",
  "retrieved_chunks": [
    {
      "chunk_id": "string",
      "similarity_score": "float",
      "content_text": "string",
      "source_file": "string",
      "module_name": "string",
      "section_heading": "string",
      "chunk_index": "int",
      "total_chunks": "int",
      "token_count": "int",
      "retrieved_at": "string (ISO timestamp)"
    }
  ],
  "response_time_ms": "float",
  "query_tokens": "int",
  "response_tokens": "int",
  "retrieval_success": "bool"
}
```

**Response** (Error 400):
```json
{
  "error": "string",
  "details": "string"
}
```

**Response** (Error 500):
```json
{
  "error": "string",
  "code": "string"
}
```

## Agent Conversation Endpoint

### POST /api/agent/conversation

Continues an existing conversation with the agent, maintaining context from previous turns.

**Request**:
```json
{
  "conversation_id": "string (required)",
  "query_text": "string (required, max 1000 chars)",
  "selected_text": "string (optional)"
}
```

**Response**: Same as Query endpoint

## Agent Session Management

### GET /api/agent/session/{session_id}

Retrieves information about a specific conversation session.

**Response** (Success 200):
```json
{
  "session_id": "string",
  "created_at": "string (ISO timestamp)",
  "last_accessed": "string (ISO timestamp)",
  "turn_count": "int",
  "active": "bool"
}
```

### DELETE /api/agent/session/{session_id}

Ends a conversation session and cleans up resources.

**Response** (Success 200):
```json
{
  "message": "string",
  "session_id": "string",
  "status": "ended"
}
```

## Health Check

### GET /api/agent/health

Checks the health status of the agent service.

**Response** (Success 200):
```json
{
  "status": "healthy",
  "timestamp": "string (ISO timestamp)",
  "dependencies": {
    "qdrant": "bool",
    "cohere": "bool",
    "gemini_api": "bool"
  }
}
```

## Validation Rules

- `query_text` must be non-empty and not exceed 1000 characters
- `temperature` must be between 0.0 and 1.0
- `top_k` must be between 1 and 10
- `score_threshold` must be between 0.0 and 1.0
- `max_turns` must be between 1 and 50
- `session_id` must be a valid UUID format if provided

## Error Codes

- `AGENT_QUERY_INVALID`: Query text is invalid or missing
- `AGENT_RETRIEVAL_FAILED`: Retrieval from Qdrant failed
- `AGENT_GENERATION_FAILED`: Response generation failed
- `AGENT_SESSION_NOT_FOUND`: Specified session does not exist
- `AGENT_MAX_TURNS_EXCEEDED`: Conversation exceeded maximum turn limit
- `AGENT_API_ERROR`: General API or service error