# Quickstart: RAG Agent Integration

## Overview

The RAG Agent provides an intelligent interface to query the Physical AI & Humanoid Robotics textbook using natural language. The agent retrieves relevant content from the vector database and generates contextually appropriate responses.

## Prerequisites

- Python 3.13
- UV package manager
- Environment variables configured:
  - `QDRANT_URL`: URL to your Qdrant instance
  - `QDRANT_API_KEY`: API key for Qdrant access
  - `QDRANT_COLLECTION_NAME`: Collection name (default: "humanoid-robotics-textbook")
  - `COHERE_API_KEY`: API key for Cohere embedding service
  - `GEMINI_API_KEY`: API key for Gemini 2.0 Flash model

## Setup

1. **Install dependencies**:
   ```bash
   uv sync
   ```

2. **Verify environment**:
   ```bash
   python -c "import agents; print('OpenAI Agents SDK available')"
   python -c "import cohere; print('Cohere SDK available')"
   python -c "import qdrant_client; print('Qdrant client available')"
   ```

3. **Verify vector database**:
   Ensure your Qdrant instance contains the embedded textbook content from Spec 1.

## Basic Usage

### Single Query Example

```python
from backend.agent import run_agent_query

# Simple query
result = run_agent_query(
    query_text="How do I install ROS 2?",
    api_key=os.getenv("GEMINI_API_KEY")
)

print(f"Response: {result.response_text}")
print(f"Retrieved {len(result.retrieved_chunks)} chunks")
```

### Multi-Turn Conversation

```python
from backend.agent import create_agent_session, continue_agent_session

# Create new session
session = create_agent_session(session_id="my-session-123")

# First query
response1 = continue_agent_session(
    session_id="my-session-123",
    query_text="What is Gazebo simulation?"
)

# Follow-up query (context maintained)
response2 = continue_agent_session(
    session_id="my-session-123",
    query_text="How do I get started with it?"
)
```

### Selected Text Mode

```python
from backend.agent import run_agent_query

# Query with selected text context
result = run_agent_query(
    query_text="Explain this",
    selected_text="Gazebo is a robot simulation environment that provides physics simulation, sensor simulation, and robot models.",
    api_key=os.getenv("GEMINI_API_KEY")
)
```

## Configuration Options

### Custom Parameters

```python
result = run_agent_query(
    query_text="Your question here",
    temperature=0.3,        # Lower for more factual responses
    top_k=3,               # Number of chunks to retrieve
    score_threshold=0.6,   # Minimum similarity score
    max_turns=5           # For conversation sessions
)
```

## Testing

### Run Agent Tests

```bash
# Unit tests for agent functionality
pytest backend/tests/test_agent.py

# Integration tests with retrieval
pytest backend/tests/test_agent_integration.py

# End-to-end validation
python -m backend.tests.test_agent_retrieval
```

### Validation Queries

Run the following queries to validate agent functionality:

```bash
# Test basic retrieval
python -c "from backend.agent import validate_agent; validate_agent()"

# Run 5 sample queries covering different textbook modules:
# 1. "How do I install ROS 2?"
# 2. "What is Gazebo simulation?"
# 3. "Explain VLA models"
# 4. "How to work with humanoid robots?"
# 5. "What are the key concepts in robotics?"
```

## Expected Performance

- **Response Time**: <5 seconds for 95% of queries
- **Success Rate**: 100% for 5+ validation queries
- **Context Accuracy**: >90% of responses cite correct textbook modules
- **Multi-turn**: Maintains context across 5+ conversation turns

## Troubleshooting

### Common Issues

**Issue**: "No results returned for query"
- **Cause**: Query too specific or similarity threshold too high
- **Solution**: Lower `score_threshold` parameter or rephrase query

**Issue**: "API connection failed"
- **Cause**: Invalid API keys or network connectivity
- **Solution**: Verify environment variables and network access

**Issue**: "Session not found"
- **Cause**: Invalid or expired session ID
- **Solution**: Create a new session or verify session ID format

### Debugging

Enable debug logging:
```bash
export LOG_LEVEL=DEBUG
python -c "from backend.agent import run_agent_query; run_agent_query('test query')"
```

### Error Handling

The agent gracefully handles these scenarios:
- Empty retrieval results (falls back to general knowledge)
- API connection failures (retries with backoff)
- Invalid input parameters (returns validation errors)
- Max turns exceeded (terminates conversation gracefully)

## API Endpoints

Once deployed, the agent is accessible via:

- `POST /api/agent/query` - Single query processing
- `POST /api/agent/conversation` - Multi-turn conversation
- `GET /api/agent/session/{id}` - Session information
- `DELETE /api/agent/session/{id}` - End session
- `GET /api/agent/health` - Service health check

## Next Steps

1. **Customize agent instructions** in the system prompt for specific use cases
2. **Adjust retrieval parameters** based on your content and performance requirements
3. **Add custom tools** beyond retrieval if additional functionality is needed
4. **Implement rate limiting** for production deployments
5. **Add monitoring and observability** for production environments