# Humanoid Robotics Textbook - Backend

RAG (Retrieval-Augmented Generation) agent for querying the Physical AI & Humanoid Robotics textbook with intelligent retrieval and multi-turn conversation support.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
- [Configuration](#configuration)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Development](#development)

---

## Overview

This backend provides an intelligent agent that can:

1. **Answer Questions**: Query the textbook using natural language and receive context-aware responses
2. **Multi-Turn Conversations**: Maintain conversation history for follow-up questions
3. **Selected Text Mode**: Ask questions about specific highlighted passages from the textbook

**Technology Stack**:
- **Python 3.13+** with UV package manager
- **Qdrant** vector database (1,278+ embedded textbook chunks)
- **Cohere** embeddings (embed-english-v3.0)
- **Gemini 2.5 Flash** response generation
- **pytest** for comprehensive testing

---

## Features

### 1. RAG Query Processing

Submit natural language queries and receive accurate, textbook-grounded responses.

**Capabilities**:
- Automatic query embedding (Cohere embed-english-v3.0)
- Vector similarity search (Qdrant with 1,278+ chunks)
- Context-augmented response generation (Gemini 2.5 Flash)
- Configurable retrieval parameters (top_k, score_threshold)
- Configurable generation parameters (temperature, max_tokens)

**Example**:
```python
from agent import run_agent_query

result = run_agent_query(
    query_text="How do I install ROS 2?",
    top_k=5,
    score_threshold=0.5,
    temperature=0.5
)

print(result.response_text)
# Output: Detailed installation instructions citing textbook modules
print(f"Retrieved {len(result.retrieved_chunks)} chunks")
print(f"Response time: {result.response_time_ms}ms")
```

### 2. Multi-Turn Conversations

Maintain conversation context across multiple queries for natural follow-up questions.

**Capabilities**:
- Session-based conversation management
- Conversation history (last 3 turns for context)
- Context-aware pronoun resolution ("it", "they", "this")
- Automatic max_turns enforcement (10 turn limit)
- Session CRUD operations

**Example**:
```python
from agent import create_agent_session, continue_agent_session

# Start a new conversation
session = create_agent_session()
print(f"Session ID: {session.session_id}")

# Turn 1: Ask about VLA models
result1 = continue_agent_session(
    session_id=session.session_id,
    query_text="What are VLA models?"
)
print(f"Turn 1: {result1.response_text[:100]}...")

# Turn 2: Follow-up question (context-aware)
result2 = continue_agent_session(
    session_id=session.session_id,
    query_text="How are they trained?"  # "they" = VLA models
)
print(f"Turn 2: {result2.response_text[:100]}...")

# Verify session state
print(f"Current turn: {session.current_turn}")
print(f"Conversation history: {len(session.conversation_history)} turns")
```

### 3. Selected Text Mode

Ask questions about specific highlighted passages from the textbook.

**Capabilities**:
- Selected-text biasing for retrieval
- Specialized system prompts focusing on selected content
- Automatic truncation for long selections
- Fallback handling for unrelated selections

**Example**:
```python
from agent import run_agent_query

# User highlights this text from the textbook
selected_text = """
ROS 2 nodes are the fundamental building blocks of a ROS 2 application.
Each node represents a single, modular purpose in the robotics system.
"""

# Ask a question about the selection
result = run_agent_query(
    query_text="Explain this concept in more detail",
    selected_text=selected_text,
    top_k=5,
    score_threshold=0.4
)

print(result.response_text)
# Output: Focused explanation of ROS 2 nodes with additional context
```

---

## Architecture

### System Flow

```
User Query
    ↓
run_agent_query() [Validation & Orchestration]
    ↓
┌─────────────────────────────────────────────┐
│  Retrieval Pipeline                         │
│  1. Cohere Embedding (embed-english-v3.0)  │
│  2. Qdrant Vector Search (1,278+ chunks)   │
│  3. Chunk Re-ranking (similarity scores)   │
└─────────────────────────────────────────────┘
    ↓
Context Augmentation [Selected-text biasing]
    ↓
┌─────────────────────────────────────────────┐
│  Response Generation                        │
│  - Gemini 2.5 Flash (google.generativeai)  │
│  - System prompt engineering               │
│  - Temperature control (0.0-1.0)           │
└─────────────────────────────────────────────┘
    ↓
AgentResponse [Structured output with metadata]
```

### Core Components

**agent.py** (804 lines):
- Data classes: `AgentRequest`, `AgentResponse`, `AgentSession`, `ConversationTurn`
- Session management: `create_agent_session()`, `continue_agent_session()`, `get_agent_session()`, `delete_agent_session()`
- Query processing: `run_agent_query()`
- Helper functions: `_build_retrieval_query()`, `_generate_rag_response()`, `_build_conversation_context()`

**retrieve.py** (from Spec 1):
- Qdrant vector search
- `retrieve_content()` function

**embedding.py** (from Spec 1-2):
- Cohere embedding generation
- `generate_embedding()` function

**storage.py** (from Spec 1):
- Textbook content preprocessing and storage

---

## Installation

### Prerequisites

- Python 3.13+
- UV package manager
- Qdrant instance (cloud or local)
- API keys (Cohere, Gemini, Qdrant)

### Setup

1. **Clone Repository**:
```bash
cd backend
```

2. **Install Dependencies**:
```bash
uv sync
```

3. **Configure Environment**:

Create `.env` file in backend directory:
```bash
# Cohere API (embeddings)
COHERE_API_KEY=your_cohere_key_here

# Gemini API (response generation)
GEMINI_API_KEY=your_gemini_key_here

# Qdrant (vector database)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
```

4. **Verify Installation**:
```bash
uv run python -c "from retrieve import retrieve_content; from embedding import generate_embedding; print('✓ Functions verified')"
```

---

## Quick Start

### Basic Query

```python
from agent import run_agent_query

# Ask a question about the textbook
result = run_agent_query(
    query_text="What is Gazebo used for in robotics?",
    top_k=5,              # Retrieve top 5 chunks
    score_threshold=0.5,  # Minimum similarity score
    temperature=0.5       # Response creativity (0.0-1.0)
)

# Display results
print(f"Response: {result.response_text}")
print(f"Retrieved {len(result.retrieved_chunks)} chunks")
print(f"Response time: {result.response_time_ms}ms")

# Check for errors
if not result.retrieval_success:
    print(f"Error: {result.error_message}")
```

### Multi-Turn Conversation

```python
from agent import create_agent_session, continue_agent_session

# Create a new session
session = create_agent_session()

# Have a conversation
queries = [
    "What are humanoid robots?",
    "What are some examples?",
    "How do they maintain balance?"
]

for i, query in enumerate(queries, 1):
    result = continue_agent_session(
        session_id=session.session_id,
        query_text=query,
        top_k=5
    )
    print(f"\nTurn {i}: {query}")
    print(f"Response: {result.response_text[:200]}...")
```

### Selected Text Query

```python
from agent import run_agent_query

# User highlights text from the textbook
selected_text = """
Gazebo is a premier simulation environment for robotics, providing
realistic physics modeling, sensor simulation, and 3D visualization.
"""

# Ask about the selection
result = run_agent_query(
    query_text="Why is this important for humanoid robotics?",
    selected_text=selected_text,
    top_k=5,
    score_threshold=0.4
)

print(result.response_text)
```

---

## API Reference

### Core Functions

#### `run_agent_query()`

Process a single query with optional selected text.

**Signature**:
```python
def run_agent_query(
    query_text: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None,
    temperature: float = 0.5,
    max_turns: int = 5,
    top_k: int = 5,
    score_threshold: float = 0.5,
    api_key: Optional[str] = None
) -> AgentResponse:
```

**Parameters**:
- `query_text` (str): User's question (1-1000 characters)
- `selected_text` (Optional[str]): Highlighted text from textbook
- `session_id` (Optional[str]): Unused (for future session integration)
- `temperature` (float): Response creativity, 0.0-1.0 (default: 0.5)
- `max_turns` (int): Maximum conversation turns, 1-50 (default: 5)
- `top_k` (int): Number of chunks to retrieve, 1-10 (default: 5)
- `score_threshold` (float): Minimum similarity score, 0.0-1.0 (default: 0.5)
- `api_key` (Optional[str]): Gemini API key (defaults to env var)

**Returns**:
- `AgentResponse`: Structured response with metadata

**Example**:
```python
result = run_agent_query(
    query_text="How do I install ROS 2?",
    top_k=5,
    score_threshold=0.5,
    temperature=0.5
)

if result.retrieval_success:
    print(result.response_text)
else:
    print(f"Error: {result.error_message}")
```

#### `create_agent_session()`

Create a new conversation session.

**Signature**:
```python
def create_agent_session() -> AgentSession:
```

**Returns**:
- `AgentSession`: New session with unique ID

**Example**:
```python
session = create_agent_session()
print(f"Session ID: {session.session_id}")
```

#### `continue_agent_session()`

Continue an existing conversation session with a new query.

**Signature**:
```python
def continue_agent_session(
    session_id: str,
    query_text: str,
    selected_text: Optional[str] = None,
    temperature: float = 0.5,
    top_k: int = 5,
    score_threshold: float = 0.5,
    api_key: Optional[str] = None
) -> AgentResponse:
```

**Parameters**:
- `session_id` (str): UUID from `create_agent_session()`
- `query_text` (str): User's query
- `selected_text` (Optional[str]): Highlighted text (if applicable)
- `temperature` (float): Response creativity, 0.0-1.0
- `top_k` (int): Number of chunks to retrieve, 1-10
- `score_threshold` (float): Minimum similarity score, 0.0-1.0
- `api_key` (Optional[str]): Gemini API key

**Returns**:
- `AgentResponse`: Response with conversation context

**Example**:
```python
session = create_agent_session()

result1 = continue_agent_session(
    session_id=session.session_id,
    query_text="What are VLA models?"
)

result2 = continue_agent_session(
    session_id=session.session_id,
    query_text="How are they trained?"  # Context-aware
)
```

#### `get_agent_session()`

Retrieve an existing session.

**Signature**:
```python
def get_agent_session(session_id: str) -> Optional[AgentSession]:
```

**Parameters**:
- `session_id` (str): UUID from `create_agent_session()`

**Returns**:
- `AgentSession` if found, `None` otherwise

**Example**:
```python
session = get_agent_session("session_abc123...")
if session:
    print(f"Turn: {session.current_turn}")
    print(f"History: {len(session.conversation_history)} turns")
```

#### `delete_agent_session()`

Delete a session from storage.

**Signature**:
```python
def delete_agent_session(session_id: str) -> bool:
```

**Parameters**:
- `session_id` (str): UUID from `create_agent_session()`

**Returns**:
- `True` if deleted, `False` if not found

**Example**:
```python
deleted = delete_agent_session("session_abc123...")
print(f"Session deleted: {deleted}")
```

---

### Data Classes

#### `AgentRequest`

User query with configuration parameters.

**Fields**:
- `query_text` (str): User's question
- `selected_text` (Optional[str]): Highlighted text
- `session_id` (Optional[str]): Session UUID
- `temperature` (float): Response creativity
- `max_turns` (int): Maximum conversation turns
- `top_k` (int): Number of chunks to retrieve
- `score_threshold` (float): Minimum similarity score

**Validation**:
- `query_text`: Non-empty, <1000 characters
- `temperature`: 0.0-1.0
- `max_turns`: 1-50
- `top_k`: 1-10
- `score_threshold`: 0.0-1.0

#### `AgentResponse`

Structured response with metadata.

**Fields**:
- `response_text` (str): Generated response
- `retrieved_chunks` (List[RetrievedChunk]): Retrieved textbook chunks
- `conversation_id` (str): Unique response ID
- `response_time_ms` (float): Total processing time
- `query_tokens` (int): Input token count
- `response_tokens` (int): Output token count
- `retrieval_success` (bool): Whether retrieval succeeded
- `error_message` (Optional[str]): Error details if failed

**Example**:
```python
result = run_agent_query("What is ROS 2?")

print(f"Response: {result.response_text}")
print(f"Retrieved {len(result.retrieved_chunks)} chunks:")
for chunk in result.retrieved_chunks:
    print(f"  - {chunk.source_file} (score: {chunk.similarity_score:.2f})")
print(f"Processing time: {result.response_time_ms:.0f}ms")
```

#### `AgentSession`

Conversation session state.

**Fields**:
- `session_id` (str): Unique session UUID
- `created_at` (str): ISO timestamp
- `last_accessed` (str): ISO timestamp
- `conversation_history` (List[ConversationTurn]): All turns
- `current_turn` (int): Turn counter
- `metadata` (dict): Additional session data

#### `ConversationTurn`

Individual turn in a conversation.

**Fields**:
- `turn_number` (int): Turn counter (1-indexed)
- `user_input` (str): User's query
- `retrieved_chunks` (List[RetrievedChunk]): Retrieved chunks
- `agent_response` (str): Generated response
- `timestamp` (str): ISO timestamp
- `tool_calls` (List[dict]): Tool invocations (if any)
- `context_used` (Optional[str]): Conversation context

---

## Configuration

### Environment Variables

Create `.env` file with:

```bash
# Required
COHERE_API_KEY=your_cohere_key       # Embeddings
GEMINI_API_KEY=your_gemini_key       # Response generation
QDRANT_URL=https://your-cluster      # Vector database
QDRANT_API_KEY=your_qdrant_key       # Vector database auth

# Optional
LOG_LEVEL=INFO                        # Logging level
MAX_SESSION_TURNS=10                  # Conversation turn limit
CONTEXT_TURNS=3                       # Turns kept in context
```

### Query Parameters

Adjust these parameters based on your use case:

**Retrieval Quality vs. Speed**:
```python
# High quality (slower)
result = run_agent_query(
    query_text="...",
    top_k=10,             # More context
    score_threshold=0.6   # Higher precision
)

# Fast (lower quality)
result = run_agent_query(
    query_text="...",
    top_k=3,              # Less context
    score_threshold=0.3   # Higher recall
)
```

**Response Creativity**:
```python
# Factual, deterministic
result = run_agent_query(
    query_text="...",
    temperature=0.0       # Minimal creativity
)

# Creative, varied
result = run_agent_query(
    query_text="...",
    temperature=0.9       # High creativity
)
```

**Selected Text Queries**:
```python
# For selected text, use lower threshold
result = run_agent_query(
    query_text="Explain this",
    selected_text="...",
    score_threshold=0.3   # More lenient matching
)
```

---

## Testing

### Run All Tests

```bash
cd backend
uv run pytest tests/ -v
```

### Run Specific Test Suites

**Unit Tests Only**:
```bash
uv run pytest tests/test_agent.py -v
```

**Integration Tests Only**:
```bash
uv run pytest tests/test_agent_integration.py -v
```

**Run Tests by Marker**:
```bash
# Integration tests only
uv run pytest -m integration -v

# Skip integration tests
uv run pytest -m "not integration" -v
```

### Test Coverage

**Total Tests**: 44
- **Unit Tests**: 31 (test_agent.py)
- **Integration Tests**: 13 (test_agent_integration.py)

**Coverage Areas**:
- ✅ AgentRequest validation (12 tests)
- ✅ AgentResponse validation (3 tests)
- ✅ run_agent_query() function (4 tests)
- ✅ AgentSession validation (3 tests)
- ✅ ConversationTurn validation (3 tests)
- ✅ Selected-text validation (3 tests)
- ✅ Selected-text processing (3 tests)
- ✅ ROS 2 queries (2 integration tests)
- ✅ Gazebo queries (2 integration tests)
- ✅ Multi-turn conversations (3 integration tests)
- ✅ Selected-text mode (3 integration tests)

### Example Test Run

```bash
$ uv run pytest tests/test_agent.py -v

tests/test_agent.py::TestAgentRequestValidation::test_request_validation_success PASSED
tests/test_agent.py::TestAgentRequestValidation::test_request_validation_empty_query PASSED
...
tests/test_agent.py::TestSelectedTextProcessing::test_selected_text_processing_success PASSED

============================== 31 passed in 2.5s ==============================
```

---

## Troubleshooting

### Common Issues

#### 1. API Quota Exceeded

**Error**:
```
ERROR: Response generation failed: 429 You exceeded your current quota
* Quota exceeded for metric: generativelanguage.googleapis.com/generate_content_free_tier_requests, limit: 20, model: gemini-2.5-flash
```

**Cause**: Gemini free tier has 20 requests/day limit.

**Solutions**:
- Wait 24 hours for quota reset
- Upgrade to paid Gemini API tier (recommended for production)
- Check `result.error_message` for graceful error handling

**Example**:
```python
result = run_agent_query("What is ROS 2?")
if not result.retrieval_success:
    if "quota" in result.error_message.lower():
        print("API quota exceeded. Try again tomorrow or upgrade to paid tier.")
    else:
        print(f"Error: {result.error_message}")
```

#### 2. Session Not Found

**Error**:
```python
result = continue_agent_session(session_id="invalid", query_text="test")
# result.error_message: "Session not found: invalid"
```

**Cause**: Session ID doesn't exist or expired (server restart clears in-memory sessions).

**Solution**:
```python
# Always check if session exists
session = get_agent_session(session_id)
if not session:
    session = create_agent_session()
```

#### 3. Maximum Turns Reached

**Error**:
```python
# After 10 turns
result = continue_agent_session(session_id="...", query_text="test")
# result.error_message: "Session has reached maximum turns (10)"
```

**Cause**: Sessions limited to 10 turns to prevent infinite conversations.

**Solution**:
```python
# Start a new session
new_session = create_agent_session()
```

#### 4. No Chunks Retrieved

**Error**:
```python
result = run_agent_query("xyz", score_threshold=0.9)
# result.response_text includes: "No relevant content found in the textbook."
```

**Cause**: Query too specific or score_threshold too high.

**Solutions**:
- Lower score_threshold (try 0.3-0.5)
- Rephrase query to match textbook topics
- Increase top_k to retrieve more candidates

**Example**:
```python
# If no results, retry with lower threshold
result = run_agent_query("specific query", score_threshold=0.5)
if "No relevant content" in result.response_text:
    result = run_agent_query("specific query", score_threshold=0.3)
```

#### 5. Import Errors

**Error**:
```
ModuleNotFoundError: No module named 'agent'
```

**Solution**:
```bash
# Ensure you're in the backend directory
cd backend

# Run with uv
uv run python -c "from agent import run_agent_query"
```

#### 6. Missing Environment Variables

**Error**:
```
ERROR: GEMINI_API_KEY not found in environment
```

**Solution**:
```bash
# Check .env file exists
ls -la .env

# Verify contents
cat .env | grep GEMINI_API_KEY

# If missing, add it
echo "GEMINI_API_KEY=your_key_here" >> .env
```

---

## Development

### Project Structure

```
backend/
├── agent.py                    # RAG agent implementation
├── retrieve.py                 # Qdrant retrieval (Spec 1)
├── embedding.py                # Cohere embeddings (Spec 1-2)
├── storage.py                  # Content storage (Spec 1)
├── tests/
│   ├── test_agent.py          # Unit tests (31 tests)
│   └── test_agent_integration.py  # Integration tests (13 tests)
├── .env                        # Environment configuration
├── pyproject.toml             # UV dependencies
├── README.md                  # This file
└── VALIDATION_REPORT.md       # Test results and metrics
```

### Key Files

**agent.py** (804 lines):
- Data classes (140 lines)
- Session management (200 lines)
- Query processing (150 lines)
- Helper functions (300 lines)

**tests/test_agent.py** (502 lines):
- 31 unit tests covering all data classes and core functions
- Test organization: 8 test classes
- Coverage: validation, query processing, sessions, selected-text

**tests/test_agent_integration.py** (600+ lines):
- 13 integration tests covering end-to-end workflows
- Test organization: 5 test classes
- Coverage: ROS 2, Gazebo, VLA models, multi-turn, selected-text

### Code Quality

**Documentation**:
- ✅ Google-style docstrings on all functions
- ✅ Type hints on all parameters and return values
- ✅ Inline comments for complex logic

**Linting**:
```bash
# Check code quality
uv run ruff check .

# Auto-fix issues
uv run ruff check . --fix
```

**Type Checking**:
```bash
# Run mypy
uv run mypy agent.py
```

### Performance Metrics

**Typical Response Times**:
- Query processing: 6-8 seconds (end-to-end)
  - Embedding generation: ~500ms
  - Qdrant retrieval: ~300ms
  - Context augmentation: ~100ms
  - Gemini response generation: 5-7 seconds

**Multi-Turn Conversation Overhead**:
- Session lookup: <10ms
- Conversation history building: ~50ms (for 3 turns)
- Total overhead: ~60ms (negligible)

### Known Limitations

1. **In-Memory Session Storage**: Sessions lost on server restart (use Redis for production)
2. **API Quota**: Gemini free tier has 20 requests/day limit
3. **Conversation History**: Only last 3 turns kept (to avoid token limits)
4. **No Streaming**: Responses generated in full before returning
5. **Selected-Text Truncation**: Limited to 500 chars for retrieval, 1000 for prompts

### Contributing

When adding new features:

1. **Write Tests First** (TDD):
   - Add unit tests to `tests/test_agent.py`
   - Add integration tests to `tests/test_agent_integration.py`
   - Run tests: `uv run pytest tests/ -v`

2. **Add Documentation**:
   - Google-style docstrings on all functions
   - Type hints on all parameters
   - Update this README with usage examples

3. **Check Code Quality**:
   ```bash
   uv run ruff check .
   uv run mypy agent.py
   uv run pytest tests/ -v --cov=agent
   ```

---

## Support

**Documentation**:
- See `VALIDATION_REPORT.md` for test results and metrics
- See `specs/002-rag-agent-integration/` for design documents

**Issues**:
- Report bugs or feature requests in the project issue tracker

**API Documentation**:
- Cohere: https://docs.cohere.com/
- Gemini: https://ai.google.dev/docs
- Qdrant: https://qdrant.tech/documentation/

---

**Version**: 1.0.0
**Last Updated**: 2025-12-26
**Status**: MVP Complete (54/70 tasks, 77%)
