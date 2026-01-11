# RAG Agent Integration - Rewrite Plan
## Migration from Gemini API to OpenAI Agents SDK

**Date**: 2025-12-28
**Status**: Architecture Plan
**Scope**: Complete rewrite of `backend/agent.py`

---

## Executive Summary

The current implementation violates the specification by using Google Gemini API directly instead of the OpenAI Agents SDK. This plan details the complete architectural transformation required to align with specification requirements.

**Current State**: Gemini API with manual orchestration (0% SDK usage)
**Target State**: OpenAI Agents SDK with Qwen LLM via OpenRouter (100% SDK compliance)

---

## 1. Architecture Comparison

### Current Architecture (INCORRECT)

```
┌─────────────────────────────────────────────┐
│         backend/agent.py                    │
├─────────────────────────────────────────────┤
│                                             │
│  ┌─────────────────────────────────────┐   │
│  │  Manual Orchestration Layer         │   │
│  │  - run_agent_query()                │   │
│  │  - continue_agent_session()         │   │
│  │  - _generate_rag_response()         │   │
│  │  - _generate_rag_response_with_*()  │   │
│  └──────────────┬──────────────────────┘   │
│                 │                           │
│                 ▼                           │
│  ┌─────────────────────────────────────┐   │
│  │  Google Gemini API                  │   │
│  │  - google.generativeai              │   │
│  │  - genai.GenerativeModel()          │   │
│  │  - model='gemini-2.5-flash'         │   │
│  └─────────────────────────────────────┘   │
│                                             │
│  ┌─────────────────────────────────────┐   │
│  │  Manual Session Management          │   │
│  │  - _active_sessions dict            │   │
│  │  - AgentSession dataclass           │   │
│  └─────────────────────────────────────┘   │
│                                             │
│  ┌─────────────────────────────────────┐   │
│  │  Manual Retrieval Integration       │   │
│  │  - Direct retrieve_content() calls  │   │
│  │  - Manual prompt construction       │   │
│  └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

**Issues**:
- No SDK components (`Agent`, `Runner`, `function_tool`)
- Duplicated response generation code (3+ functions)
- Manual session management instead of SQLiteSession
- Wrong LLM (Gemini instead of Qwen)
- No tool integration pattern

### Target Architecture (CORRECT)

```
┌─────────────────────────────────────────────┐
│         backend/agent.py                    │
├─────────────────────────────────────────────┤
│                                             │
│  ┌─────────────────────────────────────┐   │
│  │  OpenAI Agents SDK Layer            │   │
│  │  - Agent() with instructions        │   │
│  │  - Runner.run() / run_sync()        │   │
│  │  - Runner.run_streamed()            │   │
│  └──────────────┬──────────────────────┘   │
│                 │                           │
│                 ▼                           │
│  ┌─────────────────────────────────────┐   │
│  │  LitellmModel (OpenRouter)          │   │
│  │  - model='qwen/qwen3-coder:free'    │   │
│  │  - api_base=openrouter.ai/api/v1    │   │
│  │  - api_key=OPENROUTER_API_KEY       │   │
│  └─────────────────────────────────────┘   │
│                                             │
│  ┌─────────────────────────────────────┐   │
│  │  SDK Session Management             │   │
│  │  - SQLiteSession()                  │   │
│  │  - previous_response_id             │   │
│  └─────────────────────────────────────┘   │
│                                             │
│  ┌─────────────────────────────────────┐   │
│  │  Function Tools (@function_tool)    │   │
│  │  - retrieve_textbook_content()      │   │
│  │  - Wraps retrieve_content()         │   │
│  └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

**Benefits**:
- Single source of truth (SDK handles orchestration)
- No code duplication
- Built-in session management
- Correct LLM (Qwen via OpenRouter)
- Standard tool integration pattern

---

## 2. Code Transformation Examples

### Example 1: Agent Creation

**BEFORE** (Current - WRONG):
```python
import google.generativeai as genai

genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
_gemini_model_cache = genai.GenerativeModel('gemini-2.5-flash')

def _generate_rag_response(
    query_text: str,
    retrieved_chunks: List[RetrievalChunk],
    api_key: Optional[str] = None,
    model_name: str = "gemini-2.0-flash",
    temperature: float = 0.7,
    selected_text: Optional[str] = None
) -> str:
    """Generate response using Gemini with retrieved context."""
    try:
        import google.generativeai as genai

        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-2.5-flash')

        # Manual prompt construction
        context = "\n\n".join([
            f"[{chunk.source_file}] {chunk.content_text}"
            for chunk in retrieved_chunks
        ])

        full_prompt = f"""You are an expert AI assistant...

Context from textbook:
{context}

User query: {query_text}"""

        response = model.generate_content(
            full_prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=temperature,
                max_output_tokens=8192
            )
        )

        return response.text
    except Exception as e:
        logger.error(f"Error generating response: {str(e)}")
        return f"Error: {str(e)}"
```

**AFTER** (Target - CORRECT):
```python
from agents import Agent, Runner, function_tool
from agents.extensions.models.litellm_model import LitellmModel
from typing import Annotated

# Create retrieval tool
@function_tool
def retrieve_textbook_content(
    query: Annotated[str, "The user's question about the textbook"],
    top_k: Annotated[int, "Number of chunks to retrieve"] = 5,
    score_threshold: Annotated[float, "Minimum similarity score"] = 0.4
) -> str:
    """Retrieve relevant content from the Physical AI & Humanoid Robotics textbook.

    This tool searches the vector database for content relevant to the user's query
    and returns formatted chunks with source information.
    """
    logger.info(f"Retrieving content for query: {query[:50]}...")

    result = retrieve_content(
        query_text=query,
        client=_get_cached_qdrant_client(),
        collection_name=os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook"),
        top_k=top_k,
        score_threshold=score_threshold,
        api_key=os.getenv("COHERE_API_KEY")
    )

    if not result.retrieval_success:
        return f"Error retrieving content: {result.error_message}"

    if not result.retrieved_chunks:
        return "No relevant content found in the textbook for this query."

    # Format chunks for agent
    formatted_chunks = []
    for i, chunk in enumerate(result.retrieved_chunks, 1):
        formatted_chunks.append(
            f"[Source {i}: {chunk.source_file}]\n{chunk.content_text}\n"
        )

    return "\n".join(formatted_chunks)


# Create agent instance
_agent = Agent(
    name="Physical AI & Humanoid Robotics Assistant",
    instructions="""You are an expert AI assistant specializing in Physical AI and Humanoid Robotics.

Your role:
- Answer questions about robotics, ROS 2, Gazebo simulation, VLA models, and humanoid robots
- Base your answers on the textbook content retrieved via the retrieve_textbook_content tool
- Cite specific modules, sections, or sources when referencing the textbook
- Be clear, accurate, and educational in your explanations
- If the textbook doesn't contain relevant information, acknowledge this and provide general guidance

Instructions:
1. For every user question, use the retrieve_textbook_content tool to fetch relevant information
2. Analyze the retrieved content carefully
3. Provide a comprehensive answer based on the textbook material
4. Cite sources (module names, file names) when making specific claims
5. If answering based on general knowledge (not in textbook), clearly indicate this""",

    model=LitellmModel(
        model="qwen/qwen3-coder:free",
        api_base="https://openrouter.ai/api/v1",
        api_key=os.getenv("OPENROUTER_API_KEY")
    ),

    tools=[retrieve_textbook_content]
)
```

**Changes**:
- ✅ Removed all Gemini imports and configuration
- ✅ Added OpenAI Agents SDK imports
- ✅ Created `@function_tool` for retrieval (replaces manual prompt construction)
- ✅ Created `Agent` instance with Qwen via OpenRouter
- ✅ Moved instructions into Agent configuration
- ✅ Eliminated manual response generation function

---

### Example 2: Query Execution

**BEFORE** (Current - WRONG):
```python
def run_agent_query(
    query_text: str,
    api_key: Optional[str] = None,
    model_name: str = "gemini-2.0-flash",
    temperature: float = 0.7,
    top_k: int = 5,
    score_threshold: float = 0.5,
    selected_text: Optional[str] = None,
    max_turns: Optional[int] = None
) -> AgentResponse:
    """Execute a single RAG agent query."""
    start_time = time.time()
    conversation_id = f"conv_{uuid.uuid4().hex[:16]}"

    try:
        # Step 1: Retrieve content
        retrieval_result = retrieve_content(
            query_text=query_text,
            client=_get_cached_qdrant_client(),
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook"),
            top_k=top_k,
            score_threshold=score_threshold,
            api_key=os.getenv("COHERE_API_KEY")
        )

        # Step 2: Generate response manually
        if selected_text:
            response_text = _generate_rag_response_with_selected_text(...)
        else:
            response_text = _generate_rag_response(...)

        # Step 3: Manual response construction
        end_time = time.time()
        return AgentResponse(
            conversation_id=conversation_id,
            response_text=response_text,
            retrieved_chunks=retrieval_result.retrieved_chunks,
            retrieval_success=retrieval_result.retrieval_success,
            response_time_ms=(end_time - start_time) * 1000
        )
    except Exception as e:
        logger.error(f"Error in run_agent_query: {str(e)}")
        return AgentResponse(...)
```

**AFTER** (Target - CORRECT):
```python
def run_agent_query(
    query_text: str,
    temperature: float = 0.7,
    top_k: int = 5,
    score_threshold: float = 0.5,
    selected_text: Optional[str] = None,
    max_tokens: int = 8192
) -> AgentResponse:
    """Execute a single RAG agent query using OpenAI Agents SDK.

    Args:
        query_text: The user's question
        temperature: Response randomness (0.0-1.0)
        top_k: Number of textbook chunks to retrieve
        score_threshold: Minimum similarity score for retrieval
        selected_text: Optional text selected by user for context
        max_tokens: Maximum response length

    Returns:
        AgentResponse with the agent's answer and metadata
    """
    start_time = time.time()
    conversation_id = f"conv_{uuid.uuid4().hex[:16]}"

    try:
        # Augment query if selected text provided
        if selected_text:
            augmented_query = f"""The user has selected this text:

\"\"\"{selected_text}\"\"\"

And asks: {query_text}

Please retrieve relevant textbook content and answer based on both the selected text and the textbook."""
        else:
            augmented_query = query_text

        # Configure model settings
        model_settings = ModelSettings(
            temperature=temperature,
            max_tokens=max_tokens
        )

        # Run agent (SDK handles tool calling automatically)
        result = Runner.run_sync(
            agent=_agent,
            query=augmented_query,
            model_settings=model_settings
        )

        # Extract retrieved chunks from tool calls
        retrieved_chunks = _extract_chunks_from_tool_calls(result)

        # Construct response
        end_time = time.time()
        return AgentResponse(
            conversation_id=conversation_id,
            response_text=result.final_output,
            retrieved_chunks=retrieved_chunks,
            retrieval_success=len(retrieved_chunks) > 0,
            response_time_ms=(end_time - start_time) * 1000,
            error_message=None
        )

    except Exception as e:
        logger.error(f"Error in run_agent_query: {str(e)}")
        end_time = time.time()
        return AgentResponse(
            conversation_id=conversation_id,
            response_text="",
            retrieved_chunks=[],
            retrieval_success=False,
            response_time_ms=(end_time - start_time) * 1000,
            error_message=str(e)
        )


def _extract_chunks_from_tool_calls(result: RunResult) -> List[RetrievalChunk]:
    """Extract retrieved chunks from agent's tool calls.

    Args:
        result: The RunResult from Runner.run_sync()

    Returns:
        List of RetrievalChunk objects
    """
    chunks = []

    # Parse tool calls to extract retrieved content
    # This assumes the retrieve_textbook_content tool stores chunk metadata
    # Implementation depends on SDK's tool call result structure

    return chunks
```

**Changes**:
- ✅ Removed manual retrieval + generation steps
- ✅ SDK handles tool calling automatically via `Runner.run_sync()`
- ✅ Removed `api_key` parameter (configured in Agent)
- ✅ Removed `model_name` parameter (configured in Agent)
- ✅ Added `ModelSettings` for temperature/max_tokens
- ✅ Simplified logic: SDK orchestrates everything

---

### Example 3: Multi-Turn Conversations

**BEFORE** (Current - WRONG):
```python
_active_sessions: Dict[str, AgentSession] = {}

@dataclass
class AgentSession:
    session_id: str
    conversation_history: List[Dict[str, Any]]
    created_at: datetime
    last_activity: datetime
    current_turn: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)

def create_agent_session(
    session_id: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None
) -> AgentSession:
    """Create a new agent session for multi-turn conversation."""
    if session_id is None:
        session_id = f"session_{uuid.uuid4().hex[:16]}"

    session = AgentSession(
        session_id=session_id,
        conversation_history=[],
        created_at=datetime.now(),
        last_activity=datetime.now(),
        metadata=metadata or {}
    )

    _active_sessions[session_id] = session
    logger.info(f"Created session: {session_id}")
    return session

def continue_agent_session(
    session_id: str,
    query_text: str,
    api_key: Optional[str] = None,
    temperature: float = 0.7,
    top_k: int = 5,
    score_threshold: float = 0.5
) -> AgentResponse:
    """Continue multi-turn conversation."""
    if session_id not in _active_sessions:
        raise ValueError(f"Session not found: {session_id}")

    session = _active_sessions[session_id]

    # Manual history management
    session.conversation_history.append({
        "role": "user",
        "content": query_text,
        "timestamp": datetime.now().isoformat()
    })

    # Retrieve + generate with history
    response_text = _generate_rag_response_with_history(
        query_text=query_text,
        conversation_history=session.conversation_history,
        ...
    )

    # Update session manually
    session.conversation_history.append({
        "role": "assistant",
        "content": response_text,
        "timestamp": datetime.now().isoformat()
    })
    session.current_turn += 1
    session.last_activity = datetime.now()

    return AgentResponse(...)
```

**AFTER** (Target - CORRECT):
```python
# No manual session management needed!

def create_agent_session(
    session_id: Optional[str] = None
) -> AgentSession:
    """Create a new agent session for multi-turn conversation.

    Args:
        session_id: Optional custom session ID (auto-generated if None)

    Returns:
        AgentSession with session metadata
    """
    if session_id is None:
        session_id = f"session_{uuid.uuid4().hex[:16]}"

    # SQLiteSession handles all persistence automatically
    session = SQLiteSession(session_id, "conversations.db")

    logger.info(f"Created session: {session_id}")

    return AgentSession(
        session_id=session_id,
        conversation_history=[],
        created_at=datetime.now(),
        last_activity=datetime.now(),
        current_turn=0,
        metadata={}
    )


def continue_agent_session(
    session_id: str,
    query_text: str,
    temperature: float = 0.7,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 8192
) -> AgentResponse:
    """Continue multi-turn conversation using SDK session management.

    Args:
        session_id: The session ID to continue
        query_text: The user's question
        temperature: Response randomness (0.0-1.0)
        top_k: Number of textbook chunks to retrieve
        score_threshold: Minimum similarity score for retrieval
        max_tokens: Maximum response length

    Returns:
        AgentResponse with the agent's answer and metadata
    """
    start_time = time.time()

    try:
        # SQLiteSession automatically loads conversation history
        session = SQLiteSession(session_id, "conversations.db")

        # Configure model settings
        model_settings = ModelSettings(
            temperature=temperature,
            max_tokens=max_tokens
        )

        # Run agent with session (SDK handles history automatically)
        result = Runner.run_sync(
            agent=_agent,
            query=query_text,
            session=session,  # ← SDK manages history!
            model_settings=model_settings
        )

        # Extract retrieved chunks
        retrieved_chunks = _extract_chunks_from_tool_calls(result)

        # Get current turn count from session
        turn_count = len(session.get_messages()) // 2  # user + assistant pairs

        end_time = time.time()
        return AgentResponse(
            conversation_id=session_id,
            response_text=result.final_output,
            retrieved_chunks=retrieved_chunks,
            retrieval_success=len(retrieved_chunks) > 0,
            response_time_ms=(end_time - start_time) * 1000,
            error_message=None
        )

    except Exception as e:
        logger.error(f"Error in continue_agent_session: {str(e)}")
        end_time = time.time()
        return AgentResponse(
            conversation_id=session_id,
            response_text="",
            retrieved_chunks=[],
            retrieval_success=False,
            response_time_ms=(end_time - start_time) * 1000,
            error_message=str(e)
        )
```

**Changes**:
- ✅ Removed `_active_sessions` dict
- ✅ Removed manual conversation history management
- ✅ SQLiteSession handles all persistence automatically
- ✅ SDK maintains context across turns
- ✅ No manual history appending needed

---

### Example 4: Streaming Responses

**BEFORE** (Current - WRONG):
```python
def run_agent_query_stream(
    query_text: str,
    api_key: Optional[str] = None,
    **kwargs
) -> Iterator[str]:
    """Stream agent response token-by-token."""
    try:
        import google.generativeai as genai

        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-2.5-flash')

        # Manual streaming implementation
        response = model.generate_content(
            prompt,
            generation_config=...,
            stream=True
        )

        for chunk in response:
            if chunk.text:
                yield chunk.text

    except Exception as e:
        yield f"Error: {str(e)}"
```

**AFTER** (Target - CORRECT):
```python
async def run_agent_query_stream(
    query_text: str,
    temperature: float = 0.7,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 8192
) -> AsyncIterator[str]:
    """Stream agent response token-by-token using SDK streaming.

    Args:
        query_text: The user's question
        temperature: Response randomness (0.0-1.0)
        top_k: Number of textbook chunks to retrieve
        score_threshold: Minimum similarity score for retrieval
        max_tokens: Maximum response length

    Yields:
        Text chunks as they are generated
    """
    try:
        # Configure model settings
        model_settings = ModelSettings(
            temperature=temperature,
            max_tokens=max_tokens
        )

        # Run agent in streaming mode (SDK handles everything)
        result = Runner.run_streamed(
            agent=_agent,
            query=query_text,
            model_settings=model_settings
        )

        # Stream events
        async for event in result.stream_events():
            if event.type == "raw_response_event":
                if isinstance(event.data, ResponseTextDeltaEvent):
                    yield event.data.delta

    except Exception as e:
        logger.error(f"Error in run_agent_query_stream: {str(e)}")
        yield f"Error: {str(e)}"
```

**Changes**:
- ✅ Removed manual Gemini streaming
- ✅ Use `Runner.run_streamed()` for SDK streaming
- ✅ Handle `ResponseTextDeltaEvent` from SDK
- ✅ Cleaner async/await pattern

---

## 3. Migration Checklist

### Phase 1: Dependencies
- [ ] **Remove** Gemini dependencies:
  ```bash
  uv remove google-generativeai
  ```

- [ ] **Add** OpenAI Agents SDK dependencies:
  ```bash
  uv add openai-agents litellm
  ```

- [ ] **Verify** installation:
  ```bash
  python -c "from agents import Agent, Runner, function_tool; print('✓ OpenAI Agents SDK available')"
  python -c "from agents.extensions.models.litellm_model import LitellmModel; print('✓ LiteLLM available')"
  ```

### Phase 2: Environment Variables
- [ ] **Add** OpenRouter configuration to `.env`:
  ```bash
  OPENROUTER_API_KEY=your_openrouter_key_here
  ```

- [ ] **Remove** (or keep for other purposes):
  ```bash
  # GEMINI_API_KEY=...  (no longer needed for agent)
  ```

- [ ] **Verify** environment:
  ```bash
  python -c "import os; from dotenv import load_dotenv; load_dotenv(); print('OPENROUTER_API_KEY:', 'SET' if os.getenv('OPENROUTER_API_KEY') else 'MISSING')"
  ```

### Phase 3: Code Transformation

#### 3.1 Create Function Tool (`agent.py` lines ~100-150)
- [ ] Add imports:
  ```python
  from agents import Agent, Runner, function_tool
  from agents.extensions.models.litellm_model import LitellmModel
  from agents.types import ModelSettings
  from typing import Annotated
  ```

- [ ] Create `@function_tool` for retrieval (replace `_generate_rag_response*` functions)

- [ ] Test tool independently:
  ```python
  result = retrieve_textbook_content("How do I install ROS 2?", top_k=3)
  print(result)
  ```

#### 3.2 Create Agent Instance (`agent.py` lines ~150-200)
- [ ] Define agent with:
  - Name: "Physical AI & Humanoid Robotics Assistant"
  - Instructions: System prompt from old implementation
  - Model: `LitellmModel(model="qwen/qwen3-coder:free", api_base="https://openrouter.ai/api/v1", api_key=os.getenv("OPENROUTER_API_KEY"))`
  - Tools: `[retrieve_textbook_content]`

- [ ] Test agent creation:
  ```python
  print(f"Agent created: {_agent.name}")
  print(f"Tools: {[tool.__name__ for tool in _agent.tools]}")
  ```

#### 3.3 Rewrite Query Functions (`agent.py` lines ~400-600)
- [ ] Replace `run_agent_query()` with SDK-based version
- [ ] Remove `_generate_rag_response()` function (lines ~540-590)
- [ ] Remove `_generate_rag_response_with_selected_text()` function (lines ~617-698)
- [ ] Remove `_generate_rag_response_with_history()` function (lines ~995-1049)
- [ ] Add `_extract_chunks_from_tool_calls()` helper

#### 3.4 Rewrite Session Management (`agent.py` lines ~765-900)
- [ ] Replace `_active_sessions` dict with SQLiteSession usage
- [ ] Update `create_agent_session()` to use SQLiteSession
- [ ] Replace `continue_agent_session()` with SDK-based version
- [ ] Remove manual history management code
- [ ] Update `get_session_info()` to work with SQLiteSession

#### 3.5 Rewrite Streaming (`agent.py` lines ~1100-1200)
- [ ] Replace `run_agent_query_stream()` with `Runner.run_streamed()`
- [ ] Update stream event handling
- [ ] Remove manual Gemini streaming code

#### 3.6 Update Health Check (`agent.py` lines ~1300-1400)
- [ ] Update `check_health()` to verify:
  - OpenRouter API key present
  - SQLiteSession database accessible
  - Agent instance initialized correctly

### Phase 4: Testing
- [ ] **Unit Tests** - Update test fixtures:
  ```bash
  # Update backend/tests/test_agent.py
  # - Mock LitellmModel instead of Gemini
  # - Mock Runner.run_sync() instead of genai calls
  # - Test retrieve_textbook_content tool directly
  ```

- [ ] **Integration Tests** - Run with OpenRouter:
  ```bash
  pytest backend/tests/test_agent_integration.py -v
  ```

- [ ] **Quickstart Validation** - Run full validation:
  ```bash
  python backend/quickstart_validation.py
  ```

- [ ] **Manual Testing** - Try 5 queries:
  ```python
  from backend.agent import run_agent_query

  queries = [
      "How do I install ROS 2?",
      "What is Gazebo simulation?",
      "Explain VLA models",
      "How to work with humanoid robots?",
      "What are the key concepts in robotics?"
  ]

  for query in queries:
      result = run_agent_query(query)
      print(f"\nQuery: {query}")
      print(f"Success: {result.retrieval_success}")
      print(f"Response: {result.response_text[:200]}...")
  ```

### Phase 5: Documentation
- [ ] Update `specs/002-rag-agent-integration/spec.md`:
  - Change "Gemini 2.0 Flash" to "Qwen via OpenRouter"
  - Add OpenRouter configuration details
  - Update LLM provider section

- [ ] Update `specs/002-rag-agent-integration/plan.md`:
  - Update agent.py architecture diagram
  - Add LitellmModel configuration details
  - Update dependencies list

- [ ] Update `specs/002-rag-agent-integration/quickstart.md`:
  - Replace Gemini examples with Qwen/OpenRouter
  - Update environment variable documentation
  - Update code examples to show SDK usage

---

## 4. File-by-File Changes

### `backend/agent.py`

| Section | Lines | Current (WRONG) | Target (CORRECT) | Priority |
|---------|-------|-----------------|------------------|----------|
| Imports | 1-30 | `import google.generativeai as genai` | `from agents import Agent, Runner, function_tool`<br>`from agents.extensions.models.litellm_model import LitellmModel` | CRITICAL |
| Model Cache | 74-76 | `_gemini_model_cache = genai.GenerativeModel(...)` | `_agent = Agent(...)` | CRITICAL |
| Retrieval Tool | N/A | (missing) | `@function_tool`<br>`def retrieve_textbook_content(...)` | CRITICAL |
| Response Gen 1 | 540-590 | `def _generate_rag_response(...)` | **DELETE** (handled by SDK) | CRITICAL |
| Response Gen 2 | 617-698 | `def _generate_rag_response_with_selected_text(...)` | **DELETE** (handled by SDK) | CRITICAL |
| Response Gen 3 | 995-1049 | `def _generate_rag_response_with_history(...)` | **DELETE** (handled by SDK) | CRITICAL |
| Query Function | 400-500 | Manual orchestration | `Runner.run_sync(agent, query, ...)` | CRITICAL |
| Session Dict | 765 | `_active_sessions: Dict[str, AgentSession] = {}` | **DELETE** (use SQLiteSession) | HIGH |
| Create Session | 770-800 | Manual session creation | `SQLiteSession(session_id, "conversations.db")` | HIGH |
| Continue Session | 850-900 | Manual history management | `Runner.run_sync(agent, query, session=session)` | HIGH |
| Streaming | 1100-1200 | Manual Gemini streaming | `Runner.run_streamed(agent, query, ...)` | MEDIUM |
| Health Check | 1300-1350 | Check Gemini API | Check OpenRouter API + SQLiteSession DB | LOW |

### `specs/002-rag-agent-integration/spec.md`

| Section | Lines | Change Required |
|---------|-------|-----------------|
| LLM Provider | 20-25 | Change "Gemini 2.0 Flash" → "Qwen 3 Coder (qwen/qwen3-coder:free) via OpenRouter" |
| Environment Vars | 35-45 | Replace `GEMINI_API_KEY` → `OPENROUTER_API_KEY` |
| Technical Constraints | 80-90 | Add "MUST use OpenRouter as LLM provider with base URL: https://openrouter.ai/api/v1" |
| Architecture | 140-150 | Update agent creation to show LitellmModel configuration |

### `specs/002-rag-agent-integration/plan.md`

| Section | Lines | Change Required |
|---------|-------|-----------------|
| Dependencies | 50-60 | Remove `google-generativeai`, add `litellm` |
| Agent Implementation | 200-250 | Replace Gemini code examples with SDK examples |
| Session Management | 300-350 | Replace manual session management with SQLiteSession |
| LLM Configuration | 400-420 | Add LitellmModel configuration details |

### `backend/.env.example`

| Change | Details |
|--------|---------|
| Add | `OPENROUTER_API_KEY=your_openrouter_key_here` |
| Update comment | "# LLM Provider: OpenRouter (https://openrouter.ai)" |

---

## 5. Testing Strategy

### Unit Tests (backend/tests/test_agent.py)

**Current Issues**:
- Mocks Gemini API instead of SDK
- Tests manual orchestration logic

**Required Changes**:
```python
import pytest
from unittest.mock import Mock, patch, AsyncMock
from agents import Agent, Runner
from agents.types import RunResult
from backend.agent import retrieve_textbook_content, run_agent_query

class TestRetrievalTool:
    """Test the @function_tool implementation."""

    def test_retrieve_textbook_content_success(self):
        """Test retrieval tool returns formatted chunks."""
        result = retrieve_textbook_content(
            query="How do I install ROS 2?",
            top_k=3,
            score_threshold=0.5
        )

        assert isinstance(result, str)
        assert len(result) > 0
        assert "[Source" in result  # Check formatting

    def test_retrieve_textbook_content_no_results(self):
        """Test retrieval tool handles no results gracefully."""
        result = retrieve_textbook_content(
            query="asdfghjkl nonsense query zxcvbnm",
            top_k=3,
            score_threshold=0.99  # Very high threshold
        )

        assert "No relevant content found" in result


class TestAgentQuery:
    """Test agent query execution."""

    @patch('backend.agent.Runner.run_sync')
    def test_run_agent_query_success(self, mock_run):
        """Test successful agent query."""
        # Mock SDK response
        mock_result = Mock(spec=RunResult)
        mock_result.final_output = "ROS 2 can be installed using apt-get..."
        mock_run.return_value = mock_result

        response = run_agent_query("How do I install ROS 2?")

        assert response.retrieval_success
        assert len(response.response_text) > 0
        assert response.response_time_ms > 0
        mock_run.assert_called_once()

    @patch('backend.agent.Runner.run_sync')
    def test_run_agent_query_with_selected_text(self, mock_run):
        """Test query with selected text context."""
        mock_result = Mock(spec=RunResult)
        mock_result.final_output = "Gazebo is a simulation environment..."
        mock_run.return_value = mock_result

        response = run_agent_query(
            query_text="Explain this",
            selected_text="Gazebo is a robot simulator..."
        )

        assert response.retrieval_success
        # Verify query was augmented with selected text
        call_args = mock_run.call_args
        assert "selected this text" in call_args.kwargs.get('query', '')


class TestAgentSession:
    """Test multi-turn conversations."""

    @patch('backend.agent.Runner.run_sync')
    def test_continue_agent_session(self, mock_run):
        """Test session continuation maintains context."""
        session_id = "test_session_123"

        # First query
        mock_result1 = Mock(spec=RunResult)
        mock_result1.final_output = "Gazebo is a simulation environment..."
        mock_run.return_value = mock_result1

        response1 = continue_agent_session(
            session_id=session_id,
            query_text="What is Gazebo?"
        )

        assert response1.retrieval_success

        # Second query (context maintained by SQLiteSession)
        mock_result2 = Mock(spec=RunResult)
        mock_result2.final_output = "To get started with Gazebo, first install..."
        mock_run.return_value = mock_result2

        response2 = continue_agent_session(
            session_id=session_id,
            query_text="How do I get started with it?"
        )

        assert response2.retrieval_success
        # Verify session was passed to Runner
        call_args = mock_run.call_args
        assert call_args.kwargs.get('session') is not None
```

### Integration Tests (backend/tests/test_agent_integration.py)

**New Test Cases**:
```python
import pytest
from backend.agent import run_agent_query, create_agent_session, continue_agent_session

class TestAgentIntegration:
    """Integration tests with real OpenRouter API."""

    def test_real_query_ros_installation(self):
        """Test real query about ROS 2 installation."""
        response = run_agent_query(
            query_text="How do I install ROS 2?",
            top_k=5,
            score_threshold=0.4
        )

        assert response.retrieval_success
        assert len(response.retrieved_chunks) > 0
        assert "ROS" in response.response_text or "ros" in response.response_text.lower()
        assert response.response_time_ms < 10000  # < 10 seconds

    def test_real_multi_turn_conversation(self):
        """Test multi-turn conversation with real API."""
        session = create_agent_session()

        # Turn 1
        response1 = continue_agent_session(
            session_id=session.session_id,
            query_text="What is Gazebo simulation?"
        )
        assert response1.retrieval_success

        # Turn 2 (reference to previous context)
        response2 = continue_agent_session(
            session_id=session.session_id,
            query_text="What are its main features?"
        )
        assert response2.retrieval_success
        # Should understand "its" refers to Gazebo from previous turn

    def test_tool_calling_behavior(self):
        """Verify agent calls retrieval tool correctly."""
        response = run_agent_query(
            query_text="Explain VLA models",
            top_k=3
        )

        # Agent should have called retrieve_textbook_content tool
        assert response.retrieval_success
        assert len(response.retrieved_chunks) > 0
        # Response should be based on retrieved content
        assert len(response.response_text) > 100
```

---

## 6. Rollback Plan

If the rewrite introduces critical bugs:

1. **Immediate Rollback**:
   ```bash
   git checkout backend/agent.py
   ```

2. **Restore Dependencies**:
   ```bash
   uv add google-generativeai
   uv remove openai-agents litellm
   ```

3. **Restore Environment**:
   - Re-enable `GEMINI_API_KEY` in `.env`

4. **Verify**:
   ```bash
   python backend/quickstart_validation.py
   ```

---

## 7. Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| OpenRouter API quota limits | Medium | High | Monitor usage, implement rate limiting |
| Qwen model performance < Gemini | Medium | Medium | Run A/B comparison, be prepared to switch to better Qwen variant |
| SQLiteSession DB corruption | Low | Medium | Regular backups, implement DB health checks |
| SDK breaking changes | Low | High | Pin SDK version, monitor release notes |
| Tool calling failures | Medium | High | Implement robust error handling in @function_tool |

---

## 8. Success Criteria

### Functional Requirements
- ✅ All 9 quickstart validation steps pass (T064)
- ✅ Agent uses OpenAI Agents SDK (not Gemini API)
- ✅ Agent uses Qwen via OpenRouter (not Gemini models)
- ✅ Retrieval integrated as `@function_tool`
- ✅ Multi-turn conversations use SQLiteSession
- ✅ No code duplication (single source of truth)

### Performance Requirements
- ✅ Response time < 5 seconds for 95% of queries
- ✅ Success rate 100% for validation queries
- ✅ Context maintained across 5+ turns

### Code Quality
- ✅ Zero references to `google.generativeai`
- ✅ Zero manual session management code
- ✅ Zero duplicated response generation functions
- ✅ All tests pass with new implementation

---

## 9. Timeline (No Time Estimates)

### Step 1: Rewrite Plan ✓ (COMPLETED)
- Create this document
- Review with stakeholder

### Step 2: Generate Corrected agent.py
- Apply all transformations from Section 2
- Implement all checklist items from Section 3
- Test each function independently

### Step 3: Update spec.md
- Apply changes from Section 4
- Add OpenRouter configuration details
- Update LLM provider documentation

### Step 4: Update plan.md
- Update architecture diagrams
- Update code examples
- Update dependencies list

---

## 10. Appendix: Quick Reference

### Old vs New: Key Function Signatures

| Function | Old (Gemini) | New (SDK) |
|----------|--------------|-----------|
| Generate response | `_generate_rag_response(query, chunks, ...)` | **DELETED** (use `Runner.run_sync()`) |
| Run query | `run_agent_query(query, api_key, model_name, ...)` | `run_agent_query(query, temperature, top_k, ...)` |
| Create session | `create_agent_session()` → stores in dict | `create_agent_session()` → creates SQLiteSession |
| Continue session | `continue_agent_session(...)` → manual history | `continue_agent_session(...)` → SDK auto-history |
| Streaming | `run_agent_query_stream()` → Gemini stream | `run_agent_query_stream()` → SDK stream |

### Environment Variables

| Variable | Old Usage | New Usage |
|----------|-----------|-----------|
| `GEMINI_API_KEY` | Required for agent | Not needed for agent (can keep for other uses) |
| `OPENROUTER_API_KEY` | Not used | **REQUIRED** for agent LLM |
| `QDRANT_URL` | Same | Same |
| `QDRANT_API_KEY` | Same | Same |
| `COHERE_API_KEY` | Same | Same |

### Imports Comparison

**Old (WRONG)**:
```python
import google.generativeai as genai
from typing import Optional, List, Dict, Any, Iterator
```

**New (CORRECT)**:
```python
from agents import Agent, Runner, function_tool
from agents.extensions.models.litellm_model import LitellmModel
from agents.types import ModelSettings, RunResult, ResponseTextDeltaEvent
from typing import Annotated, Optional, List, Dict, Any, AsyncIterator
```

---

## Summary

This rewrite plan provides a complete roadmap for migrating from the incorrect Gemini-based implementation to the correct OpenAI Agents SDK implementation with Qwen via OpenRouter.

**Key Changes**:
1. Replace all Gemini API code with OpenAI Agents SDK
2. Wrap retrieval in `@function_tool` decorator
3. Use `Agent` + `LitellmModel` for Qwen/OpenRouter integration
4. Replace manual orchestration with `Runner.run_sync()`
5. Replace manual session management with `SQLiteSession`
6. Update all documentation to reflect new architecture

**Next Step**: Review this plan and proceed to Step 2 (Generate corrected agent.py).
