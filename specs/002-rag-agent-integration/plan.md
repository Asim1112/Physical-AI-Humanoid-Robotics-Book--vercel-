# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create an AI agent using OpenAI Agents SDK that integrates with the existing RAG pipeline to process user queries about the Physical AI & Humanoid Robotics textbook. The agent will use Cohere embeddings for query processing and retrieve relevant content from Qdrant vector database before generating responses. The implementation will reuse existing retrieval functions from Specs 1-2, support multi-turn conversations, and handle selected-text mode for targeted queries.

## Technical Context

**Language/Version**: Python 3.13 (as established in previous specs)
**Primary Dependencies**: OpenAI Agents SDK (`openai-agents>=0.6.4`), OpenAI Python SDK (`openai` for AsyncOpenAI client), Cohere SDK (embeddings), Qdrant client (vector database), existing backend modules (retrieve.py, embedding.py, storage.py)
**LLM Model**: Llama 3.3 70B Versatile (`llama-3.3-70b-versatile`) via Groq (`https://api.groq.com/openai/v1`)
**Storage**: Qdrant vector database (existing from Spec 1), SQLiteSession for conversation history (`conversations.db`)
**Testing**: pytest (as established in previous specs)
**Target Platform**: Linux/Windows server environment
**Project Type**: Backend service integration (extension of existing backend)
**Performance Goals**: <5 second response time for 95% of queries (from spec SC-001), 100% success rate for 5+ test queries (from spec SC-002)
**Constraints**: Must reuse existing retrieval functions from Specs 1-2, integrate with OpenAI Agents SDK, support multi-turn conversations, handle API failures gracefully
**Scale/Scope**: Single agent service, integration with existing 1,278 vector chunks, 5+ test queries validation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I (Precision and Depth)**: ✅ VALID - Implementation will use official OpenAI Agents SDK documentation and existing backend modules, ensuring technical accuracy and verifiable implementation.

**Principle II (Consistency Across Artifacts)**: ✅ VALID - Agent will follow established patterns from Specs 1-2 for embedding, retrieval, and API interactions, maintaining consistent terminology and approaches.

**Principle III (Source-Aware Content)**: ✅ VALID - Will use official OpenAI Agents SDK documentation as authoritative source for agent implementation, and leverage existing functions from retrieve.py and embedding.py as established sources.

**Principle IV (Modularity)**: ✅ VALID - Agent implementation will be modular, potentially reusable for other RAG applications while maintaining standalone functionality.

**Principle V (Pedagogical Clarity)**: ✅ VALID - Implementation will follow clear patterns and include proper documentation for educational purposes as part of the textbook project.

**Principle VI (Spec-First Execution)**: ✅ VALID - Implementation strictly follows feature specification requirements from spec.md with no improvisation.

**Principle VII (Code Quality and Runnability)**: ✅ VALID - Implementation will include proper testing with 5+ sample queries as specified in the requirements.

**Structural & Architectural Rules**: ✅ VALID - Implementation will be in backend/ directory as a backend service, not affecting the frontend structure.

**Gate Status**: ✅ **PASSED** - All constitution checks pass, implementation may proceed.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-agent-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── agent.py             # Main agent implementation using OpenAI Agents SDK
├── retrieve.py          # Existing retrieval functions (from Spec 2, reused)
├── embedding.py         # Existing embedding functions (from Spec 1, reused)
├── storage.py           # Existing Qdrant functions (from Spec 1, reused)
├── main.py              # Updated to include agent orchestration
├── tests/
│   ├── test_agent.py        # Unit tests for agent functionality
│   ├── test_agent_integration.py  # Integration tests with retrieval
│   └── test_agent_retrieval.py    # Tests for retrieval integration
└── logs/
```

**Structure Decision**: Backend service integration following Option 2 pattern. The agent implementation will be in a single file (agent.py) that integrates with existing backend modules (retrieve.py, embedding.py, storage.py) to maintain consistency with the established architecture from Specs 1-2.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

## Architecture Overview

### Design Philosophy

The RAG Agent implementation follows the **OpenAI Agents SDK** architecture, which provides:
- **Provider-Agnostic Design**: Uses LitellmModel to support 100+ LLM providers, enabling flexibility in model selection
- **Tool-First Approach**: Retrieval is implemented as a function tool that the agent can invoke autonomously
- **Automatic Loop Management**: SDK handles the tool-calling loop (LLM call → tool execution → result → next LLM call)
- **Built-in Session Management**: SQLiteSession provides automatic conversation history persistence
- **Async-First Patterns**: Native async/await support for efficient I/O operations

### Core Components

```
┌─────────────────────────────────────────────────────────────┐
│                     Agent Instance                          │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Name: "Physical AI & Humanoid Robotics Assistant"  │  │
│  │  Instructions: System prompt with RAG guidelines     │  │
│  │  Model: OpenAIChatCompletionsModel (Groq Llama 3.3) │  │
│  │  Tools: [retrieve_textbook_content]                 │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Runner.run()                             │
│  - Executes agent loop asynchronously                       │
│  - Parameters: agent, input, session, model_settings        │
│  - Returns: Result with final_output                        │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              @function_tool: retrieve_textbook_content      │
│  - Accepts: query (str), top_k (int), score_threshold       │
│  - Calls: retrieve_content() from retrieve.py              │
│  - Returns: Formatted chunks with metadata                  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│           Existing Retrieval Pipeline (Reused)             │
│  - embedding.py: generate_embedding() with Cohere          │
│  - retrieve.py: retrieve_content() with Qdrant             │
│  - storage.py: Qdrant client management                     │
└─────────────────────────────────────────────────────────────┘
```

### Agent Implementation Pattern

**Technology Stack**:
- **Agent Framework**: OpenAI Agents SDK (`openai-agents>=0.6.4`)
- **LLM Provider**: Groq (`https://api.groq.com/openai/v1`)
- **LLM Model**: Llama 3.3 70B Versatile (`llama-3.3-70b-versatile`)
  - **Note**: Groq provides OpenAI-compatible endpoints, integrated via AsyncOpenAI client
  - Ranked #1 on Berkeley Function Calling Leaderboard for tool use
  - 14,400 free requests/day, 300+ tokens/second inference speed
- **Session Storage**: SQLiteSession (built-in, uses `conversations.db`)
- **Embeddings**: Cohere API (existing from Spec 1)
- **Vector Database**: Qdrant (existing from Spec 1)

**Key Implementation Files**:

#### backend/agent.py

**1. Imports and Setup**
```python
from agents import Agent, Runner, function_tool, SQLiteSession, ModelSettings, RunConfig, set_tracing_disabled, OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from backend.retrieve import retrieve_content
from backend.embedding import generate_embedding
import os
import asyncio
```

**2. Function Tool Definition**
```python
@function_tool
def retrieve_textbook_content(
    query: Annotated[str, "The user's question about the textbook"],
    top_k: Annotated[int, "Number of chunks to retrieve"] = 5,
    score_threshold: Annotated[float, "Minimum similarity score"] = 0.5
) -> str:
    """Retrieve relevant content from the Physical AI & Humanoid Robotics textbook."""

    # Call existing retrieval function
    result = retrieve_content(
        query_text=query,
        client=qdrant_client,
        collection_name=os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook"),
        top_k=top_k,
        score_threshold=score_threshold,
        api_key=os.getenv("COHERE_API_KEY")
    )

    # Format chunks for agent
    formatted_chunks = []
    for i, chunk in enumerate(result.retrieved_chunks, 1):
        formatted_chunks.append(
            f"[Source {i}: {chunk.module_name} - {chunk.section_heading}]\n"
            f"{chunk.content_text}\n"
            f"(Similarity score: {chunk.similarity_score:.3f})\n"
        )

    return "\n".join(formatted_chunks)
```

**3. Agent Instance Creation (Singleton)**
```python
_agent_instance = None

def _get_agent_instance() -> Agent:
    """Get or create the singleton agent instance."""
    global _agent_instance

    if _agent_instance is None:
        _agent_instance = Agent(
            name="Physical AI & Humanoid Robotics Assistant",
            instructions="""You are an expert AI assistant specializing in Physical AI and Humanoid Robotics.

Your role:
- Answer questions about robotics, ROS 2, Gazebo simulation, VLA models, and humanoid robots
- Base your answers on the textbook content retrieved via the retrieve_textbook_content tool
- Cite specific modules, sections, or sources when referencing the textbook
- Be clear, accurate, and educational in your explanations

Critical Instructions:
1. ALWAYS use the retrieve_textbook_content tool for every user question
2. Analyze the retrieved content carefully before answering
3. Provide comprehensive answers based on the textbook material
4. Cite sources explicitly (e.g., "According to Module 2: ROS 2 Basics...")
5. If answering from general knowledge, state this clearly
6. When the user refers to "it", "this", "that", use conversation context
7. Be concise but thorough - aim for clarity over brevity""",

            model=LitellmModel(
                model="openrouter/qwen/qwen3-coder:free",
                api_key=os.getenv("OPENROUTER_API_KEY")
            ),

            tools=[retrieve_textbook_content]
        )

    return _agent_instance
```

**4. Async Query Execution**
```python
async def run_agent_query_async(
    query_text: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None,
    temperature: float = 0.5,
    max_turns: int = 10,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 2000,
    api_key: Optional[str] = None
) -> AgentResponse:
    """Execute a query with the RAG agent using OpenAI Agents SDK."""

    # Augment query if selected text provided
    if selected_text and selected_text.strip():
        truncated_selection = selected_text[:1000]
        augmented_query = f'''The user has selected this text from the textbook:

"""{truncated_selection}"""

The user asks: {query_text}

Please retrieve relevant textbook content and answer based on both the selected text and additional information.'''
    else:
        augmented_query = query_text

    # Configure run settings
    run_config = RunConfig(
        model_settings=ModelSettings(
            temperature=temperature,
            max_tokens=max_tokens
        )
    )

    # Get agent instance
    agent = _get_agent_instance()

    # Run agent (SDK handles tool calling automatically)
    result = await Runner.run(
        agent,
        augmented_query,
        run_config=run_config
    )

    # Extract response
    response_text = result.final_output if hasattr(result, 'final_output') else str(result)

    return AgentResponse(
        response_text=response_text,
        retrieved_chunks=[],
        conversation_id=f"conv_{uuid.uuid4().hex[:16]}",
        response_time_ms=response_time_ms,
        retrieval_success=True,
        error_message=None
    )
```

**5. Synchronous Wrapper**
```python
def run_agent_query(query_text: str, **kwargs) -> AgentResponse:
    """Synchronous wrapper for backward compatibility."""
    return asyncio.run(run_agent_query_async(query_text, **kwargs))
```

**6. Session Management with SQLiteSession**
```python
async def continue_agent_session_async(
    session_id: str,
    query_text: str,
    selected_text: Optional[str] = None,
    temperature: float = 0.5,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 2000,
    api_key: Optional[str] = None
) -> AgentResponse:
    """Continue an existing agent session with conversation history."""

    # SQLiteSession automatically loads conversation history
    session = SQLiteSession(session_id, "conversations.db")

    # Augment query if selected text provided
    augmented_query = query_text
    if selected_text and selected_text.strip():
        truncated_selection = selected_text[:1000]
        augmented_query = f'''The user has selected this text:

"""{truncated_selection}"""

The user asks: {query_text}'''

    # Configure run settings
    run_config = RunConfig(
        model_settings=ModelSettings(temperature=temperature, max_tokens=max_tokens)
    )

    # Get agent instance
    agent = _get_agent_instance()

    # Run agent with session (SDK manages history automatically)
    result = await Runner.run(
        agent,
        augmented_query,
        session=session,  # SDK automatically includes history!
        run_config=run_config
    )

    # Extract response
    response_text = result.final_output if hasattr(result, 'final_output') else str(result)

    return AgentResponse(
        response_text=response_text,
        retrieved_chunks=[],
        conversation_id=session_id,
        response_time_ms=response_time_ms,
        retrieval_success=True,
        error_message=None
    )
```

### Dependencies

**Required Python Packages** (from `backend/pyproject.toml`):

```toml
[project]
dependencies = [
    "openai-agents[litellm]>=0.6.4",  # OpenAI Agents SDK with LiteLLM support
    "cohere>=5.20.1",                  # Embeddings (existing)
    "qdrant-client>=1.16.2",           # Vector database (existing)
    "python-dotenv>=1.2.1",            # Environment variables
    "tiktoken>=0.12.0",                # Token counting
    "requests>=2.32.5",                # HTTP requests
    "markdown-it-py>=4.0.0"            # Markdown processing
]
```

**Key Dependency Changes from Previous Implementation**:
- ✅ **Added**: `openai-agents[litellm]>=0.6.4` (OpenAI Agents SDK with LiteLLM extension)
- ❌ **Removed**: `google-generativeai` (no longer using Gemini API directly)
- ✅ **Retained**: All existing dependencies (cohere, qdrant-client, etc.) for retrieval pipeline

### Data Flow

**Single-Turn Query Flow**:
```
1. User Query → run_agent_query(query_text)
                      ↓
2. Augment query (if selected_text provided)
                      ↓
3. Create ModelSettings (temperature, max_tokens)
                      ↓
4. Get Agent Instance (singleton with LitellmModel)
                      ↓
5. await Runner.run(agent, query, run_config=RunConfig(model_settings=settings))
                      ↓
6. SDK Loop:
   a. Send query to Qwen LLM via OpenRouter
   b. LLM decides to call retrieve_textbook_content tool
   c. Tool executes: query → Cohere embedding → Qdrant search → formatted chunks
   d. Tool result returned to LLM
   e. LLM generates final response based on retrieved content
   f. SDK returns Result with final_output
                      ↓
7. Extract response_text from result.final_output
                      ↓
8. Return AgentResponse(response_text, metadata, ...)
```

**Multi-Turn Conversation Flow**:
```
1. User Query + session_id → continue_agent_session_async(session_id, query)
                      ↓
2. Create SQLiteSession(session_id, "conversations.db")
   - SDK automatically loads previous turns from database
                      ↓
3. Augment query (if selected_text provided)
                      ↓
4. Get Agent Instance
                      ↓
5. await Runner.run(agent, query, session=session, run_config=RunConfig(model_settings=settings))
   - SDK includes conversation history in context automatically
                      ↓
6. SDK Loop (same as single-turn, but with conversation context)
                      ↓
7. Extract response_text
   - SDK automatically saves new turn to database
                      ↓
8. Return AgentResponse with same session_id
```

**Streaming Flow**:
```
1. User Query → run_agent_query_streaming(query_text)
                      ↓
2. Setup (augment query, create session if provided)
                      ↓
3. result = Runner.run_streamed(agent, query, session=session, run_config=run_config)
                      ↓
4. async for event in result.stream_events():
   - Yield response chunks as they arrive
   - Event types: raw_response_event, tool_call_event, etc.
                      ↓
5. Accumulate full_response from chunks
                      ↓
6. Yield final metadata (response_time_ms, response_length)
```

### Session Management Strategy

**Session Storage**: SQLiteSession (built-in)
- Database file: `conversations.db` (created automatically)
- Schema: Managed by SDK (no manual table creation needed)
- Persistence: Automatic on each `Runner.run()` call with session parameter

**Session Lifecycle**:
1. **Create**: `session = SQLiteSession(session_id, "conversations.db")`
2. **Load History**: Automatically loaded when passed to `Runner.run()`
3. **Save Turn**: Automatically saved after each successful `Runner.run()`
4. **Retrieve History**: `messages = await session.get_items()` (for debugging/export)

**Session ID Generation**:
- New conversations: `f"conv_{uuid.uuid4().hex[:16]}"` (16-character hex)
- Continued conversations: Client provides existing session_id

### Error Handling Strategy

**Levels of Resilience**:

1. **Retrieval Tool Level**:
   - Catches retrieval errors
   - Returns error message as tool output (LLM sees the error and can respond appropriately)
   - Logs errors for debugging

2. **Agent Execution Level**:
   - Catches SDK execution errors
   - Returns AgentResponse with error_message populated
   - Ensures API never crashes

3. **API Integration Level**:
   - Validates AgentRequest before processing
   - Returns structured error responses
   - Maintains consistent response format

**Error Scenarios Handled**:
- Qdrant connection failures → Tool returns error message
- Cohere API failures → Tool returns error message with retry logic
- OpenRouter API failures → SDK handles retries, falls back to error response
- No retrieval results → Tool returns "No relevant content found"
- Session database errors → Graceful fallback to stateless mode
- Token limit exceeded → Truncate context or selected text
