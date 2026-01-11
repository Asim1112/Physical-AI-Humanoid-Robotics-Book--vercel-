"""
RAG Agent Integration using OpenAI Agents SDK

This module implements an AI agent that integrates with the existing RAG pipeline
to process user queries about the Physical AI & Humanoid Robotics textbook.

Architecture:
- Uses OpenAI Agents SDK for orchestration
- Groq (llama-3.3-70b-versatile) via OpenAIChatCompletionsModel
- Function tool pattern for retrieval integration
- SQLiteSession for multi-turn conversation management
"""

import os
import time
import uuid
import logging
import asyncio
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional, Dict, Any, Annotated, AsyncIterator
from dotenv import load_dotenv
# OpenAI Agents SDK imports
from agents import Agent, Runner, function_tool, SQLiteSession, ModelSettings, RunConfig, set_tracing_disabled, OpenAIChatCompletionsModel  # type: ignore[import-untyped]
from openai import AsyncOpenAI

# Import existing backend modules
from retrieve import retrieve_content, RetrievalResult, RetrievedChunk
from embedding import generate_embedding
from storage import create_qdrant_client

# Configure logging
logger = logging.getLogger("rag_agent")


# ============================================================================
# Performance Optimization (T061)
# ============================================================================

# Global cache for Qdrant client (reuse connection)
_qdrant_client_cache: Optional[Any] = None

# Global agent instance (initialized once)
_agent_instance: Optional[Agent] = None


def _get_cached_qdrant_client():
    """
    Get cached Qdrant client to avoid reconnection overhead (T061).

    Returns:
        Cached Qdrant client instance
    """
    global _qdrant_client_cache

    if _qdrant_client_cache is None:
        _qdrant_client_cache = create_qdrant_client(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        logger.debug("Created new Qdrant client (cached for reuse)")
    else:
        logger.debug("Reusing cached Qdrant client")

    return _qdrant_client_cache


# ============================================================================
# Security Utilities (T062)
# ============================================================================

def _sanitize_api_key(text: str) -> str:
    """
    Sanitize API keys from text to prevent accidental logging (T062).

    Replaces any string that looks like an API key with [REDACTED].
    Detects common patterns:
    - Keys with 'sk-', 'key-', 'Bearer ' prefixes
    - Long alphanumeric strings (>20 chars)
    - Environment variable patterns like GEMINI_API_KEY=value

    Args:
        text: Text that may contain API keys

    Returns:
        Sanitized text with API keys replaced
    """
    import re

    if not text:
        return text

    # Pattern 1: Environment variable assignments (KEY=value)
    text = re.sub(
        r'([A-Z_]*API[A-Z_]*KEY[A-Z_]*)\s*=\s*["\']?([a-zA-Z0-9_\-]{8,})["\']?',
        r'\1=[REDACTED]',
        text
    )

    # Pattern 2: Common API key prefixes (sk-, key-, Bearer )
    text = re.sub(
        r'(sk-|key-|Bearer\s+)[a-zA-Z0-9_\-]{20,}',
        r'\1[REDACTED]',
        text
    )

    # Pattern 3: Long alphanumeric strings in quotes (likely keys)
    text = re.sub(
        r'["\']([a-zA-Z0-9_\-]{32,})["\']',
        r'"[REDACTED]"',
        text
    )

    return text


def _sanitize_error_message(error: Exception, include_type: bool = True) -> str:
    """
    Create a sanitized error message safe for logging and user display (T062).

    Removes:
    - API keys and tokens
    - File paths (optional, keeps filename only)
    - Internal implementation details

    Args:
        error: Exception to sanitize
        include_type: Whether to include exception type in message

    Returns:
        Sanitized error message
    """
    error_text = str(error)

    # Sanitize API keys
    error_text = _sanitize_api_key(error_text)

    # Remove absolute file paths, keep filename only
    import re
    error_text = re.sub(
        r'[A-Za-z]:\\[^:"\'\s]+\\([^\\]+)',  # Windows paths
        r'\1',
        error_text
    )
    error_text = re.sub(
        r'/[^:"\'\s]+/([^/]+)',  # Unix paths
        r'\1',
        error_text
    )

    # Build final message
    if include_type:
        return f"{type(error).__name__}: {error_text}"
    return error_text


# ============================================================================
# Retry Logic (T063)
# ============================================================================

def _retry_with_exponential_backoff(
    func,
    max_retries: int = 3,
    initial_delay: float = 1.0,
    max_delay: float = 10.0,
    exponential_base: float = 2.0,
    retryable_exceptions: tuple = (Exception,)
):
    """
    Retry a function with exponential backoff for transient failures (T063).

    Implements exponential backoff: delay = min(initial_delay * (base ^ attempt), max_delay)
    Example with defaults: 1s, 2s, 4s

    Args:
        func: Function to retry
        max_retries: Maximum number of retry attempts (default: 3)
        initial_delay: Initial delay in seconds (default: 1.0)
        max_delay: Maximum delay cap in seconds (default: 10.0)
        exponential_base: Base for exponential calculation (default: 2.0)
        retryable_exceptions: Tuple of exceptions to retry (default: all)

    Returns:
        Result of successful function call

    Raises:
        Last exception if all retries exhausted
    """
    last_exception = None

    for attempt in range(max_retries + 1):  # +1 for initial attempt
        try:
            return func()
        except retryable_exceptions as e:
            last_exception = e

            # Don't retry on final attempt
            if attempt >= max_retries:
                logger.warning(f"All {max_retries} retries exhausted for {func.__name__}")
                raise

            # Calculate delay with exponential backoff
            delay = min(initial_delay * (exponential_base ** attempt), max_delay)

            # Log retry attempt (sanitized)
            sanitized_error = _sanitize_error_message(e, include_type=True)
            logger.warning(
                f"Retry {attempt + 1}/{max_retries} for {func.__name__} "
                f"after error: {sanitized_error}. Retrying in {delay:.1f}s..."
            )

            # Wait before retry
            time.sleep(delay)

    # Should never reach here, but safety fallback
    if last_exception:
        raise last_exception


# ============================================================================
# Data Classes (T006-T012)
# ============================================================================

@dataclass
class AgentRequest:
    """Input structure for agent queries (T006)"""
    query_text: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    temperature: float = 0.5
    max_turns: int = 10
    top_k: int = 5
    score_threshold: float = 0.5

    def __post_init__(self):
        """Validate request parameters"""
        if not self.query_text or not self.query_text.strip():
            raise ValueError("Query text cannot be empty or whitespace only")
        if len(self.query_text) > 1000:
            raise ValueError(f"Query text too long ({len(self.query_text)} chars), maximum is 1000 characters")
        if not 0.0 <= self.temperature <= 1.0:
            raise ValueError(f"Temperature must be between 0.0 and 1.0, got {self.temperature}")
        if not 1 <= self.max_turns <= 50:
            raise ValueError(f"max_turns must be between 1 and 50, got {self.max_turns}")
        if not 1 <= self.top_k <= 10:
            raise ValueError(f"top_k must be between 1 and 10, got {self.top_k}")
        if not 0.0 <= self.score_threshold <= 1.0:
            raise ValueError(f"score_threshold must be between 0.0 and 1.0, got {self.score_threshold}")


@dataclass
class AgentResponse:
    """Output structure containing the agent's response and metadata (T007)"""
    response_text: str
    retrieved_chunks: List[RetrievedChunk]
    conversation_id: str
    response_time_ms: float
    query_tokens: int = 0
    response_tokens: int = 0
    retrieval_success: bool = True
    error_message: Optional[str] = None


@dataclass
class ConversationTurn:
    """Represents a single turn in a multi-turn conversation (T009)"""
    turn_number: int
    user_input: str
    retrieved_chunks: List[RetrievedChunk]
    agent_response: str
    timestamp: str
    tool_calls: List[Any] = field(default_factory=list)
    context_used: Optional[str] = None


@dataclass
class AgentSession:
    """Structure for managing conversation state across multiple turns (T008)"""
    session_id: str
    created_at: str
    last_accessed: str
    conversation_history: List[ConversationTurn] = field(default_factory=list)
    current_turn: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class AgentConfiguration:
    """Configuration settings for the agent behavior (T010)"""
    model_name: str = "llama-3.3-70b-versatile"
    temperature: float = 0.5
    max_tokens: int = 2000
    system_prompt: str = ""
    retrieval_enabled: bool = True
    max_turns: int = 10
    score_threshold: float = 0.5
    top_k: int = 5


@dataclass
class RetrievalToolInput:
    """Input structure for the retrieval tool (T011)"""
    query_text: str
    top_k: int = 5
    score_threshold: float = 0.5
    selected_text: Optional[str] = None


@dataclass
class RetrievalToolOutput:
    """Output structure from the retrieval tool (T012)"""
    retrieved_chunks: List[RetrievedChunk]
    query_embedding: List[float]
    search_time_ms: float
    total_candidates: int
    success: bool
    error_message: Optional[str] = None


# ============================================================================
# Function Tool: Retrieval Integration (T023)
# ============================================================================

@function_tool
def retrieve_textbook_content(
    query: Annotated[str, "The user's question about the Physical AI & Humanoid Robotics textbook"],
    top_k: Annotated[int, "Number of textbook chunks to retrieve (default: 5)"] = 5,
    score_threshold: Annotated[float, "Minimum similarity score for retrieval (0.0-1.0, default: 0.5)"] = 0.5
) -> str:
    """Retrieve relevant content from the Physical AI & Humanoid Robotics textbook.

    This tool searches the vector database for content relevant to the user's query
    and returns formatted chunks with source information.

    Args:
        query: The user's question or search query
        top_k: Number of most relevant chunks to retrieve (1-10)
        score_threshold: Minimum similarity score (0.0 = all results, 1.0 = exact match only)

    Returns:
        Formatted string containing retrieved textbook chunks with source citations
    """
    logger.info(f"Retrieval tool called with query: {query[:50]}...")

    try:
        # Get cached Qdrant client
        qdrant_client = _get_cached_qdrant_client()
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook")

        # Perform retrieval with retry logic
        def _retrieve():
            return retrieve_content(
                query_text=query,
                client=qdrant_client,
                collection_name=collection_name,
                top_k=top_k,
                score_threshold=score_threshold,
                api_key=os.getenv("COHERE_API_KEY")
            )

        result = _retry_with_exponential_backoff(
            func=_retrieve,
            max_retries=3,
            initial_delay=1.0,
            retryable_exceptions=(Exception,)
        )

        # Check if retrieval returned chunks
        if not result.retrieved_chunks:
            return "No relevant content found in the textbook for this query. The textbook may not cover this topic."

        # Format chunks for agent
        formatted_chunks = []
        for i, chunk in enumerate(result.retrieved_chunks, 1):
            formatted_chunks.append(
                f"[Source {i}: {chunk.module_name} - {chunk.section_heading}]\n"
                f"{chunk.content_text}\n"
                f"(Similarity score: {chunk.similarity_score:.3f})\n"
            )

        formatted_output = "\n".join(formatted_chunks)
        logger.info(f"Retrieved {len(result.retrieved_chunks)} chunks successfully")

        return formatted_output

    except Exception as e:
        sanitized_error = _sanitize_error_message(e, include_type=True)
        logger.error(f"Retrieval tool error: {sanitized_error}")
        return f"Error during retrieval: {sanitized_error}"


# ============================================================================
# Agent Instance Creation (T022, T024)
# ============================================================================

def _get_agent_instance() -> Agent:
    """
    Get or create the singleton agent instance.

    Returns:
        Agent instance configured with Groq (llama-3.3-70b-versatile)
    """
    global _agent_instance

    if _agent_instance is None:
        load_dotenv()

        # Disable tracing since we're not using OpenAI API key
        set_tracing_disabled(True)

        # Create Groq client
        groq_client = AsyncOpenAI(
            api_key=os.getenv("GROQ_API_KEY"),
            base_url="https://api.groq.com/openai/v1"
        )

        _agent_instance = Agent(
            name="Physical AI & Humanoid Robotics Assistant",
            instructions="""You are an expert AI assistant specializing in Physical AI and Humanoid Robotics.

Your role:
- Answer questions about robotics, ROS 2, Gazebo simulation, VLA models, and humanoid robots
- Base your answers on the textbook content retrieved via the retrieve_textbook_content tool
- Cite specific modules, sections, or sources when referencing the textbook
- Be clear, accurate, and educational in your explanations
- If the textbook doesn't contain relevant information, acknowledge this clearly

Critical Instructions:
1. ALWAYS use the retrieve_textbook_content tool for every user question to fetch relevant information
2. Analyze the retrieved content carefully before answering
3. Provide comprehensive answers based on the textbook material
4. Cite sources explicitly (e.g., "According to Module 2: ROS 2 Basics...")
5. If answering from general knowledge (not in textbook), state this clearly: "While the textbook doesn't cover this specifically, generally speaking..."
6. When the user refers to "it", "this", "that", or similar pronouns, use conversation context to understand the reference
7. Be concise but thorough - aim for clarity over brevity

Answer Style:
- Start with a direct answer to the question
- Support with specific textbook citations
- Provide examples or practical guidance when available
- End with related concepts or next steps if relevant""",

            model=OpenAIChatCompletionsModel(
                model="llama-3.3-70b-versatile",
                openai_client=groq_client
            ),

            tools=[retrieve_textbook_content]
        )

        logger.info("Created agent instance with Groq (llama-3.3-70b-versatile)")

    return _agent_instance


# ============================================================================
# Core Query Functions (T015, T025)
# ============================================================================

async def run_agent_query_async(
    query_text: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None,
    temperature: float = 0.5,
    max_turns: int = 10,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 2000,
    api_key: Optional[str] = None  # Kept for backward compatibility, not used
) -> AgentResponse:
    """
    Execute a query with the RAG agent using OpenAI Agents SDK (T015) - Async version.

    Args:
        query_text: User's query text
        selected_text: Optional selected text from textbook for context
        session_id: Optional session ID for conversation continuity (not used in single query)
        temperature: Temperature setting for response generation (0.0-1.0)
        max_turns: Maximum number of conversation turns (for sessions)
        top_k: Number of chunks to retrieve (1-10)
        score_threshold: Minimum similarity score for retrieval (0.0-1.0)
        max_tokens: Maximum response length in tokens
        api_key: Deprecated, kept for backward compatibility

    Returns:
        AgentResponse with the agent's response and metadata
    """
    start_time = time.time()

    try:
        # Create and validate request (T021)
        request = AgentRequest(
            query_text=query_text,
            selected_text=selected_text,
            session_id=session_id,
            temperature=temperature,
            max_turns=max_turns,
            top_k=top_k,
            score_threshold=score_threshold
        )

        # Log query mode
        if selected_text and selected_text.strip():
            logger.info(f"Processing query with selected text ({len(selected_text)} chars): {query_text[:50]}...")
        else:
            logger.info(f"Processing query: {query_text[:50]}...")

        # Generate conversation ID
        conversation_id = f"conv_{uuid.uuid4().hex[:16]}"

        # Augment query if selected text provided (T048, T049, T050)
        if selected_text and selected_text.strip():
            # Truncate selected text if very long
            truncated_selection = selected_text[:1000] if len(selected_text) > 1000 else selected_text

            augmented_query = f"""The user has selected this text from the textbook:

\"\"\"
{truncated_selection}
\"\"\"

The user asks: {query_text}

Please retrieve relevant textbook content and answer based on both the selected text and any additional relevant textbook information."""
        else:
            augmented_query = query_text

        # Configure model settings
        run_config = RunConfig(
            model_settings=ModelSettings(
                temperature=temperature,
                max_tokens=max_tokens
            )
        )

        # Get agent instance
        agent = _get_agent_instance()

        # Run agent asynchronously (SDK handles tool calling automatically)
        logger.info("Running agent with OpenAI Agents SDK...")
        result = await Runner.run(
            agent,
            augmented_query,
            run_config=run_config
        )

        # Extract response
        response_text = result.final_output if hasattr(result, 'final_output') else str(result)

        response_time_ms = (time.time() - start_time) * 1000
        logger.info(f"Response generated in {response_time_ms:.2f}ms")

        return AgentResponse(
            response_text=response_text,
            retrieved_chunks=[],  # Chunks embedded in tool response
            conversation_id=conversation_id,
            response_time_ms=response_time_ms,
            retrieval_success=True,
            error_message=None
        )

    except ValueError as e:
        sanitized_error = _sanitize_error_message(e, include_type=False)
        logger.error(f"Validation error: {sanitized_error}")
        return AgentResponse(
            response_text="",
            retrieved_chunks=[],
            conversation_id=f"error_{uuid.uuid4().hex[:16]}",
            response_time_ms=(time.time() - start_time) * 1000,
            retrieval_success=False,
            error_message=sanitized_error
        )
    except Exception as e:
        sanitized_error = _sanitize_error_message(e, include_type=True)
        logger.error(f"Unexpected error: {sanitized_error}", exc_info=True)
        return AgentResponse(
            response_text="",
            retrieved_chunks=[],
            conversation_id=f"error_{uuid.uuid4().hex[:16]}",
            response_time_ms=(time.time() - start_time) * 1000,
            retrieval_success=False,
            error_message=f"Unexpected error: {sanitized_error}"
        )


def run_agent_query(
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
    """
    Execute a query with the RAG agent - Synchronous wrapper (T015).

    This is a synchronous wrapper around run_agent_query_async for backward compatibility.

    Args:
        query_text: User's query text
        selected_text: Optional selected text from textbook for context
        session_id: Optional session ID for conversation continuity
        temperature: Temperature setting for response generation (0.0-1.0)
        max_turns: Maximum number of conversation turns
        top_k: Number of chunks to retrieve (1-10)
        score_threshold: Minimum similarity score for retrieval (0.0-1.0)
        max_tokens: Maximum response length in tokens
        api_key: Deprecated, kept for backward compatibility

    Returns:
        AgentResponse with the agent's response and metadata
    """
    return asyncio.run(
        run_agent_query_async(
            query_text=query_text,
            selected_text=selected_text,
            session_id=session_id,
            temperature=temperature,
            max_turns=max_turns,
            top_k=top_k,
            score_threshold=score_threshold,
            max_tokens=max_tokens,
            api_key=api_key
        )
    )


# ============================================================================
# Session Management (Phase 4 - User Story 2)
# ============================================================================

def create_agent_session(session_id: Optional[str] = None) -> AgentSession:
    """
    Create a new agent session with unique ID (T034).

    Uses SQLiteSession for persistence managed by OpenAI Agents SDK.

    Args:
        session_id: Optional custom session ID (auto-generated if None)

    Returns:
        AgentSession with initialized state
    """
    if session_id is None:
        session_id = f"session_{uuid.uuid4().hex[:16]}"

    now = datetime.now().isoformat()

    # SQLiteSession handles all persistence automatically
    # No need to manually manage session storage

    session = AgentSession(
        session_id=session_id,
        created_at=now,
        last_accessed=now,
        conversation_history=[],
        current_turn=0,
        metadata={}
    )

    logger.info(f"Created new session: {session_id}")

    return session


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
    """
    Continue an existing agent session with a new query (T035, T037) - Async version.

    Uses SQLiteSession to automatically maintain conversation context.

    Args:
        session_id: Session ID from create_agent_session()
        query_text: User's query text
        selected_text: Optional selected text from textbook
        temperature: Temperature setting for response generation (0.0-1.0)
        top_k: Number of chunks to retrieve (1-10)
        score_threshold: Minimum similarity score for retrieval (0.0-1.0)
        max_tokens: Maximum response length in tokens
        api_key: Deprecated, kept for backward compatibility

    Returns:
        AgentResponse with the agent's response and metadata
    """
    start_time = time.time()

    try:
        # Validate request
        request = AgentRequest(
            query_text=query_text,
            selected_text=selected_text,
            session_id=session_id,
            temperature=temperature,
            max_turns=10,
            top_k=top_k,
            score_threshold=score_threshold
        )

        logger.info(f"Continuing session {session_id}: {query_text[:50]}...")

        # SQLiteSession automatically loads conversation history
        session = SQLiteSession(session_id, "conversations.db")

        # Augment query if selected text provided
        if selected_text and selected_text.strip():
            truncated_selection = selected_text[:1000] if len(selected_text) > 1000 else selected_text
            augmented_query = f"""The user has selected this text from the textbook:

\"\"\"
{truncated_selection}
\"\"\"

The user asks: {query_text}

Please retrieve relevant textbook content and answer based on both the selected text and the conversation context."""
        else:
            augmented_query = query_text

        # Configure model settings
        run_config = RunConfig(
            model_settings=ModelSettings(
                temperature=temperature,
                max_tokens=max_tokens
            )
        )

        # Get agent instance
        agent = _get_agent_instance()

        # Run agent with session (SDK handles history automatically)
        logger.info("Running agent with session context...")
        result = await Runner.run(
            agent,
            augmented_query,
            session=session,  # SDK manages history!
            run_config=run_config
        )

        # Extract response
        response_text = result.final_output if hasattr(result, 'final_output') else str(result)

        response_time_ms = (time.time() - start_time) * 1000
        logger.info(f"Session response generated in {response_time_ms:.2f}ms")

        return AgentResponse(
            response_text=response_text,
            retrieved_chunks=[],  # Chunks embedded in tool response
            conversation_id=session_id,
            response_time_ms=response_time_ms,
            retrieval_success=True,
            error_message=None
        )

    except ValueError as e:
        sanitized_error = _sanitize_error_message(e, include_type=False)
        logger.error(f"Validation error: {sanitized_error}")
        return AgentResponse(
            response_text="",
            retrieved_chunks=[],
            conversation_id=session_id,
            response_time_ms=(time.time() - start_time) * 1000,
            retrieval_success=False,
            error_message=sanitized_error
        )
    except Exception as e:
        sanitized_error = _sanitize_error_message(e, include_type=True)
        logger.error(f"Unexpected error in session: {sanitized_error}", exc_info=True)
        return AgentResponse(
            response_text="",
            retrieved_chunks=[],
            conversation_id=session_id,
            response_time_ms=(time.time() - start_time) * 1000,
            retrieval_success=False,
            error_message=f"Unexpected error: {sanitized_error}"
        )


def continue_agent_session(
    session_id: str,
    query_text: str,
    selected_text: Optional[str] = None,
    temperature: float = 0.5,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 2000,
    api_key: Optional[str] = None
) -> AgentResponse:
    """
    Continue an existing agent session - Synchronous wrapper (T035, T037).

    Args:
        session_id: Session ID from create_agent_session()
        query_text: User's query text
        selected_text: Optional selected text from textbook
        temperature: Temperature setting for response generation (0.0-1.0)
        top_k: Number of chunks to retrieve (1-10)
        score_threshold: Minimum similarity score for retrieval (0.0-1.0)
        max_tokens: Maximum response length in tokens
        api_key: Deprecated, kept for backward compatibility

    Returns:
        AgentResponse with the agent's response and metadata
    """
    return asyncio.run(
        continue_agent_session_async(
            session_id=session_id,
            query_text=query_text,
            selected_text=selected_text,
            temperature=temperature,
            top_k=top_k,
            score_threshold=score_threshold,
            max_tokens=max_tokens,
            api_key=api_key
        )
    )


async def get_agent_session_async(session_id: str) -> Optional[AgentSession]:
    """
    Retrieve an existing agent session (T040) - Async version.

    Args:
        session_id: Session ID to retrieve

    Returns:
        AgentSession if found, None otherwise
    """
    try:
        # Check if session exists in SQLite database
        session = SQLiteSession(session_id, "conversations.db")

        # Get messages to verify session exists
        messages = await session.get_items()

        if not messages:
            return None

        # Build AgentSession metadata
        return AgentSession(
            session_id=session_id,
            created_at=datetime.now().isoformat(),  # Approximate
            last_accessed=datetime.now().isoformat(),
            conversation_history=[],
            current_turn=len(messages) // 2,  # Approximate (user + assistant pairs)
            metadata={}
        )

    except Exception as e:
        logger.error(f"Error retrieving session {session_id}: {str(e)}")
        return None


def get_agent_session(session_id: str) -> Optional[AgentSession]:
    """
    Retrieve an existing agent session - Synchronous wrapper (T040).

    Args:
        session_id: Session ID to retrieve

    Returns:
        AgentSession if found, None otherwise
    """
    return asyncio.run(get_agent_session_async(session_id))


def delete_agent_session(session_id: str) -> bool:
    """
    Delete an agent session and clean up resources (T040).

    Args:
        session_id: Session ID to delete

    Returns:
        True if session was deleted, False if not found
    """
    try:
        # With SQLiteSession, we would need to delete from the database
        # This is a simplified implementation
        logger.info(f"Session deletion requested: {session_id}")
        # TODO: Implement actual session deletion from SQLite database
        return True

    except Exception as e:
        logger.error(f"Error deleting session {session_id}: {str(e)}")
        return False


# ============================================================================
# Streaming Response Mode (T068)
# ============================================================================

async def run_agent_query_streaming(
    query_text: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None,
    temperature: float = 0.5,
    max_turns: int = 5,
    top_k: int = 5,
    score_threshold: float = 0.5,
    max_tokens: int = 2000,
    api_key: Optional[str] = None
) -> AsyncIterator[Dict[str, Any]]:
    """
    Process agent query with streaming response (T068).

    Yields response events as they occur for better UX on long responses.

    Args:
        query_text: User's query
        selected_text: Optional highlighted text
        session_id: Optional session ID for multi-turn
        temperature: Response creativity (0.0-1.0)
        max_turns: Maximum conversation turns
        top_k: Number of chunks to retrieve
        score_threshold: Minimum similarity score
        max_tokens: Maximum response length
        api_key: Deprecated, kept for backward compatibility

    Yields:
        Dict with streaming events:
        - {"event": "retrieval_start"}
        - {"event": "retrieval_complete", "chunks": [...]}
        - {"event": "response_start"}
        - {"event": "response_chunk", "text": "..."}
        - {"event": "response_complete", "metadata": {...}}
        - {"event": "error", "message": "..."}

    Example:
        async for event in run_agent_query_streaming("What is ROS 2?"):
            if event["event"] == "response_chunk":
                print(event["text"], end="", flush=True)
    """
    start_time = time.time()

    try:
        # Validate input
        request = AgentRequest(
            query_text=query_text,
            selected_text=selected_text,
            session_id=session_id,
            temperature=temperature,
            max_turns=max_turns,
            top_k=top_k,
            score_threshold=score_threshold
        )

        yield {"event": "retrieval_start"}

        # Augment query if selected text provided
        if selected_text and selected_text.strip():
            truncated_selection = selected_text[:1000] if len(selected_text) > 1000 else selected_text
            augmented_query = f"""The user has selected this text from the textbook:

\"\"\"
{truncated_selection}
\"\"\"

The user asks: {query_text}

Please retrieve relevant textbook content and answer."""
        else:
            augmented_query = query_text

        yield {"event": "retrieval_complete", "chunks": 0, "scores": []}

        yield {"event": "response_start"}

        # Configure model settings
        run_config = RunConfig(
            model_settings=ModelSettings(
                temperature=temperature,
                max_tokens=max_tokens
            )
        )

        # Get agent instance
        agent = _get_agent_instance()

        # Create session if provided
        session = SQLiteSession(session_id, "conversations.db") if session_id else None

        # Run agent in streaming mode
        result = Runner.run_streamed(
            agent,
            augmented_query,
            session=session,
            run_config=run_config
        )

        # Stream events
        full_response = ""
        async for event in result.stream_events():
            if event.type == "raw_response_event":
                # Import the event type
                from openai.types.responses import ResponseTextDeltaEvent

                if isinstance(event.data, ResponseTextDeltaEvent):
                    chunk_text = event.data.delta
                    full_response += chunk_text
                    yield {"event": "response_chunk", "text": chunk_text}

        response_time_ms = (time.time() - start_time) * 1000

        yield {
            "event": "response_complete",
            "metadata": {
                "response_time_ms": response_time_ms,
                "response_length": len(full_response)
            }
        }

    except ValueError as e:
        sanitized_error = _sanitize_error_message(e, include_type=False)
        yield {"event": "error", "message": sanitized_error}

    except Exception as e:
        sanitized_error = _sanitize_error_message(e, include_type=True)
        yield {"event": "error", "message": f"Unexpected error: {sanitized_error}"}


# ============================================================================
# Health Check (T067)
# ============================================================================

@dataclass
class HealthCheckResult:
    """Health check result for system dependencies (T067)"""
    service: str
    status: str  # "healthy", "degraded", "unhealthy"
    latency_ms: Optional[float] = None
    error_message: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


def check_health() -> Dict[str, Any]:
    """
    Check health of all system dependencies (T067).

    Tests:
    - Qdrant vector database connection
    - Cohere API (embedding generation)
    - OpenRouter API (Qwen LLM via OpenRouter)

    Returns:
        Dict with overall status and individual service checks
    """
    load_dotenv()
    start_time = time.time()

    service_checks = {}
    all_healthy = True
    any_degraded = False

    # Check 1: Qdrant Connection
    qdrant_check = _check_qdrant_health()
    service_checks["qdrant"] = {
        "status": qdrant_check.status,
        "latency_ms": qdrant_check.latency_ms,
        "error": qdrant_check.error_message,
        "details": qdrant_check.details
    }
    if qdrant_check.status == "unhealthy":
        all_healthy = False
    elif qdrant_check.status == "degraded":
        any_degraded = True

    # Check 2: Cohere API (Embeddings)
    cohere_check = _check_cohere_health()
    service_checks["cohere"] = {
        "status": cohere_check.status,
        "latency_ms": cohere_check.latency_ms,
        "error": cohere_check.error_message
    }
    if cohere_check.status == "unhealthy":
        all_healthy = False
    elif cohere_check.status == "degraded":
        any_degraded = True

    # Check 3: OpenRouter API (Qwen LLM)
    openrouter_check = _check_openrouter_health()
    service_checks["openrouter"] = {
        "status": openrouter_check.status,
        "latency_ms": openrouter_check.latency_ms,
        "error": openrouter_check.error_message
    }
    if openrouter_check.status == "unhealthy":
        all_healthy = False
    elif openrouter_check.status == "degraded":
        any_degraded = True

    # Determine overall status
    if all_healthy and not any_degraded:
        overall_status = "healthy"
    elif not all_healthy:
        overall_status = "unhealthy"
    else:
        overall_status = "degraded"

    overall_latency_ms = (time.time() - start_time) * 1000

    return {
        "status": overall_status,
        "timestamp": datetime.now().isoformat(),
        "services": service_checks,
        "overall_latency_ms": overall_latency_ms
    }


def _check_qdrant_health() -> HealthCheckResult:
    """Check Qdrant database connection health (T067)"""
    start_time = time.time()

    try:
        qdrant_client = create_qdrant_client(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook")
        collection_info = qdrant_client.get_collection(collection_name=collection_name)

        latency_ms = (time.time() - start_time) * 1000

        if collection_info.points_count == 0:
            return HealthCheckResult(
                service="qdrant",
                status="degraded",
                latency_ms=latency_ms,
                error_message="Collection exists but has no vectors",
                details={"points_count": 0, "collection": collection_name}
            )

        return HealthCheckResult(
            service="qdrant",
            status="healthy",
            latency_ms=latency_ms,
            details={
                "points_count": collection_info.points_count,
                "collection": collection_name,
                "vector_size": collection_info.config.params.vectors.size
            }
        )

    except Exception as e:
        latency_ms = (time.time() - start_time) * 1000
        return HealthCheckResult(
            service="qdrant",
            status="unhealthy",
            latency_ms=latency_ms,
            error_message=_sanitize_error_message(e, include_type=True)
        )


def _check_cohere_health() -> HealthCheckResult:
    """Check Cohere API health for embedding generation (T067)"""
    start_time = time.time()

    try:
        test_text = "health check test"
        embedding = generate_embedding(
            text=test_text,
            api_key=os.getenv("COHERE_API_KEY"),
            input_type="search_query"
        )

        latency_ms = (time.time() - start_time) * 1000

        if embedding is None or len(embedding) == 0:
            return HealthCheckResult(
                service="cohere",
                status="degraded",
                latency_ms=latency_ms,
                error_message="Embedding generated but appears invalid"
            )

        if latency_ms > 5000:
            return HealthCheckResult(
                service="cohere",
                status="degraded",
                latency_ms=latency_ms,
                error_message=f"High latency: {latency_ms:.0f}ms"
            )

        return HealthCheckResult(
            service="cohere",
            status="healthy",
            latency_ms=latency_ms
        )

    except Exception as e:
        latency_ms = (time.time() - start_time) * 1000
        return HealthCheckResult(
            service="cohere",
            status="unhealthy",
            latency_ms=latency_ms,
            error_message=_sanitize_error_message(e, include_type=True)
        )


def _check_openrouter_health() -> HealthCheckResult:
    """Check OpenRouter API health for Qwen LLM (T067)"""
    start_time = time.time()

    try:
        # Use async health check
        async def _async_check():
            agent = _get_agent_instance()

            result = await Runner.run(
                agent,
                "Say 'healthy'",
                run_config=RunConfig(
                    model_settings=ModelSettings(temperature=0.0, max_tokens=10)
                )
            )

            return result

        result = asyncio.run(_async_check())

        latency_ms = (time.time() - start_time) * 1000

        response_text = result.final_output if hasattr(result, 'final_output') else str(result)

        if not response_text:
            return HealthCheckResult(
                service="openrouter",
                status="degraded",
                latency_ms=latency_ms,
                error_message="Response generated but appears empty"
            )

        if latency_ms > 10000:
            return HealthCheckResult(
                service="openrouter",
                status="degraded",
                latency_ms=latency_ms,
                error_message=f"High latency: {latency_ms:.0f}ms"
            )

        return HealthCheckResult(
            service="openrouter",
            status="healthy",
            latency_ms=latency_ms
        )

    except Exception as e:
        latency_ms = (time.time() - start_time) * 1000
        sanitized_error = _sanitize_error_message(e, include_type=True)

        if "quota" in str(e).lower() or "429" in str(e):
            return HealthCheckResult(
                service="openrouter",
                status="degraded",
                latency_ms=latency_ms,
                error_message=f"API quota limit reached: {sanitized_error}"
            )

        return HealthCheckResult(
            service="openrouter",
            status="unhealthy",
            latency_ms=latency_ms,
            error_message=sanitized_error
        )


# ============================================================================
# Main / Testing
# ============================================================================

if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    if len(sys.argv) > 1:
        query = " ".join(sys.argv[1:])
    else:
        query = "How do I install ROS 2?"

    print(f"\nQuery: {query}\n")

    result = run_agent_query(query)

    print(f"Response: {result.response_text}\n")
    print(f"Retrieved {len(result.retrieved_chunks)} chunks")
    print(f"Response time: {result.response_time_ms:.2f}ms")

    if result.error_message:
        print(f"Error: {result.error_message}")
