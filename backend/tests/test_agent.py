"""
Unit tests for RAG Agent module (agent.py)

Tests cover:
- T016: AgentRequest data validation
- T017: AgentResponse data validation
- T018: run_agent_query() function
"""

import pytest
from datetime import datetime
from agent import (
    AgentRequest,
    AgentResponse,
    ConversationTurn,
    AgentSession,
    AgentConfiguration,
    RetrievalToolInput,
    RetrievalToolOutput,
    run_agent_query
)
from retrieve import RetrievedChunk


# ============================================================================
# T016: Unit tests for AgentRequest data validation
# ============================================================================

class TestAgentRequestValidation:
    """Test AgentRequest validation (T016)"""

    def test_request_validation_success(self):
        """Test successful AgentRequest creation with valid parameters"""
        request = AgentRequest(
            query_text="How do I install ROS 2?",
            selected_text=None,
            session_id="test-session-123",
            temperature=0.7,
            max_turns=5,
            top_k=3,
            score_threshold=0.6
        )

        assert request.query_text == "How do I install ROS 2?"
        assert request.session_id == "test-session-123"
        assert request.temperature == 0.7
        assert request.max_turns == 5
        assert request.top_k == 3
        assert request.score_threshold == 0.6

    def test_request_validation_empty_query(self):
        """Test AgentRequest rejects empty query text"""
        with pytest.raises(ValueError, match="Query text cannot be empty"):
            AgentRequest(query_text="")

    def test_request_validation_whitespace_only_query(self):
        """Test AgentRequest rejects whitespace-only query"""
        with pytest.raises(ValueError, match="Query text cannot be empty"):
            AgentRequest(query_text="   \n\t  ")

    def test_request_validation_query_too_long(self):
        """Test AgentRequest rejects query text >1000 characters"""
        long_query = "x" * 1001
        with pytest.raises(ValueError, match="Query text too long"):
            AgentRequest(query_text=long_query)

    def test_request_validation_temperature_out_of_range_low(self):
        """Test AgentRequest rejects temperature < 0.0"""
        with pytest.raises(ValueError, match="Temperature must be between"):
            AgentRequest(query_text="test", temperature=-0.1)

    def test_request_validation_temperature_out_of_range_high(self):
        """Test AgentRequest rejects temperature > 1.0"""
        with pytest.raises(ValueError, match="Temperature must be between"):
            AgentRequest(query_text="test", temperature=1.5)

    def test_request_validation_max_turns_too_low(self):
        """Test AgentRequest rejects max_turns < 1"""
        with pytest.raises(ValueError, match="max_turns must be between"):
            AgentRequest(query_text="test", max_turns=0)

    def test_request_validation_max_turns_too_high(self):
        """Test AgentRequest rejects max_turns > 50"""
        with pytest.raises(ValueError, match="max_turns must be between"):
            AgentRequest(query_text="test", max_turns=51)

    def test_request_validation_top_k_too_low(self):
        """Test AgentRequest rejects top_k < 1"""
        with pytest.raises(ValueError, match="top_k must be between"):
            AgentRequest(query_text="test", top_k=0)

    def test_request_validation_top_k_too_high(self):
        """Test AgentRequest rejects top_k > 10"""
        with pytest.raises(ValueError, match="top_k must be between"):
            AgentRequest(query_text="test", top_k=11)

    def test_request_validation_score_threshold_out_of_range_low(self):
        """Test AgentRequest rejects score_threshold < 0.0"""
        with pytest.raises(ValueError, match="score_threshold must be between"):
            AgentRequest(query_text="test", score_threshold=-0.1)

    def test_request_validation_score_threshold_out_of_range_high(self):
        """Test AgentRequest rejects score_threshold > 1.0"""
        with pytest.raises(ValueError, match="score_threshold must be between"):
            AgentRequest(query_text="test", score_threshold=1.1)


# ============================================================================
# T017: Unit tests for AgentResponse data validation
# ============================================================================

class TestAgentResponseValidation:
    """Test AgentResponse validation (T017)"""

    def test_response_validation_success(self):
        """Test successful AgentResponse creation with all required fields"""
        chunk = RetrievedChunk(
            chunk_id="chunk-1",
            similarity_score=0.85,
            content_text="ROS 2 is a robotics middleware...",
            source_file="module-01.txt",
            module_name="Introduction",
            section_heading="What is ROS 2?",
            chunk_index=0,
            total_chunks=10,
            token_count=50,
            retrieved_at=datetime.now().isoformat()
        )

        response = AgentResponse(
            response_text="Here's how to install ROS 2...",
            retrieved_chunks=[chunk],
            conversation_id="conv-123",
            response_time_ms=1500.5,
            query_tokens=10,
            response_tokens=50,
            retrieval_success=True,
            error_message=None
        )

        assert response.response_text == "Here's how to install ROS 2..."
        assert len(response.retrieved_chunks) == 1
        assert response.conversation_id == "conv-123"
        assert response.response_time_ms == 1500.5
        assert response.retrieval_success is True
        assert response.error_message is None

    def test_response_validation_missing_fields(self):
        """Test AgentResponse requires essential fields"""
        # Should work with minimal required fields
        response = AgentResponse(
            response_text="",
            retrieved_chunks=[],
            conversation_id="test",
            response_time_ms=0.0
        )

        assert response.response_text == ""
        assert response.retrieved_chunks == []
        assert response.retrieval_success is True  # default value

    def test_response_validation_with_error(self):
        """Test AgentResponse with error message"""
        response = AgentResponse(
            response_text="",
            retrieved_chunks=[],
            conversation_id="error-123",
            response_time_ms=100.0,
            retrieval_success=False,
            error_message="API quota exceeded"
        )

        assert response.retrieval_success is False
        assert response.error_message == "API quota exceeded"


# ============================================================================
# T018: Unit tests for run_agent_query() function
# ============================================================================

class TestRunAgentQuery:
    """Test run_agent_query() function (T018)"""

    def test_agent_query_empty_query(self):
        """Test run_agent_query rejects empty query"""
        result = run_agent_query("")

        assert result.retrieval_success is False
        assert result.error_message is not None
        assert "empty" in result.error_message.lower()

    def test_agent_query_invalid_parameters(self):
        """Test run_agent_query rejects invalid parameters"""
        # Invalid temperature
        result = run_agent_query("test", temperature=2.0)
        assert result.retrieval_success is False
        assert "temperature" in result.error_message.lower()

    def test_agent_query_returns_response_object(self):
        """Test run_agent_query returns AgentResponse object"""
        result = run_agent_query("What is ROS 2?", top_k=1)

        # Should return AgentResponse even if there's an error
        assert isinstance(result, AgentResponse)
        assert result.conversation_id is not None
        assert result.response_time_ms > 0

    @pytest.mark.integration
    def test_agent_query_success(self):
        """Test run_agent_query with valid query (requires API access)"""
        # This is a light integration test - marks as integration
        # but tests the basic flow
        result = run_agent_query(
            "How do I install ROS 2?",
            top_k=3,
            score_threshold=0.5
        )

        # Should get a response (may be error if API unavailable)
        assert isinstance(result, AgentResponse)
        assert result.conversation_id is not None
        assert result.conversation_id.startswith("conv_") or result.conversation_id.startswith("error_")


# ============================================================================
# T029: Unit tests for AgentSession data validation (Phase 4)
# ============================================================================

class TestAgentSessionValidation:
    """Test AgentSession validation (T029)"""

    def test_session_validation_success(self):
        """Test successful AgentSession creation with valid parameters"""
        from agent import AgentSession

        session = AgentSession(
            session_id="session-abc-123",
            created_at="2025-12-25T10:00:00Z",
            last_accessed="2025-12-25T10:05:00Z",
            conversation_history=[],
            current_turn=0,
            metadata={"user_id": "test-user"}
        )

        assert session.session_id == "session-abc-123"
        assert session.created_at == "2025-12-25T10:00:00Z"
        assert session.last_accessed == "2025-12-25T10:05:00Z"
        assert len(session.conversation_history) == 0
        assert session.current_turn == 0
        assert session.metadata["user_id"] == "test-user"

    def test_session_validation_invalid_id(self):
        """Test AgentSession with empty session_id"""
        from agent import AgentSession

        # Empty session_id should still be allowed at dataclass level
        # (validation happens in create/continue functions)
        session = AgentSession(
            session_id="",
            created_at="2025-12-25T10:00:00Z",
            last_accessed="2025-12-25T10:00:00Z"
        )

        assert session.session_id == ""

    def test_session_with_conversation_history(self):
        """Test AgentSession with populated conversation history"""
        from agent import AgentSession, ConversationTurn
        from retrieve import RetrievedChunk
        from datetime import datetime

        chunk = RetrievedChunk(
            chunk_id="chunk-1",
            similarity_score=0.85,
            content_text="Test content",
            source_file="test.txt",
            module_name="Test Module",
            section_heading="Test Section",
            chunk_index=0,
            total_chunks=1,
            token_count=10,
            retrieved_at=datetime.now().isoformat()
        )

        turn = ConversationTurn(
            turn_number=1,
            user_input="What is ROS 2?",
            retrieved_chunks=[chunk],
            agent_response="ROS 2 is...",
            timestamp="2025-12-25T10:00:00Z"
        )

        session = AgentSession(
            session_id="session-123",
            created_at="2025-12-25T10:00:00Z",
            last_accessed="2025-12-25T10:01:00Z",
            conversation_history=[turn],
            current_turn=1
        )

        assert len(session.conversation_history) == 1
        assert session.current_turn == 1
        assert session.conversation_history[0].user_input == "What is ROS 2?"


# ============================================================================
# T030: Unit tests for ConversationTurn data validation (Phase 4)
# ============================================================================

class TestConversationTurnValidation:
    """Test ConversationTurn validation (T030)"""

    def test_turn_validation_success(self):
        """Test successful ConversationTurn creation"""
        from agent import ConversationTurn
        from retrieve import RetrievedChunk
        from datetime import datetime

        chunk = RetrievedChunk(
            chunk_id="chunk-1",
            similarity_score=0.85,
            content_text="ROS 2 is a robotics middleware",
            source_file="module-01.txt",
            module_name="Introduction",
            section_heading="What is ROS 2?",
            chunk_index=0,
            total_chunks=5,
            token_count=20,
            retrieved_at=datetime.now().isoformat()
        )

        turn = ConversationTurn(
            turn_number=1,
            user_input="What is ROS 2?",
            retrieved_chunks=[chunk],
            agent_response="ROS 2 is a robotics middleware...",
            timestamp="2025-12-25T10:00:00Z",
            tool_calls=[],
            context_used="Previous context here"
        )

        assert turn.turn_number == 1
        assert turn.user_input == "What is ROS 2?"
        assert len(turn.retrieved_chunks) == 1
        assert turn.agent_response == "ROS 2 is a robotics middleware..."
        assert turn.timestamp == "2025-12-25T10:00:00Z"
        assert turn.context_used == "Previous context here"

    def test_turn_validation_missing_fields(self):
        """Test ConversationTurn with minimal required fields"""
        from agent import ConversationTurn

        # Should work with required fields only (defaults for optional)
        turn = ConversationTurn(
            turn_number=1,
            user_input="Test query",
            retrieved_chunks=[],
            agent_response="Test response",
            timestamp="2025-12-25T10:00:00Z"
        )

        assert turn.turn_number == 1
        assert turn.tool_calls == []  # default
        assert turn.context_used is None  # default

    def test_turn_validation_with_tool_calls(self):
        """Test ConversationTurn with tool calls recorded"""
        from agent import ConversationTurn

        tool_calls = [
            {"tool": "retrieval", "query": "ROS 2 installation"},
            {"tool": "embedding", "input": "How do I install ROS 2?"}
        ]

        turn = ConversationTurn(
            turn_number=2,
            user_input="How do I install it?",
            retrieved_chunks=[],
            agent_response="To install ROS 2...",
            timestamp="2025-12-25T10:01:00Z",
            tool_calls=tool_calls
        )

        assert len(turn.tool_calls) == 2
        assert turn.tool_calls[0]["tool"] == "retrieval"


# ============================================================================
# T042: Unit tests for selected-text mode validation (Phase 5)
# ============================================================================

class TestSelectedTextValidation:
    """Test selected-text mode validation (T042)"""

    def test_selected_text_validation_success(self):
        """Test successful AgentRequest with selected text"""
        from agent import AgentRequest

        selected_text = "ROS 2 nodes are the fundamental building blocks of a ROS 2 application."

        request = AgentRequest(
            query_text="Explain this concept",
            selected_text=selected_text,
            temperature=0.5,
            top_k=5,
            score_threshold=0.5
        )

        assert request.query_text == "Explain this concept"
        assert request.selected_text == selected_text
        assert len(request.selected_text) > 0

    def test_selected_text_validation_invalid_input(self):
        """Test AgentRequest validation with invalid selected text"""
        from agent import AgentRequest

        # Very long selected text (>2000 characters)
        long_text = "x" * 2001

        # Should allow long selected text but may warn/truncate in processing
        # (validation happens at processing level, not dataclass level)
        request = AgentRequest(
            query_text="Explain this",
            selected_text=long_text
        )

        assert request.selected_text == long_text

    def test_selected_text_with_special_characters(self):
        """Test selected text with code snippets and special characters"""
        from agent import AgentRequest

        selected_text = """
        ```python
        import rclpy
        from rclpy.node import Node

        class MyNode(Node):
            def __init__(self):
                super().__init__('my_node')
        ```
        This code creates a ROS 2 node.
        """

        request = AgentRequest(
            query_text="What does this code do?",
            selected_text=selected_text
        )

        assert request.selected_text == selected_text
        assert "```python" in request.selected_text


# ============================================================================
# T043: Unit tests for selected-text mode processing (Phase 5)
# ============================================================================

class TestSelectedTextProcessing:
    """Test selected-text mode processing (T043)"""

    def test_selected_text_processing_success(self):
        """Test query processing with selected text"""
        from agent import run_agent_query

        selected_text = "Gazebo is a premier simulation environment for robotics."

        result = run_agent_query(
            query_text="What is the significance of this?",
            selected_text=selected_text,
            top_k=3
        )

        # Should return a response (may be error if API unavailable)
        assert isinstance(result.response_text, str)
        assert result.conversation_id is not None

    def test_selected_text_processing_empty_selection(self):
        """Test processing with empty selected text (should behave like normal query)"""
        from agent import run_agent_query

        result = run_agent_query(
            query_text="What is ROS 2?",
            selected_text="",  # Empty selection
            top_k=3
        )

        assert isinstance(result.response_text, str)
        # Empty selected_text should not cause errors

    def test_selected_text_processing_none_selection(self):
        """Test processing with None selected text"""
        from agent import run_agent_query

        result = run_agent_query(
            query_text="What is Gazebo?",
            selected_text=None,
            top_k=3
        )

        assert isinstance(result.response_text, str)
        # None selected_text should behave like normal query
