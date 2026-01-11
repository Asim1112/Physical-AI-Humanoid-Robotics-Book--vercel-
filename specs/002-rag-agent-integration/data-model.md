# Data Model: RAG Agent Integration

## AgentRequest

**Description**: Input structure for agent queries, containing user query text and optional parameters.

**Fields**:
- `query_text` (str): The user's query text to be processed by the agent
- `selected_text` (str, optional): Optional selected text from the textbook interface for targeted queries
- `session_id` (str, optional): Session identifier for maintaining conversation history across turns
- `temperature` (float, optional): Temperature setting for response generation (default: 0.5)
- `max_turns` (int, optional): Maximum number of conversation turns (default: 10)
- `top_k` (int, optional): Number of chunks to retrieve (default: 5)
- `score_threshold` (float, optional): Minimum similarity score for retrieved chunks (default: 0.5)

**Validation Rules**:
- `query_text` must be non-empty and <1000 characters
- `temperature` must be between 0.0 and 1.0
- `max_turns` must be between 1 and 50
- `top_k` must be between 1 and 10
- `score_threshold` must be between 0.0 and 1.0

## AgentResponse

**Description**: Output structure containing the agent's response and associated metadata.

**Fields**:
- `response_text` (str): The agent's generated response to the user query
- `retrieved_chunks` (list[RetrievedChunk]): List of chunks retrieved during the query processing
- `conversation_id` (str): Identifier for the conversation thread
- `response_time_ms` (float): Time taken to generate the response in milliseconds
- `query_tokens` (int): Number of tokens in the input query
- `response_tokens` (int): Number of tokens in the agent response
- `retrieval_success` (bool): Whether the retrieval operation was successful
- `error_message` (str, optional): Error message if any occurred during processing

## RetrievedChunk (from retrieve.py)

**Description**: Data structure representing a single chunk retrieved from the vector database.

**Fields**:
- `chunk_id` (str): Unique identifier for the chunk
- `similarity_score` (float): Similarity score of the chunk to the query (0.0-1.0)
- `content_text` (str): The actual text content of the chunk
- `source_file` (str): Name of the source file where the chunk originated
- `module_name` (str): Name of the textbook module containing the chunk
- `section_heading` (str): Section heading where the chunk appears
- `chunk_index` (int): Position of the chunk within the original document
- `total_chunks` (int): Total number of chunks in the original document
- `token_count` (int): Number of tokens in the chunk
- `retrieved_at` (str): Timestamp when the chunk was retrieved

## AgentSession

**Description**: Structure for managing conversation state across multiple turns.

**Fields**:
- `session_id` (str): Unique identifier for the conversation session
- `created_at` (str): Timestamp when the session was created
- `last_accessed` (str): Timestamp of the last interaction
- `conversation_history` (list[ConversationTurn]): History of all turns in the conversation
- `current_turn` (int): Current turn number in the conversation
- `metadata` (dict): Additional session metadata

## ConversationTurn

**Description**: Represents a single turn in a multi-turn conversation.

**Fields**:
- `turn_number` (int): Sequential number of this turn in the conversation
- `user_input` (str): The user's input for this turn
- `retrieved_chunks` (list[RetrievedChunk]): Chunks retrieved for this turn
- `agent_response` (str): The agent's response for this turn
- `timestamp` (str): When this turn occurred
- `tool_calls` (list): List of tools called during this turn
- `context_used` (str, optional): Additional context used for this turn (e.g., selected text)

## AgentConfiguration

**Description**: Configuration settings for the agent behavior.

**Fields**:
- `model_name` (str): Name of the LLM model to use (e.g., "gemini-2.0-flash")
- `temperature` (float): Temperature setting for response generation
- `max_tokens` (int): Maximum tokens for response generation
- `system_prompt` (str): System instructions for the agent
- `retrieval_enabled` (bool): Whether retrieval tool is enabled
- `max_turns` (int): Maximum number of conversation turns
- `score_threshold` (float): Minimum similarity score for retrieved chunks
- `top_k` (int): Number of chunks to retrieve per query

## RetrievalToolInput

**Description**: Input structure for the retrieval tool called by the agent.

**Fields**:
- `query_text` (str): Query text to search for in the vector database
- `top_k` (int, optional): Number of results to return (default: 5)
- `score_threshold` (float, optional): Minimum similarity score (default: 0.5)
- `selected_text` (str, optional): Additional context from selected text

## RetrievalToolOutput

**Description**: Output structure from the retrieval tool.

**Fields**:
- `retrieved_chunks` (list[RetrievedChunk]): List of retrieved chunks
- `query_embedding` (list[float]): The embedding vector of the query
- `search_time_ms` (float): Time taken for the retrieval operation
- `total_candidates` (int): Total number of candidates considered
- `success` (bool): Whether the retrieval operation was successful
- `error_message` (str, optional): Error message if retrieval failed