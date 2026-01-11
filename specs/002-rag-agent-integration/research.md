# Research: RAG Agent Integration

## Decision: OpenAI Agents SDK Architecture Pattern

**Rationale**: The OpenAI Agents SDK provides two primary architectural patterns for integrating retrieval: (1) Manager pattern where a central agent orchestrates specialized sub-agents as tools, and (2) Handoff pattern where peer agents delegate to specialized agents. For this RAG implementation, the Manager pattern is optimal as it allows a central agent to invoke the retrieval function as a tool while maintaining control of the conversation flow.

**Alternatives considered**:
- Handoff pattern: Would require separate retrieval agent that takes over conversation, adding complexity without clear benefits for simple RAG queries
- Direct function calls: Would bypass the agent framework's orchestration capabilities and session management
- Custom agent framework: Would require significant development effort and maintenance

## Decision: Tool Integration Approach

**Rationale**: The OpenAI Agents SDK allows custom tools to be created using the `@function_tool` decorator. This approach allows the existing `retrieve_content()` function from retrieve.py to be wrapped as an agent tool, enabling the agent to call it directly. The tool will accept query text and return retrieved chunks with metadata.

**Implementation approach**:
- Create a `RetrievalTool` class that wraps the existing retrieval function
- Use `@function_tool` decorator to make it available to the agent
- The tool will handle the embedding → Qdrant search → result formatting workflow
- This maintains consistency with existing retrieval logic while making it available to the agent

**Alternatives considered**:
- Agent-as-tool pattern: More complex for simple retrieval needs
- External tool API: Would add network overhead and complexity
- Direct Qdrant integration in agent: Would duplicate existing retrieval logic

## Decision: Model Selection - Gemini 2.0 Flash via OpenAI-Compatible API

**Rationale**: Using Gemini 2.0 Flash through the OpenAI-compatible API endpoint (https://generativelanguage.googleapis.com/v1beta/openai/) allows leveraging Google's efficient model while maintaining compatibility with the OpenAI Agents SDK. The OpenAI-compatible interface means the agent framework can work with it without modification.

**Configuration approach**:
- Set base URL to `https://generativelanguage.googleapis.com/v1beta/openai/`
- Use standard OpenAI client configuration with Google API key
- Configure model as `gemini-2.0-flash` or equivalent
- Maintain existing OpenAI Agents SDK patterns without framework changes

**Alternatives considered**:
- Native OpenAI models: Would require different pricing considerations
- Open-source models via Ollama: Would require different API patterns and infrastructure
- Anthropic models: Would require different client libraries and potentially different agent framework

## Decision: Session Management for Multi-Turn Conversations

**Rationale**: The OpenAI Agents SDK provides built-in session management through the `Session` class, which automatically handles conversation history across turns. Using `SQLiteSession` or similar persistent session storage allows the agent to remember context from previous interactions within a conversation.

**Implementation approach**:
- Use the SDK's session capabilities to maintain conversation history
- Each conversation gets a unique session ID
- Session automatically manages context window and history
- Supports both streaming and non-streaming modes

**Alternatives considered**:
- Custom session management: Would require significant development effort
- In-memory sessions: Would not persist across application restarts
- External database: Would add complexity without clear benefits

## Decision: Selected-Text Mode Implementation

**Rationale**: For selected-text mode, the approach will be to prepend the selected text to the user query and potentially bias the retrieval to prioritize chunks from the same source module. This can be achieved by modifying the retrieval parameters or by adding the selected text as additional context in the agent's prompt.

**Implementation approach**:
- When selected text is provided, include it as additional context in the agent's system prompt
- Optionally adjust retrieval to prioritize chunks from the same source as the selected text
- Use the existing retrieval function but with modified parameters for targeted queries
- This maintains the same tool interface while providing enhanced functionality

**Alternatives considered**:
- Separate agent for selected-text mode: Would add complexity without clear benefits
- Pre-filtering by source document: Would require additional logic to identify source relationships
- Direct injection into LLM call: Would bypass the agent framework's orchestration

## Decision: Error Handling Strategy

**Rationale**: The agent must handle various failure modes gracefully including API failures, empty retrieval results, and model unavailability. The OpenAI Agents SDK provides hooks and error handling mechanisms that can be leveraged.

**Implementation approach**:
- Use SDK's built-in error handling for tool failures
- Implement retry logic with exponential backoff for API calls
- Provide fallback responses when retrieval returns no results
- Log errors for debugging and monitoring
- Configure max_turns to prevent infinite loops

**Alternatives considered**:
- Custom error handling framework: Would add complexity
- Hard failures on any error: Would provide poor user experience
- Generic error messages: Would not be helpful for debugging

## Decision: Agent Configuration and Parameters

**Rationale**: The agent needs to be configured with appropriate parameters for the RAG use case including model selection, temperature, max_turns, and system instructions that guide the agent to use the retrieval tool effectively.

**Configuration approach**:
- System instructions that explicitly direct the agent to use retrieval for textbook queries
- Temperature setting appropriate for factual responses (likely 0.3-0.5)
- Max_turns limit to prevent infinite conversations
- Tool configuration that ensures retrieval is available and properly parameterized
- Response formatting that includes citation of source documents

**Alternatives considered**:
- Generic agent configuration: Would not be optimized for RAG
- Multiple specialized agents: Would add complexity without clear benefits for this use case