# Feature Specification: RAG Agent Integration

**Feature Branch**: `002-rag-agent-integration`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Build an AI agent with retrieval integration for RAG chatbot in Physical AI & Humanoid Robotics textbook. Target audience: Hackathon participants and evaluators. Focus on developing an agent using OpenAI Agents SDK that incorporates retrieval from Qdrant vector database, leveraging Cohere embeddings for query processing and context augmentation."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query Textbook with Agent Responses (Priority: P1) 🎯 MVP

Hackathon participants and evaluators can ask questions about the humanoid robotics textbook and receive accurate, context-based answers from an AI agent that retrieves relevant content from the vector database.

**Why this priority**: This is the core functionality that delivers immediate value - the ability to query the textbook content through an intelligent agent. Without this, the feature has no purpose.

**Independent Test**: Can be fully tested by submitting a query like "How do I install ROS 2?" and verifying that the agent retrieves relevant chunks from Qdrant, augments its prompt with that context, and generates an accurate response that references the textbook content.

**Acceptance Scenarios**:

1. **Given** the agent is running and connected to Qdrant, **When** a user submits "What is Gazebo simulation?", **Then** the agent retrieves 3-5 relevant chunks with similarity scores >0.5, augments its prompt with the retrieved context, and responds with information directly from the textbook modules
2. **Given** a query has been processed, **When** the user reviews the response, **Then** the response contains accurate information matching the textbook content and cites which modules/sections the information came from
3. **Given** the vector database contains 1,278 chunks, **When** a user submits any robotics-related query, **Then** the agent completes the full cycle (embedding → retrieval → augmentation → generation) in under 5 seconds
4. **Given** the agent has existing retrieval functions from Spec 1-2, **When** the agent processes a query, **Then** it reuses the existing `retrieve_content()` function without duplicating retrieval logic

---

### User Story 2 - Multi-Turn Conversations with Context (Priority: P2)

Users can have multi-turn conversations with the agent where it remembers previous questions and answers, allowing for follow-up questions and clarifications about the textbook content.

**Why this priority**: Multi-turn conversation capability significantly improves the user experience by allowing natural dialogue, but the agent can still provide value with single-turn queries in P1.

**Independent Test**: Submit a first query "What is ROS 2?", then a follow-up "How do I install it?" without re-stating the context. Verify the agent understands "it" refers to ROS 2 from the previous turn and retrieves installation-specific content.

**Acceptance Scenarios**:

1. **Given** a user has asked "What are VLA models?", **When** they follow up with "Give me an example", **Then** the agent retrieves examples of VLA models from the textbook and responds with context from both turns
2. **Given** a conversation session exists, **When** the user asks a follow-up question, **Then** the agent maintains conversation history across turns without requiring the user to repeat context
3. **Given** a multi-turn conversation, **When** reviewing the conversation history, **Then** each turn includes the query, retrieved chunks, and agent response in a structured format
4. **Given** a conversation reaches 10+ turns, **When** the next query is processed, **Then** the agent maintains relevant context while staying within token limits

---

### User Story 3 - Selected Text Mode for Targeted Queries (Priority: P3)

Users can highlight specific text from the textbook interface and ask questions specifically about that selection, with the agent prioritizing the selected context in its retrieval and response.

**Why this priority**: This enhances precision for specific questions about particular textbook sections, but core query functionality (P1-P2) delivers value without it.

**Independent Test**: Pass a query "Explain this" along with selected text from Module 1 about ROS 2 nodes. Verify the agent retrieves additional context related to the selected text and generates a response that directly addresses the selection.

**Acceptance Scenarios**:

1. **Given** a user has selected a paragraph about Gazebo physics simulation, **When** they ask "How does this work?", **Then** the agent retrieves chunks semantically related to the selected text and responds with explanations specific to that topic
2. **Given** selected text from Module 3, **When** the agent processes the query, **Then** the response prioritizes information from Module 3 and related sections over unrelated modules
3. **Given** a query with selected text, **When** no directly related chunks are found in Qdrant, **Then** the agent still provides a response based on the selected text itself and general knowledge

---

### Edge Cases

- What happens when a user query returns zero results from Qdrant (similarity scores all <0.5)?
- How does the agent handle queries in languages other than English?
- What happens when the Qdrant vector database is unavailable or connection fails?
- How does the agent respond when a query is completely outside the textbook domain (e.g., "What's the weather today?")?
- What happens when embeddings generation fails due to Cohere API errors?
- How does the agent handle very long queries (>1000 characters) that exceed embedding limits?
- What happens when max_turns is exceeded in a complex multi-turn conversation?
- How does the agent maintain performance when conversation history grows very large?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST create an agent that can process user queries about the humanoid robotics textbook
- **FR-002**: System MUST integrate retrieval as a tool in the agent's workflow, enabling the agent to fetch relevant chunks from the vector database before generating responses
- **FR-003**: System MUST reuse existing embedding and retrieval functions from previous specifications without duplicating retrieval logic
- **FR-004**: System MUST augment the agent's prompt with retrieved context before response generation, ensuring answers are grounded in textbook content
- **FR-005**: System MUST support multi-turn conversations where the agent remembers previous queries and responses within a session
- **FR-006**: System MUST handle conversation history management using session capabilities
- **FR-007**: System MUST support optional selected-text mode where users can provide highlighted textbook text for more targeted responses
- **FR-008**: System MUST configure the agent with error resilience for API failures (embeddings, retrieval, generation)
- **FR-009**: System MUST return agent responses in a structured format suitable for backend API integration, including the query, retrieved chunks metadata, response text, and conversation ID
- **FR-010**: System MUST validate agent responses with at least 5 diverse test queries covering different textbook modules (ROS2, Gazebo, VLA, humanoid robots, general robotics concepts)
- **FR-011**: System MUST log all agent interactions including input queries, retrieval results, tool calls, and final outputs for debugging and evaluation
- **FR-012**: System MUST configure max_turns limit to prevent infinite loops in agent execution
- **FR-013**: System MUST handle cases where retrieval returns no results by providing a fallback response based on general knowledge
- **FR-014**: System MUST support both streaming and non-streaming response modes for different integration scenarios
- **FR-015**: System MUST maintain compatibility with the existing backend structure from previous specifications and prepare for API integration

### Key Entities

- **Agent**: The AI agent instance configured with instructions, model settings, and retrieval tool; represents the core conversational entity that processes queries
- **RetrievalTool**: A tool wrapped around the existing retrieval function that the agent can invoke to fetch relevant textbook chunks from the vector database
- **AgentSession**: Manages conversation history and context across multiple turns, storing previous queries, retrieved chunks, and responses
- **AgentRequest**: Input structure containing user query text, optional selected text, optional session ID, and configuration parameters
- **AgentResponse**: Output structure containing the agent's response text, list of retrieved chunks with metadata (source file, similarity score, module name), conversation ID, and execution metrics (latency, tokens used)
- **ConversationTurn**: A single query-response cycle within a session, including the input, retrieved context, tool calls made, and final output
- **AgentConfiguration**: Settings for the agent including model selection, temperature, max_turns, retrieval parameters (top_k, score_threshold), and error handling policies

### Technical Constraints

- **TC-001**: System MUST use OpenAI Agents SDK (openai-agents) for agent implementation, NOT direct LLM API calls
- **TC-002**: System MUST use Llama 3.3 70B Versatile model (`llama-3.3-70b-versatile`) as the LLM backend
- **TC-003**: System MUST use Groq as the LLM provider with base URL: `https://api.groq.com/openai/v1`
- **TC-004**: System MUST use OpenAIChatCompletionsModel with AsyncOpenAI client for model configuration
- **TC-005**: System MUST implement function tools using the `@function_tool` decorator pattern
- **TC-006**: System MUST use async/await patterns with `await Runner.run()` for agent execution
- **TC-007**: System MUST use SQLiteSession for conversation history management
- **TC-008**: System MUST reuse existing `retrieve_content()` function from previous specifications
- **TC-009**: System MUST use Cohere embeddings API for query embedding generation (consistent with existing retrieval system)
- **TC-010**: System MUST connect to existing Qdrant vector database with collection `humanoid-robotics-textbook`

### LLM Provider Configuration

**Model**: Llama 3.3 70B Versatile (`llama-3.3-70b-versatile`)
**Provider**: Groq
**Base URL**: `https://api.groq.com/openai/v1`
**Integration Method**: OpenAIChatCompletionsModel with AsyncOpenAI client

**Configuration Example**:
```python
from agents import Agent, OpenAIChatCompletionsModel, set_tracing_disabled
from openai import AsyncOpenAI

# Disable tracing since we're not using OpenAI API key
set_tracing_disabled(True)

# Create Groq client
groq_client = AsyncOpenAI(
    api_key=os.getenv("GROQ_API_KEY"),
    base_url="https://api.groq.com/openai/v1"
)

agent = Agent(
    name="Physical AI & Humanoid Robotics Assistant",
    instructions="...",
    model=OpenAIChatCompletionsModel(
        model="llama-3.3-70b-versatile",
        openai_client=groq_client
    ),
    tools=[retrieve_textbook_content]
)
```

**Note**: Groq provides OpenAI-compatible API endpoints, allowing seamless integration with the OpenAI Agents SDK through the `AsyncOpenAI` client configured with Groq's base URL.

**Rationale**: Groq was selected over the originally planned OpenRouter/Qwen setup due to superior function calling support. Llama 3.3 70B Versatile ranks #1 on the Berkeley Function Calling Leaderboard and provides excellent performance for retrieval-augmented generation tasks. Groq offers 14,400 free requests/day with ultra-fast inference (300+ tokens/second), making it ideal for development and hackathon demonstration. The model's strong technical comprehension and reliable tool-calling capabilities ensure accurate responses grounded in textbook content.

### Environment Variables

The following environment variables MUST be configured:

- `GROQ_API_KEY`: API key for Groq service (LLM provider)
- `COHERE_API_KEY`: API key for Cohere embeddings generation
- `QDRANT_URL`: URL of Qdrant vector database (default: `http://localhost:6333`)
- `QDRANT_API_KEY`: API key for Qdrant Cloud authentication
- `QDRANT_COLLECTION_NAME`: Name of the vector collection (default: `humanoid-robotics-textbook`)

**Security**: All API keys MUST be stored in `.env` file and NEVER committed to version control. The `.env` file MUST be listed in `.gitignore`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive relevant, accurate answers to textbook queries in under 5 seconds for 95% of single-turn interactions
- **SC-002**: Agent successfully retrieves and incorporates textbook content for at least 5 diverse test queries, with 100% query processing success rate
- **SC-003**: Multi-turn conversations maintain context accurately across at least 5 consecutive turns without losing relevant information
- **SC-004**: Agent responses correctly cite or reference the source textbook modules/sections for at least 90% of factual claims
- **SC-005**: System handles at least 3 different error scenarios gracefully (database unavailable, API failure, no retrieval results) without crashing
- **SC-006**: Hackathon evaluators can successfully test the agent and verify it answers questions about all major textbook modules (ROS2, Gazebo, VLA, humanoid robots)
- **SC-007**: Agent maintains consistent response quality across 10+ consecutive queries without degradation
- **SC-008**: Selected-text mode improves response relevance by prioritizing context from the selected section in at least 80% of cases where selected text is provided
