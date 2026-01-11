# Feature Specification: Frontend-Backend Integration for RAG Chatbot

**Feature Branch**: `003-frontend-backend-integration`
**Created**: 2025-12-29
**Status**: Draft
**Input**: User description: "Integrate backend with frontend for RAG chatbot in Physical AI & Humanoid Robotics textbook. Target audience: Hackathon participants and evaluators for the textbook project. Focus: Establish FastAPI backend endpoints to serve the OpenAI agent with retrieval, integrate Neon Serverless Postgres for session management, and embed the chatbot UI in the Docusaurus frontend using OpenAI ChatKit SDK, enabling local and deployed connections for querying the book's content including selected-text mode"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query Textbook via Chat Interface (Priority: P1) 🎯 MVP

Hackathon participants and evaluators can interact with the RAG agent through an embedded chat widget in the Docusaurus frontend, asking questions about the textbook and receiving accurate, context-based answers.

**Why this priority**: This is the core user-facing functionality that delivers immediate value - the ability to query the textbook through a chat interface. Without this, the feature has no purpose.

**Independent Test**: Can be fully tested by opening the Docusaurus site, typing a question in the chat widget (e.g., "How do I install ROS 2?"), and verifying that the widget sends the query to the backend API, receives a response from the agent, and displays it in the chat interface.

**Acceptance Scenarios**:

1. **Given** the Docusaurus frontend is loaded with the chat widget visible, **When** a user types "What is Gazebo simulation?" and presses send, **Then** the message appears in the chat history, the backend processes the query through the RAG agent, and the response displays in the chat widget within 5 seconds
2. **Given** a user has submitted a query, **When** the agent retrieves relevant chunks and generates a response, **Then** the response is formatted correctly in the chat widget with proper markdown rendering and source citations
3. **Given** the chat widget is open, **When** a user submits multiple queries in succession, **Then** each query-response pair appears in chronological order in the chat history
4. **Given** the backend is unavailable or returns an error, **When** a user submits a query, **Then** the chat widget displays a user-friendly error message and allows retry

---

### User Story 2 - Session Persistence Across Page Navigation (Priority: P2)

Users can navigate between different pages of the textbook while maintaining their conversation history with the chatbot, allowing them to reference previous questions and answers.

**Why this priority**: Session persistence significantly improves the user experience by maintaining context across navigation, but the chat can still provide value without it (single-page conversations).

**Independent Test**: Open the chat widget on one textbook page, ask a question about ROS 2, navigate to a different page (e.g., Gazebo module), verify the chat history persists, and ask a follow-up question referencing the previous conversation.

**Acceptance Scenarios**:

1. **Given** a user has asked 3 questions on the homepage, **When** they navigate to the Module 1 page, **Then** the chat widget opens with the full conversation history visible
2. **Given** a user has an active session, **When** they close the browser and return within 24 hours, **Then** their conversation history is restored from the database
3. **Given** a user asks "What is ROS 2?" and then navigates to another page, **When** they ask "How do I install it?", **Then** the agent understands "it" refers to ROS 2 from the previous turn
4. **Given** a session exists in the database, **When** the user explicitly clears the chat history, **Then** a new session is created and the old conversation is archived

---

### User Story 3 - Selected-Text Mode for Contextual Queries (Priority: P3)

Users can highlight specific text from the textbook and ask questions directly about that selection, with the chat widget automatically including the selected text as context for more precise answers.

**Why this priority**: This enhances precision for specific questions about particular textbook sections, but core chat functionality (P1-P2) delivers value without it.

**Independent Test**: Highlight a paragraph about Gazebo physics simulation, right-click or use a keyboard shortcut to trigger the chat widget, type "Explain this in simpler terms", and verify the agent receives both the query and the selected text, generating a response specific to that content.

**Acceptance Scenarios**:

1. **Given** a user has selected a paragraph about ROS 2 nodes, **When** they open the chat widget and ask "What does this mean?", **Then** the backend receives the query with the selected text, and the agent prioritizes that content in its response
2. **Given** a user has selected text from Module 2, **When** they submit a query, **Then** the chat widget displays a visual indicator showing the selected text was included as context
3. **Given** a user has selected text and asked a question, **When** they clear the selection and ask another question, **Then** the new query is processed without the previous selection
4. **Given** a user selects text that is too long (>2000 characters), **When** they try to query it, **Then** the chat widget shows a warning and uses only the first 2000 characters as context

---

### User Story 4 - Local Development and Production Deployment (Priority: P1) 🎯 MVP

Developers can run the full stack locally during development (frontend connecting to localhost:8000 backend) and deploy to production with the frontend on Vercel connecting to a hosted backend.

**Why this priority**: Essential for development workflow and production deployment. Without this, the feature cannot be developed or deployed.

**Independent Test**: Run the backend locally with `uvicorn` or similar command, start the frontend dev server, verify the chat widget connects to localhost, then deploy the frontend to Vercel with the backend URL configured as an environment variable and verify the chat works in production.

**Acceptance Scenarios**:

1. **Given** the backend is running locally, **When** the frontend dev server starts, **Then** the chat widget successfully connects to the local API and queries work
2. **Given** the frontend is deployed on Vercel with backend URL environment variable set, **When** a user opens the deployed site, **Then** the chat widget connects to the production backend URL
3. **Given** the backend has CORS configured, **When** the frontend makes API requests from a different origin, **Then** requests are allowed and responses are returned correctly
4. **Given** environment variables are configured incorrectly, **When** the chat widget tries to connect, **Then** a clear error message displays indicating the connection issue

---

### Edge Cases

- What happens when the backend API is down or unreachable?
- How does the chat widget handle network timeouts during long-running queries?
- What happens if the database connection fails during session creation?
- How does the system handle concurrent requests from multiple users?
- What happens if the user's browser blocks localStorage/sessionStorage for session management?
- How does the selected-text mode handle special characters, code blocks, or non-English text?
- What happens when a session ID becomes invalid or expires?
- How does the chat widget behave on mobile devices with limited screen space?
- What happens if the ChatKit SDK fails to load due to network issues or CDN unavailability?
- How does the system handle rate limiting from the LLM API when multiple users query simultaneously?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST expose a REST API endpoint for chat queries that accepts user messages and returns agent responses
- **FR-002**: System MUST expose a REST API endpoint for streaming responses from the agent
- **FR-003**: System MUST expose REST API endpoints for session management: create session, retrieve session, delete session
- **FR-004**: Backend MUST integrate with database to store chat sessions including user queries, agent responses, timestamps, and session metadata
- **FR-005**: System MUST support selected-text mode where the frontend can send both a query and selected textbook content, and the backend prioritizes that content in retrieval
- **FR-006**: Frontend MUST embed a chat widget component in the Docusaurus site
- **FR-007**: Chat widget MUST connect to the backend API with configurable base URL (localhost for development, deployed URL for production)
- **FR-008**: Chat widget MUST display conversation history with proper formatting for markdown, code blocks, and citations
- **FR-009**: Chat widget MUST handle loading states, error states, and empty states with user-friendly messages
- **FR-010**: System MUST implement CORS configuration with whitelisted origins to enable secure cross-origin requests between frontend and backend
- **FR-011**: Backend MUST validate all incoming requests (query text, selected text, session IDs) and return appropriate error codes
- **FR-012**: System SHOULD implement API key authentication in future versions (deferred post-MVP; hackathon demo operates without authentication per plan.md)
- **FR-013**: Frontend MUST provide a mechanism for users to trigger selected-text mode
- **FR-014**: System MUST log all API requests, responses, and errors for debugging and monitoring purposes
- **FR-015**: Chat widget MUST be responsive and work on desktop, tablet, and mobile devices
- **FR-016**: System MUST implement health check endpoints for monitoring
- **FR-017**: Backend MUST reuse the existing agent and retrieval logic from previous specifications
- **FR-018**: Chat widget MUST persist user preferences in browser storage
- **FR-019**: System MUST handle session expiration and automatic session renewal
- **FR-020**: Frontend MUST include error boundaries to prevent chat widget errors from crashing the entire site
- **FR-021**: System MUST implement retry logic with exponential backoff (up to 3 retries) for backend connectivity failures and network timeouts
- **FR-022**: System MUST handle LLM API rate limits with user-friendly error messages and retry suggestions
- **FR-023**: Chat widget MUST provide fallback UI when ChatKit SDK fails to load due to network issues or CDN unavailability
- **FR-024**: System MUST sanitize selected text to prevent XSS attacks and handle special characters, code blocks, and non-English text correctly
- **FR-025**: System MUST implement graceful degradation when browser localStorage is unavailable, maintaining session in memory for the current page lifecycle

### Key Entities

- **ChatSession**: Represents a conversation between a user and the agent, stored in the database with attributes: session identifier (unique ID), created timestamp, last accessed timestamp, user identifier (optional for future authentication), metadata (additional context)
- **ChatMessage**: Represents a single message in a conversation, with attributes: message identifier (unique ID), session identifier (foreign reference), role (user or assistant), content (message text), selected text (optional context), timestamp, response time measurement, error message (if applicable)
- **APIRequest**: Input structure for the chat endpoint, with attributes: query (text, required), session identifier (optional), selected text (optional context), stream flag (boolean)
- **APIResponse**: Output structure from the chat endpoint, with attributes: response text, session identifier, message identifier, retrieved chunks metadata, response time measurement, error details (if applicable)
- **ChatWidget**: Frontend component that manages the chat UI, state, and API communication, with properties: open state, messages array, loading state, error state, session identifier

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can submit queries through the chat widget and receive responses in under 5 seconds for 95% of interactions
- **SC-002**: Chat sessions persist across page navigation and browser sessions for at least 24 hours
- **SC-003**: At least 5 diverse test queries successfully complete end-to-end (user types in widget → backend processes → response displays) without errors
- **SC-004**: Selected-text mode correctly sends highlighted content to the backend and receives contextually relevant responses for 90% of tested selections
- **SC-005**: The chat widget loads and initializes within 2 seconds on the Docusaurus site
- **SC-006**: The system handles at least 50 concurrent users without significant performance degradation (response time remains under 10 seconds)
- **SC-007**: Deployment to production completes successfully with chat widget connecting to the hosted backend
- **SC-008**: Error scenarios (backend down, network timeout, invalid session) display user-friendly error messages in the chat widget
- **SC-009**: The chat widget is fully functional on mobile devices with screen widths down to 375px
- **SC-010**: Hackathon evaluators can successfully query all major textbook modules through the chat interface and verify accurate responses
