---
id: 001
title: RAG Agent Integration Plan
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-rag-agent-integration
branch: 002-rag-agent-integration
user: Asim Hussain
command: /sp.plan
labels: ["plan", "rag", "agent", "openai-sdk", "architecture"]
links:
  spec: specs/002-rag-agent-integration/spec.md
  plan: specs/002-rag-agent-integration/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-agent-integration/plan.md
  - specs/002-rag-agent-integration/research.md
  - specs/002-rag-agent-integration/data-model.md
  - specs/002-rag-agent-integration/quickstart.md
  - specs/002-rag-agent-integration/contracts/agent_contract.md
  - .specify/memory/constitution.md
tests:
  - "Constitution check passed all 7 principles"
  - "Research completed on OpenAI Agents SDK patterns"
  - "Data models defined for all key entities"
  - "API contracts created for agent endpoints"
---

## Prompt

Create: agent architecture overview, workflow diagram for query processing with retrieval integration, agent configuration files, testing scripts, and validation reports.

Decisions needing documentation: choices for OpenAI Agents SDK components (e.g., tool integration for retrieval, agent executor type) with tradeoffs on flexibility vs. simplicity; model selection (use Gemini 2.0 Flash as the LLM via OpenAI-compatible API configuration and configured through the OpenAI Agents SDK using the OpenAI compatibility base URL https://generativelanguage.googleapis.com/v1beta/openai/); handling of selected-text mode in agent prompts.

Testing strategy: unit tests for agent initialization and tool calls; integration tests with at least 5 sample queries, verifying response accuracy, relevance to retrieved chunks, and multi-turn handling; error tests for failed retrievals or invalid inputs; manual review of responses against book content.

Technical details:

- Build upon Spec 1 and 2: Import and reuse ingestion/retrieval functions from main.py and retrieve.py as tools in the agent.

- In backend, create agent.py as the only file for the agent implementation, containing all functionalities: agent setup using OpenAI Agents SDK (following documentation at https://openai.github.io/openai-agents-python), integration of Cohere embedding and Qdrant retrieval as custom tools, query processing logic for full RAG and selected-text modes, and configuration of the Gemini 2.0 Flash model through the OpenAI-compatible base URL.

- Organize by phases: Setup (agent config) → Tool Integration (retrieval) → Agent Execution → Testing → Validation

- Main function: Orchestrate agent runs by initializing the agent, processing sample queries (embed with Cohere, retrieve from Qdrant, augment and respond via the Gemini 2.0 Flash model using the OpenAI-compatible endpoint), handling API keys via environment variables, supporting multi-turn chats, and generating test summaries.

## Response snapshot

Successfully created comprehensive implementation plan for RAG Agent Integration (Spec 002) following the specification workflow:

**Architecture Overview**:
- Agent implementation in single `backend/agent.py` file
- Integration with existing modules: retrieve.py, embedding.py, storage.py
- OpenAI Agents SDK with Gemini 2.0 Flash via OpenAI-compatible API
- Tool-based retrieval integration using @function_tool decorator

**Key Decisions Documented**:
1. OpenAI Agents SDK Architecture: Manager pattern (central agent orchestrates retrieval as tool)
2. Tool Integration: @function_tool wrapper around existing retrieve_content() function
3. Model Selection: Gemini 2.0 Flash via OpenAI-compatible endpoint (https://generativelanguage.googleapis.com/v1beta/openai/)
4. Session Management: Built-in OpenAI Agents SDK sessions with SQLite backend
5. Selected-Text Mode: Prepend selected text to query with source biasing
6. Error Handling: SDK-native error handling with retry logic and fallbacks

**Files Created**:
1. `specs/002-rag-agent-integration/plan.md` - Complete implementation plan with constitution check
2. `specs/002-rag-agent-integration/research.md` - Detailed research on all technical decisions
3. `specs/002-rag-agent-integration/data-model.md` - Complete data models for all entities
4. `specs/002-rag-agent-integration/contracts/agent_contract.md` - API contracts for agent endpoints
5. `specs/002-rag-agent-integration/quickstart.md` - Complete setup and usage guide

**Constitution Check**: All 7 principles validated and passed (Precision, Consistency, Source-Awareness, Modularity, Pedagogical Clarity, Spec-First Execution, Code Quality)

**Project Structure**:
- Single agent file: `backend/agent.py`
- Tests: `backend/tests/test_agent*.py`
- Integration with existing: `retrieve.py`, `embedding.py`, `storage.py`
- Updated: `main.py` for agent orchestration

**Data Models Defined**:
- AgentRequest, AgentResponse, RetrievedChunk, AgentSession, ConversationTurn, AgentConfiguration
- Complete field definitions and validation rules for all entities

**API Contracts**:
- Query endpoint: POST /api/agent/query
- Conversation endpoint: POST /api/agent/conversation
- Session management: GET/DELETE /api/agent/session/{id}
- Health check: GET /api/agent/health

**Quickstart Guide**:
- Setup and environment configuration
- Basic usage examples (single query, multi-turn, selected text)
- Configuration options and testing procedures
- Troubleshooting and error handling

## Outcome

- ✅ Impact: Complete implementation plan ready for development with all architectural decisions documented, data models defined, API contracts specified, and quickstart guide provided. Constitution compliance verified.
- 🧪 Tests: Plan includes comprehensive testing strategy with unit, integration, and validation tests covering all requirements from the feature spec.
- 📁 Files: Created 5 key planning artifacts (plan.md, research.md, data-model.md, quickstart.md, contracts/agent_contract.md) plus updated agent context in CLAUDE.md
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks, or `/sp.implement` to proceed directly to implementation
- 🧠 Reflection: Plan successfully integrates with existing architecture from Specs 1-2, maintains consistency with project constitution, and provides clear path for implementation of all feature requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow completed successfully with all constitution checks passing
- Graders run and results (PASS/FAIL): Constitution check - PASS (all 7 principles validated)
- Prompt variant (if applicable): Standard /sp.plan workflow with research, data modeling, and contract generation phases
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to generate specific implementation tasks based on this plan
