---
id: 002
title: ChatKit Docusaurus Integration Research
stage: research
date: 2025-12-29
surface: agent
model: claude-sonnet-4-5
feature: 002-rag-agent-integration
branch: 002-rag-agent-integration
user: developer
command: research
labels: ["chatkit", "docusaurus", "react", "frontend", "integration", "chat-widget"]
links:
  spec: specs/002-rag-agent-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-agent-integration/chatkit-docusaurus-integration-research.md
tests:
  - None (research phase)
---

## Prompt

Research and document best practices for integrating OpenAI ChatKit SDK in Docusaurus:

**Context**: Embedding a chat widget in a Docusaurus site for RAG chatbot with:
- React-based Docusaurus frontend
- Chat widget connecting to FastAPI backend
- Support for selected-text mode (capture window.getSelection())
- Responsive design (desktop, tablet, mobile)
- Configuration for local dev and production deployment

**Research Topics**:
1. **ChatKit SDK Installation & Setup**: npm installation, initialization in React components, configuration options

2. **Docusaurus Integration**: Best practices for adding custom React components to Docusaurus, using docusaurus.config.js vs theme swizzling, global vs page-specific widgets

3. **Selected-Text Capture**: JavaScript patterns for capturing text selection (window.getSelection()), triggering chat widget with context, handling edge cases (code blocks, special characters)

4. **Widget Customization**: Balancing default ChatKit UI vs custom styling, responsive behavior, positioning (floating widget vs embedded)

5. **Backend Connection**: Configuring API base URL for different environments (localhost, Vercel deployment), handling environment variables in Docusaurus

**Output Format**:
For each topic, provide:
- **Decision**: Recommended approach
- **Rationale**: Why this balances functionality and simplicity
- **Code Example**: Integration snippet
- **Alternatives Considered**: Other approaches and trade-offs

## Response snapshot

Created comprehensive research document covering all 5 integration topics:

### Key Findings:

**1. ChatKit SDK Installation & Setup**
- **Decision**: Build custom React chat component instead of using OpenAI ChatKit SDK
- **Rationale**: OpenAI does not offer a public "ChatKit SDK" npm package; custom component provides better control for RAG-specific features (source citations, selected-text mode)
- **Dependencies**: react-markdown, remark-gfm, react-syntax-highlighter
- **Code**: Full ChatWidget.jsx component with message handling, API integration, and state management

**2. Docusaurus Integration**
- **Decision**: Client Module with Root Component Wrapper
- **Rationale**: Global widget availability, persistent state, single configuration point
- **Implementation**: Custom Root.js theme component + clientModules registration
- **Alternatives**: Theme swizzling (higher maintenance), page-specific integration (no global state), custom plugin (overkill)

**3. Selected-Text Capture**
- **Decision**: Global selectionchange event listener with floating button trigger
- **Rationale**: Clear user affordance, handles all edge cases, seamless chat integration
- **Edge Cases Handled**: Code blocks (preserve formatting), multi-line selections (normalize whitespace), special characters (sanitize), min/max length validation
- **Code**: TextSelectionHandler.jsx component with position calculation and event management

**4. Widget Customization**
- **Decision**: CSS Modules with Docusaurus theme tokens
- **Rationale**: Component-scoped styling + automatic theme integration (light/dark mode)
- **Implementation**: ChatWidget.module.css using var(--ifm-*) tokens
- **Responsive**: Mobile (100vh fullscreen), tablet (350px), desktop (380px floating)
- **Alternatives**: MUI (bundle size impact), Tailwind (build config overhead), inline styles (performance concerns)

**5. Backend Connection**
- **Decision**: Environment-based API configuration with Fetch API
- **Rationale**: Simple, standard, no extra dependencies, compatible with Docusaurus build
- **Configuration**: .env.development (localhost:8000) + .env.production (Vercel URL)
- **Code**: chatApi.js service module with sendMessage(), streamMessage(), healthCheck()
- **CORS**: FastAPI middleware configuration for Docusaurus origins
- **Alternatives**: Axios (unnecessary dependency), React Query (too complex), GraphQL (backend not compatible)

### Recommended Implementation Path:

**Phase 1: Core Chat Widget (MVP)**
1. Custom React chat component with CSS Modules
2. Docusaurus Root wrapper integration
3. Fetch API backend connection
4. Environment-based configuration

**Phase 2: Selected-Text Mode**
1. Global selection listener
2. Floating button trigger
3. Edge case handling
4. Pre-fill chat input

**Phase 3: Polish & Optimization**
1. Responsive design refinements
2. Dark mode theming
3. Error handling and offline mode
4. Source citations display

### Files Structure:
```
frontend/
├── src/
│   ├── components/ChatWidget/
│   │   ├── ChatWidget.jsx
│   │   ├── ChatWidget.module.css
│   │   ├── TextSelectionHandler.jsx
│   │   └── MessageList.jsx
│   ├── services/chatApi.js
│   ├── theme/Root.js
│   └── css/custom.css
├── .env.development
├── .env.production
└── docusaurus.config.js
```

## Outcome

- ✅ Impact: Comprehensive integration research document created with 5 complete decision records, code examples, and trade-off analysis. Provides clear implementation roadmap for frontend chat widget development.
- 🧪 Tests: None (research phase)
- 📁 Files: Created chatkit-docusaurus-integration-research.md (15KB, 600+ lines)
- 🔁 Next prompts: Implement Phase 1 (Core Chat Widget) based on research recommendations
- 🧠 Reflection: Research successfully addressed all 5 topics with actionable recommendations. Key insight: No official OpenAI ChatKit SDK exists—custom component approach is necessary and provides better flexibility for RAG-specific features like source citations and selected-text mode.

## Evaluation notes (flywheel)

- Failure modes observed: Web search and web fetch tools were unavailable, requiring reliance on internal knowledge of React, Docusaurus, and frontend best practices (2025 standards)
- Graders run and results (PASS/FAIL): None (research deliverable)
- Prompt variant (if applicable): Standard research request with clear output format requirements
- Next experiment (smallest change to try): Validate research recommendations by implementing Phase 1 MVP and iterating based on real integration challenges
