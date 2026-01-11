<!--
SYNC IMPACT REPORT - Constitution Update
========================================
Version: 2.1.0 → 2.1.1
Date: 2025-12-31

CHANGES (2.1.1):
- Corrected directory references: `/Frontend` → `frontend/` throughout document
- Updated Structural & Architectural Rules section (6 occurrences)
- Updated Book & Frontend Standards section (3 occurrences)
- This is a PATCH update correcting documentation to match actual implementation
- No functional changes to principles or rules

PREVIOUS CHANGES (2.1.0):
- Modified Principles:
  - Enhanced Principle III (Source-Aware Content) to explicitly include Context7 MCP as authoritative source
  - Enhanced Principle VI (Spec-First Execution) to include source-grounded implementation requirement
- Added Sections:
  - Documentation & Source Authority (Context7 MCP) - NEW SECTION
  - Explicit guidance on when to consult Context7 for Docusaurus decisions
  - Pause-and-clarify requirements when documentation is ambiguous
- Modified Sections:
  - Book & Frontend Standards - Enhanced with Docusaurus convention requirements
  - Writing & Content Standards - Enhanced with "no hallucinated technical facts" requirement
- Removed Sections: None

TEMPLATE UPDATES:
- ✅ plan-template.md - Constitution Check section remains flexible for dynamic rule extraction
- ✅ spec-template.md - User story structure aligns with modularity and testability requirements
- ✅ tasks-template.md - Phase-based structure supports incremental delivery
- ⚠️ Command files - Will validate .specify/templates/commands/*.md if they exist

VERSION BUMP RATIONALE (2.1.1):
- PATCH (2.1.0 → 2.1.1): Directory reference correction to align constitution with actual
  project structure. This is a documentation fix with no impact on principles, rules, or
  workflows. All references to `/Frontend` corrected to `frontend/` to match implementation.

FOLLOW-UP TODOs: None
-->

# Humanoid Robotics Book Constitution

## Core Principles

### I. Precision and Depth
All technical content MUST be accurate, verifiable, and comprehensive. Target audience is developers and AI learners requiring deep understanding, not surface-level overviews. Every concept must be explained with sufficient depth to enable practical implementation. No hallucinated technical facts; rely strictly on official model, framework, and tool documentation.

**Rationale**: The book's value derives from its technical accuracy. Readers invest time expecting authoritative, implementable knowledge. Shallow or incorrect content destroys trust and wastes reader effort.

### II. Consistency Across Artifacts
All chapters, modules, code examples, and terminology MUST maintain consistency. Use standardized terminology (AI, Python, Agents SDK, ROS 2, etc.) throughout. Standard chapter structure MUST be followed: Overview → Deep Explanation → Practical Examples → Summary/Key Takeaways.

**Rationale**: Consistency reduces cognitive load, enables readers to build mental models progressively, and ensures the book functions as a cohesive learning system rather than disconnected articles.

### III. Source-Aware Content
Content MUST be derived from credible technical documentation, official references, and current AI model specifications. All claims must be traceable to authoritative sources. When referencing external systems, APIs, or frameworks, cite official documentation. **Context7 MCP server is the authoritative documentation source for Docusaurus** - consult it before making any Docusaurus implementation decisions.

**Rationale**: Technical accuracy requires grounding in authoritative sources. Source awareness prevents drift, enables updates when specifications change, and builds reader confidence through verifiable claims. Context7 MCP provides access to official Docusaurus documentation, ensuring all UI and configuration decisions are based on current, verified information rather than model recall.

### IV. Modularity
Every chapter MUST be structured as a reusable, standalone document. Readers should be able to read any chapter independently and gain value, while the overall progression from beginner to advanced remains coherent. Chapters can reference earlier concepts but must not depend on them for basic comprehension.

**Rationale**: Modular design accommodates different learning paths, enables selective reading, and simplifies maintenance. Readers with partial background can jump to relevant sections without reading linearly.

### V. Pedagogical Clarity
Concepts MUST be explained progressively from beginner to advanced. Each chapter builds foundational understanding before introducing complexity. Use clear, accessible language with technical precision. Writing style: friendly, expressive, and educational (aligned with Asim's preferred tone).

**Rationale**: Progressive complexity prevents cognitive overload and ensures accessibility. The book serves both newcomers and experienced practitioners through layered explanation that starts simple and deepens incrementally.

### VI. Spec-First Execution
No implementation or file creation is allowed unless explicitly defined in specifications and plans. All work MUST follow the Spec-Driven Development workflow: specification → planning → task generation → implementation. No improvisation or deviation from approved specs. **Source-grounded implementation**: No UI, configuration, or structural decisions may be made from model memory alone - all decisions must be grounded in official documentation accessed via Context7 MCP.

**Rationale**: Spec-first discipline ensures alignment with project goals, prevents scope creep, enables review before implementation, and maintains traceability between requirements and delivered artifacts. Source-grounded implementation prevents hallucinated configurations and ensures all technical decisions are based on current, authoritative documentation.

### VII. Code Quality and Runnability
Code examples MUST be runnable, logically correct, and tested before inclusion. Each code snippet must serve a clear pedagogical purpose. Include 2-4 code examples per chapter. Examples must demonstrate concepts in realistic contexts, not contrived scenarios.

**Rationale**: Runnable code is the gold standard for technical books. Non-functional examples frustrate readers and undermine credibility. Tested examples ensure readers can reproduce results and build confidence.

## Structural & Architectural Rules (Non-Negotiable)

The project MUST follow clear separation of concerns:

- All book-related frontend assets MUST live inside `frontend/` directory
- `frontend/` is the ONLY location where Docusaurus is initialized and maintained
- NO Docusaurus files, configs, or documentation content may exist at repository root
- Backend components (RAG services, APIs, databases, embeddings pipelines) MUST remain outside `frontend/`
- Repository root is reserved for:
  - Spec-Kit artifacts (`/sp.constitution`, `/sp.specify`, `/sp.plan`, `/sp.tasks`)
  - Backend services
  - Shared configuration and tooling

**Rationale**: Clear boundaries prevent file conflicts, enable independent frontend/backend evolution, simplify deployment pipelines, and maintain architectural clarity. This structure supports future expansion (e.g., multiple frontends, backend services) without restructuring.

## Book & Frontend Standards

- **Book format**: Docusaurus (Markdown + MDX)
- **Frontend framework**: Docusaurus MUST be initialized only within `frontend/`
- **Chapter content**: MUST reside within the Docusaurus `docs/` structure under `frontend/`
- **Docusaurus conventions**: Sidebar, navbar, routing, and internal cross-linking MUST follow official Docusaurus conventions (consult Context7 MCP before implementation)
- **Frontend buildability**: The frontend MUST be buildable and deployable independently of backend services

**Rationale**: Docusaurus provides excellent developer documentation experience. Confining it to `frontend/` maintains architectural separation and enables CI/CD pipelines to build/test frontend independently. Following official conventions ensures compatibility with Docusaurus updates and community best practices.

## Documentation & Source Authority (Context7 MCP)

**Context7 MCP server is the authoritative documentation source for Docusaurus.** Before implementing any of the following, Claude Code MUST consult Context7-provided Docusaurus documentation:

- Docusaurus initialization
- `docusaurus.config.*` configuration
- Sidebar configuration (`sidebars.js` or `sidebars.ts`)
- Navbar configuration
- MDX features and syntax
- Plugin and theme usage
- Versioning and docs structure

**Official Docusaurus documentation** (https://docusaurus.io/docs) accessed via Context7 MCP **takes precedence over model recall**.

**When documentation is missing, ambiguous, or conflicting**:
- Implementation MUST pause
- Clarification MUST be requested before proceeding
- No assumptions, shortcuts, or undocumented behaviors are allowed in UI or configuration decisions

**Rationale**: Context7 MCP provides access to official, current Docusaurus documentation. This ensures all implementation decisions are grounded in authoritative sources rather than potentially outdated model training data. The pause-and-clarify requirement prevents hallucinated configurations that could break the build or violate Docusaurus best practices.

## Writing & Content Standards

All technical definitions MUST be accurate and verifiable. No hallucinated technical facts; rely strictly on official model, framework, and tool documentation. Writing style MUST be friendly, expressive, and educational (aligned with Asim's preferred tone).

**Standard chapter structure** (mandatory for all chapters):
1. Overview
2. Deep Explanation
3. Practical Examples
4. Summary / Key Takeaways

**Code examples**:
- Runnable and logically correct
- Tested before inclusion
- 2-4 examples per chapter minimum
- Each example serves clear pedagogical purpose

**Terminology**: MUST remain consistent across the entire book (AI, Python, Agents SDK, ROS 2, etc.). Define terms on first use and maintain definitions throughout.

**Rationale**: Consistent structure enables readers to navigate predictably. Friendly tone maintains engagement. Code quality ensures practical value. Terminology consistency prevents confusion and supports learning. The "no hallucinated facts" requirement reinforces Principle I and ensures technical accuracy.

## Content Scope Constraints

- **Book length**: 8–12 full chapters
- **Each chapter MUST include**:
  - At least 1 diagram (text-described for later rendering)
  - 2–4 code examples
  - A clear takeaway section
- **Chapters MUST be modular**: Readable in isolation while fitting into overall progression from beginner to advanced

**Rationale**: Scope constraints prevent endless expansion, maintain focus, and ensure deliverability. Diagrams enhance understanding of complex concepts. Code examples ground theory in practice. Takeaways reinforce learning.

## Deployment & Quality Gates

- **Deployment target**: GitHub Pages
- **CI/CD pipeline**: MUST pass with zero build errors
- **Docusaurus site**: MUST build with no warnings or broken links
- **Navigation**: Sidebar and internal references MUST function correctly
- **Compliance**: Claude Code sub-agents MUST follow this constitution as a binding quality contract

**Quality gates** (all must pass before deployment):
1. Docusaurus build completes successfully (exit code 0)
2. No broken internal links
3. All code examples tested and verified runnable
4. Sidebar navigation complete and logical
5. Technical accuracy review completed
6. Terminology consistency validated

**Rationale**: Zero-defect deployment standard ensures professional quality. Broken links and build errors undermine credibility. Quality gates prevent rework and maintain user trust.

## Success Criteria

A fully functional Docusaurus site inside `/Frontend`, built and deployed to GitHub Pages with:

1. **Completeness**: 8–12 chapters covering the defined scope
2. **Quality**: All chapters follow standard structure with diagrams, code examples, and takeaways
3. **Accuracy**: All technical content verified against authoritative sources
4. **Functionality**: Site builds cleanly, navigation works, links resolve
5. **Pedagogy**: Progressive learning path from beginner to advanced
6. **Modularity**: Each chapter readable independently
7. **Deployment**: Accessible via GitHub Pages with CI/CD automation

**Rationale**: Success criteria provide objective evaluation standards and align team/agent efforts toward measurable outcomes.

## Governance

This constitution supersedes all other practices. All agents, developers, and contributors MUST verify compliance before submitting work. Complexity beyond these principles MUST be explicitly justified in specifications and approved.

**Amendment Process**:
1. Proposed amendments documented with rationale and impact analysis
2. Template dependencies identified and migration plan prepared
3. Approval obtained before implementation
4. Version incremented according to semantic versioning:
   - **MAJOR**: Backward-incompatible governance/principle removals or redefinitions
   - **MINOR**: New principle/section added or materially expanded guidance
   - **PATCH**: Clarifications, wording, typo fixes, non-semantic refinements
5. All dependent templates and docs updated
6. Constitution amendment recorded in PHR and/or ADR as appropriate

**Compliance Review**:
- All PRs/reviews MUST verify constitution compliance
- Quality gates enforce adherence automatically where possible
- Manual reviews verify spec-first execution and principle adherence
- Constitution violations require explicit justification and approval

**Runtime Guidance**:
- See `CLAUDE.md` for agent-specific development guidance
- See `.specify/templates/` for artifact structure templates
- See `README.md` for project overview and quickstart

**Version**: 2.1.1 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-31
