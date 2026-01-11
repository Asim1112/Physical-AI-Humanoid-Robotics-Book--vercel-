---
id: 002
title: Context7 MCP Documentation Authority
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 005-module4-vla
user: asimh
command: /sp.constitution
labels: ["constitution", "context7", "mcp", "docusaurus", "source-authority"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - N/A
---

## Prompt

Project: AI/Spec-Driven Technical Book Creation using Docusaurus, Claude Code, and Spec-Kit Plus

Core Principles: Precision and depth, consistency across chapters/modules/code, source-aware content, modularity, pedagogical clarity (beginner → advanced), spec-first execution, source-grounded implementation (no UI/config decisions from model memory alone).

Structural & Architectural Rules (Non-Negotiable): Clear separation of concerns, all book frontend in /Frontend directory, Docusaurus only in /Frontend, no Docusaurus at repository root, backend outside /Frontend, repository root for Spec-Kit artifacts/backend/shared config.

Book & Frontend Standards: Docusaurus (Markdown + MDX), Docusaurus only within /Frontend, chapter content in /Frontend/docs/, sidebar/navbar/routing follow official Docusaurus conventions, frontend buildable independently.

**Documentation & Source Authority (Context7 MCP)**: Context7 MCP server is the authoritative documentation source for Docusaurus. Before implementing any of the following, Claude Code must consult Context7-provided Docusaurus documentation: Docusaurus initialization, docusaurus.config.*, Sidebar configuration, Navbar configuration, MDX features and syntax, Plugin and theme usage, Versioning and docs structure. Official Docusaurus documentation (https://docusaurus.io/docs) accessed via Context7 MCP takes precedence over model recall. If documentation is missing, ambiguous, or conflicting: Implementation must pause, Clarification must be requested before proceeding, No assumptions, shortcuts, or undocumented behaviors are allowed in UI or configuration decisions.

Writing & Content Standards: All technical definitions accurate and verifiable, no hallucinated technical facts, runnable code examples tested before inclusion, friendly/expressive/educational style, standard chapter structure (Overview → Deep Explanation → Practical Examples → Summary/Key Takeaways), consistent terminology.

Content Scope Constraints: 8-12 chapters, each with at least 1 diagram, 2-4 code examples, clear takeaway section, modular and readable in isolation.

Deployment & Quality Gates: GitHub Pages, CI/CD with zero build errors, Docusaurus builds with no warnings/broken links, navigation functions correctly, Claude Code sub-agents must follow constitution.

## Response snapshot

Updated constitution from v2.0.0 → v2.1.0 with Context7 MCP documentation authority requirements.

**Key Changes**:
- **Enhanced Principle III (Source-Aware Content)**: Explicitly added Context7 MCP server as authoritative documentation source for Docusaurus
- **Enhanced Principle VI (Spec-First Execution)**: Added source-grounded implementation requirement - no UI, configuration, or structural decisions may be made from model memory alone
- **New Section**: Documentation & Source Authority (Context7 MCP) with comprehensive guidance on:
  - When to consult Context7 (initialization, config, sidebar, navbar, MDX, plugins, versioning)
  - Official Docusaurus documentation precedence over model recall
  - Pause-and-clarify requirements when documentation is missing, ambiguous, or conflicting
  - No assumptions, shortcuts, or undocumented behaviors allowed
- **Enhanced Book & Frontend Standards**: Added explicit requirement to consult Context7 MCP before Docusaurus implementation
- **Enhanced Writing & Content Standards**: Reinforced "no hallucinated technical facts" requirement

**Version Bump Rationale**: MINOR (2.0.0 → 2.1.0) - New section added (Documentation & Source Authority) with material expansion of source-awareness guidance. Adds new operational requirements (Context7 consultation, pause-on-ambiguity) without breaking existing workflows.

**Template Consistency**: Validated plan-template.md, spec-template.md, tasks-template.md - all remain flexible and compatible with updated constitution. No command files exist yet, so no updates needed.

## Outcome

- ✅ Impact: Constitution v2.1.0 establishes Context7 MCP as authoritative Docusaurus documentation source
- 🧪 Tests: Template consistency validated (plan/spec/tasks templates remain compatible)
- 📁 Files: 1 file updated (constitution.md)
- 🔁 Next: Apply constitution requirements when implementing Docusaurus in /Frontend
- 🧠 Reflection: Context7 MCP integration critical for preventing hallucinated Docusaurus configurations. Pause-and-clarify requirement ensures all UI and configuration decisions are grounded in official documentation rather than potentially outdated model training data. This amendment materially strengthens Principles III and VI by providing concrete mechanisms for source-grounded implementation.

## Evaluation notes (flywheel)

- Failure modes: None
- Graders: Constitution consistency check PASS (templates validated)
- Variant: Standard /sp.constitution amendment workflow
- Next: Monitor Context7 MCP consultation patterns during /Frontend Docusaurus implementation to validate effectiveness of pause-and-clarify requirements
