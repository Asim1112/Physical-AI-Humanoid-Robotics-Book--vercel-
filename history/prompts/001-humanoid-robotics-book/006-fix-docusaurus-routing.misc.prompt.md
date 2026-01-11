---
id: 006
title: Fix Docusaurus Routing Issues
stage: misc
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: 001-humanoid-robotics-book
branch: 001-humanoid-robotics-book
user: Asim Hussain
command: /sp.analyze (used for debugging, not specification analysis)
labels: ["docusaurus", "routing", "debugging", "frontend", "configuration"]
links:
  spec: specs/001-humanoid-robotics-book/spec.md
  ticket: null
  adr: specs/001-humanoid-robotics-book/plan.md#adr-001
  pr: null
files:
  - Frontend/docusaurus.config.js
  - Frontend/src/pages/index.js
  - Frontend/src/pages/index.module.css
tests:
  - Manual browser testing at http://localhost:3000/
  - Landing page loads correctly
  - Get Started button navigates to /docs/ successfully
---

## Prompt

User reported persistent "Page Not Found" errors in Docusaurus development server:

1. Initial error: Homepage showed 404 error
2. After baseUrl fix: Homepage loaded but "Get Started" button still resulted in 404
3. Request: Analyze the project, use Context7 MCP to read Docusaurus docs, and fix routing issues
4. Request: Update /sp.constitution, /sp.specify, /sp.plan to reflect fixes

Full prompt sequence:
- "the same error is still occurs on the browser... use context7 mcp server and follow this link 'https://docusaurus.io/docs' to read the docusaurus documentation and then implement it. also analyze the whole project and find out why the error is not being solved"
- "the front page is shown... but once i click on 'Get Started' button, it shows the same error... analyze the /sp.constitution, /sp.specify, /sp.tasks and find out the problem, once problem findout update the files and fix the problem"
- "the problem is fixed and solved. the project is perfectly running on the browser, now please update the /sp.constitution, /sp.specify, /sp.plan if there is need to update them regarding the current fixes that you have done"

## Response snapshot

**Root Cause Analysis:**

Three interconnected routing issues identified:

1. **BaseURL misconfiguration**: `docusaurus.config.js` had `baseUrl: '/humanoid-robotics-book/'` (GitHub Pages config) instead of `baseUrl: '/'` for local development
   - Fixed at: `Frontend/docusaurus.config.js:19`

2. **Poor landing page UX**: `Frontend/src/pages/index.js` used `<Redirect to="/docs/intro" />` which:
   - Created immediate redirect with no visible landing page
   - Pointed to non-existent `/docs/intro` path (should be `/intro` or `/docs/`)
   - Violated Docusaurus best practices for custom landing pages

3. **Slug routing misunderstanding**: `intro.mdx` has `slug: /` making it accessible at `/docs/` (root of docs), not `/intro`
   - "Get Started" button incorrectly linked to `/intro`
   - Fixed to link to `/docs/` correctly

**Implementation:**

1. Changed `baseUrl` to `'/'` with comment for GitHub Pages deployment
2. Replaced redirect with proper React landing page component featuring:
   - Hero banner with site title and tagline
   - "Get Started" button
   - Welcome content section
3. Created `index.module.css` for landing page styling
4. Fixed button link from `/intro` to `/docs/`

**Documentation Updates:**

Updated `specs/001-humanoid-robotics-book/plan.md` with four ADRs:
- ADR-001: Docusaurus BaseURL Configuration for Local Development
- ADR-002: Landing Page Component vs. Redirect
- ADR-003: Docs Routing with Slug Configuration
- ADR-004: Frontend Component Structure

**Constitution Compliance:**

No constitution updates needed - fixes align with existing principles:
- Principle III (Source-Aware Content): Would have benefited from Context7 MCP usage initially
- Principle VI (Spec-First Execution): Documented decisions retroactively in plan.md
- Structural Rules: All changes confined to `/Frontend` directory

## Outcome

- ✅ Impact: Project now runs perfectly on localhost:3000 with correct routing throughout
- 🧪 Tests: Manual browser testing confirmed - landing page loads, navigation works, no 404 errors
- 📁 Files: Modified 1 config file, 2 component files (1 new)
- 🔁 Next prompts: Continue module content development, add diagrams and code examples
- 🧠 Reflection: Initial Context7 MCP consultation would have prevented these routing issues. The constitution's emphasis on source-aware implementation via Context7 is validated by this debugging session.

## Evaluation notes (flywheel)

- Failure modes observed:
  - Assumed incorrect path without verifying Docusaurus routing behavior
  - Did not initially consult Context7 MCP for Docusaurus documentation
  - Misunderstood slug configuration and how it interacts with baseUrl

- Graders run and results (PASS/FAIL):
  - ✅ PASS: Landing page renders correctly
  - ✅ PASS: Get Started button navigates to intro page
  - ✅ PASS: All navigation links functional
  - ✅ PASS: Constitution compliance maintained
  - ✅ PASS: Documentation updated with ADRs

- Prompt variant (if applicable): Standard debugging sequence with iterative fixes

- Next experiment (smallest change to try):
  - Implement environment-based baseUrl configuration (NODE_ENV check)
  - Add Context7 MCP consultation step to checklist for all Docusaurus configuration changes
  - Consider adding automated routing tests to prevent regression
