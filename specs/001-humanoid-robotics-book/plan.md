# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `001-humanoid-robotics-book` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-humanoid-robotics-book/spec.md`

## Summary

Create a comprehensive, AI-native educational textbook for teaching Physical AI and Humanoid Robotics using Docusaurus. The book consists of 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) plus a Capstone project, deployed to GitHub Pages with strict `/Frontend` isolation. All UI and configuration decisions must be grounded in official Docusaurus documentation via Context7 MCP server.

## Technical Context

**Language/Version**: Markdown/MDX, Python 3.10+ (for code examples)
**Primary Dependencies**: Docusaurus 3.x, Node.js 18+, React (via Docusaurus), ROS 2 Humble, Gazebo 11/Fortress, NVIDIA Isaac Sim 2023.1+
**Storage**: Git repository, static site generation (no database)
**Testing**: Docusaurus build validation, link checking, code example execution tests
**Target Platform**: Web (GitHub Pages), development on Ubuntu 22.04 / WSL2
**Project Type**: Documentation site (Docusaurus static site)
**Performance Goals**: Build time <5 minutes, page load <2 seconds, zero broken links
**Constraints**: All Docusaurus content in `/Frontend` only, no backend services, Context7 MCP required for all Docusaurus decisions
**Scale/Scope**: 8-12 chapters across 4 modules + Capstone, 1,500-3,000 words per chapter, 60-80 hours total reader time

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Precision and Depth
- ✅ **Compliance**: All technical content will be verified against official documentation (ROS 2, NVIDIA Isaac, Gazebo, Docusaurus)
- ✅ **Enforcement**: Minimum 50% of technical claims must reference authoritative sources (FR-004, SC-013)
- ✅ **Testing**: Technical accuracy review required before deployment (SC-009)

### Principle II: Consistency Across Artifacts
- ✅ **Compliance**: Standard chapter structure enforced: Overview → Deep Explanation → Practical Examples → Summary
- ✅ **Enforcement**: Terminology standardization across all modules (FR-007)
- ✅ **Testing**: Consistency validation in Phase 3

### Principle III: Source-Aware Content (Context7 MCP)
- ✅ **Compliance**: Context7 MCP server is authoritative source for all Docusaurus decisions
- ✅ **Enforcement**: Must consult Context7 before: initialization, config, sidebar, navbar, MDX, plugins, versioning
- ✅ **Testing**: All Docusaurus configurations must be traceable to official documentation
- ⚠️  **Action Required**: Pause and request clarification if documentation is missing, ambiguous, or conflicting

### Principle IV: Modularity
- ✅ **Compliance**: Each module independently readable with clear prerequisites
- ✅ **Enforcement**: Module isolation design, independent testability (User Stories P1-P4)
- ✅ **Testing**: Each module can be completed independently (SC-001 through SC-005)

### Principle V: Pedagogical Clarity
- ✅ **Compliance**: Progressive complexity from beginner to advanced
- ✅ **Enforcement**: 1,500-3,000 words per chapter, exercises at end of sections
- ✅ **Testing**: 85% reader rating for logical progression (SC-010)

### Principle VI: Spec-First Execution & Source-Grounded Implementation
- ✅ **Compliance**: No implementation without approved specifications and plans
- ✅ **Enforcement**: No UI/config decisions from model memory - all via Context7 MCP
- ✅ **Testing**: This plan document required before content creation begins

### Principle VII: Code Quality and Runnability
- ✅ **Compliance**: All code examples tested before inclusion
- ✅ **Enforcement**: 95% code execution success rate (SC-006), 2-4 examples per chapter
- ✅ **Testing**: Clean environment testing required for all examples

### Structural & Architectural Rules
- ✅ **Compliance**: All book content in `/Frontend` directory only
- ✅ **Enforcement**: No Docusaurus files at repository root, frontend buildable independently
- ✅ **Testing**: Build verification ensures `/Frontend` isolation

**Constitution Gate**: ✅ PASSED - All principles addressable through planned implementation

## Project Structure

### Documentation (this feature)

```text
specs/001-humanoid-robotics-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (Docusaurus structure, best practices)
├── data-model.md        # Phase 1 output (Content taxonomy, navigation model)
├── quickstart.md        # Phase 1 output (Author workflow, build process)
├── contracts/           # Phase 1 output (Module interfaces, chapter templates)
│   ├── module-structure.md
│   ├── chapter-template.md
│   └── code-example-format.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (Frontend Content)

```text
Frontend/
├── docs/
│   ├── intro.mdx                      # Landing page / Book introduction
│   ├── lab-setup/                     # Environment setup guide
│   │   ├── index.mdx
│   │   ├── ros2-installation.mdx
│   │   ├── gazebo-setup.mdx
│   │   └── isaac-setup.mdx
│   ├── module-1-ros2/                 # Module 1: Robotic Nervous System
│   │   ├── index.mdx                  # Module overview
│   │   ├── understanding-ros2.mdx     # Chapter 1.1
│   │   ├── nodes-topics.mdx           # Chapter 1.2
│   │   ├── services-actions.mdx       # Chapter 1.3
│   │   ├── urdf-modeling.mdx          # Chapter 1.4
│   │   └── system-integration.mdx     # Chapter 1.5
│   ├── module-2-digital-twin/         # Module 2: Gazebo & Unity
│   │   ├── index.mdx
│   │   ├── digital-twin-concepts.mdx  # Chapter 2.1
│   │   ├── gazebo-physics.mdx         # Chapter 2.2
│   │   ├── unity-visualization.mdx    # Chapter 2.3
│   │   └── ros2-integration.mdx       # Chapter 2.4
│   ├── module-3-isaac/                # Module 3: AI-Robot Brain
│   │   ├── index.mdx
│   │   ├── ai-pipeline-concepts.mdx   # Chapter 3.1
│   │   ├── perception-vslam.mdx       # Chapter 3.2
│   │   ├── navigation-bipedal.mdx     # Chapter 3.3
│   │   └── learning-sim-to-real.mdx   # Chapter 3.4
│   ├── module-4-vla/                  # Module 4: Vision-Language-Action
│   │   ├── index.mdx
│   │   ├── vla-paradigm.mdx           # Chapter 4.1
│   │   ├── language-intent.mdx        # Chapter 4.2
│   │   ├── vision-grounding.mdx       # Chapter 4.3
│   │   └── multimodal-integration.mdx # Chapter 4.4
│   ├── capstone/                      # Capstone Project
│   │   ├── index.mdx
│   │   ├── system-design.mdx
│   │   ├── implementation-guide.mdx
│   │   └── edge-deployment.mdx
│   └── references.mdx                 # Global references section
├── static/
│   ├── diagrams/                      # Static diagram assets
│   │   ├── module-1/
│   │   ├── module-2/
│   │   ├── module-3/
│   │   ├── module-4/
│   │   └── capstone/
│   └── img/
│       └── book-logo.png
├── src/
│   └── css/
│       └── custom.css                 # Custom Docusaurus styling
├── docusaurus.config.js               # Main Docusaurus configuration
├── sidebars.js                        # Sidebar navigation structure
├── package.json                       # Node.js dependencies
└── README.md                          # Frontend-specific README

# Repository root (outside /Frontend)
specs/                                  # Spec-Kit artifacts
.specify/                               # Spec-Kit Plus framework
history/                                # PHRs, ADRs
CLAUDE.md                               # Agent guidance
```

**Structure Decision**: Documentation site using Docusaurus within `/Frontend` directory. All book content (modules, chapters, diagrams) lives in `Frontend/docs/`. Static assets in `Frontend/static/`. No backend services required - pure static site generation.

## Complexity Tracking

> **No violations** - All constitution principles are satisfied by this design. Context7 MCP integration ensures source-grounded Docusaurus implementation.

## Phase 0: Outline & Research

### Research Tasks

**R-001: Docusaurus Structure Best Practices (Context7 MCP)**
- **Objective**: Query Context7 MCP for official Docusaurus documentation on project initialization, docs structure, and navigation
- **Questions**:
  - What is the recommended directory structure for Docusaurus 3.x documentation sites?
  - How should `sidebars.js` be configured for multi-module hierarchical navigation?
  - What are the best practices for MDX file organization with categories and subcategories?
  - How to configure `docusaurus.config.js` for GitHub Pages deployment?
- **Output**: Document official Docusaurus patterns in `research.md`

**R-002: Module Content Architecture**
- **Objective**: Define optimal content organization for 4 modules + Capstone
- **Questions**:
  - How to structure progressive learning path (Module 1 → 2 → 3 → 4 → Capstone)?
  - What granularity for chapters within each module (3-5 chapters per module)?
  - How to handle cross-module references and prerequisites?
  - Should diagrams be embedded (MDX components) or static images?
- **Output**: Content taxonomy and navigation model in `research.md`

**R-003: Docusaurus MDX Features (Context7 MCP)**
- **Objective**: Consult Context7 for MDX syntax and features available in Docusaurus
- **Questions**:
  - What MDX features are supported for code blocks, diagrams, callouts?
  - How to implement tabs for multi-language code examples (Python, ROS 2 CLI)?
  - What syntax highlighting is available for ROS 2 XML (URDF), Python, YAML?
  - How to create custom components for exercises/checkpoints?
- **Output**: MDX feature set documentation in `research.md`

**R-004: GitHub Pages Deployment (Context7 MCP)**
- **Objective**: Query Context7 for official Docusaurus GitHub Pages deployment guidance
- **Questions**:
  - What is the recommended `docusaurus.config.js` setup for GitHub Pages?
  - How to configure GitHub Actions CI/CD for automatic deployment?
  - What base URL and trailing slash configurations are required?
  - How to handle custom domains (if needed)?
- **Output**: Deployment configuration guidance in `research.md`

**R-005: Code Example Testing Strategy**
- **Objective**: Define approach for testing all Python/ROS 2 code examples
- **Questions**:
  - Should code examples be tested in Docker containers (Ubuntu 22.04 + ROS 2 Humble)?
  - How to automate code example extraction and execution testing?
  - What test environments are needed (ROS 2, Gazebo, Isaac Sim)?
  - How to handle non-runnable conceptual code (NVIDIA Isaac without hardware)?
- **Output**: Testing strategy in `research.md`

**R-006: Diagram Generation and Integration**
- **Objective**: Determine diagram creation workflow and integration approach
- **Questions**:
  - Should diagrams be created as SVG, PNG, or Mermaid.js (if supported in Docusaurus)?
  - What tools for creating architecture diagrams, flowcharts, UML?
  - How to maintain diagram source files (draw.io, PlantUML)?
  - Should diagrams be text-described first, then rendered later?
- **Output**: Diagram workflow in `research.md`

**R-007: Sidebar Navigation Structure (Context7 MCP)**
- **Objective**: Consult Context7 for sidebar configuration best practices
- **Questions**:
  - How to create multi-level sidebar hierarchy (Module → Chapter → Sections)?
  - What syntax for collapsible sidebar categories?
  - How to handle prerequisite indicators (show Module 1 completed before Module 2)?
  - What patterns for "Next Chapter" / "Previous Chapter" navigation?
- **Output**: Sidebar structure design in `research.md`

**R-008: Navbar Configuration (Context7 MCP)**
- **Objective**: Query Context7 for navbar setup and customization
- **Questions**:
  - What navbar items should be included (Modules, References, GitHub link)?
  - How to configure dropdown menus for direct module access?
  - What search integration options are available (Algolia DocSearch)?
  - How to add version selector (if book has multiple editions)?
- **Output**: Navbar design in `research.md`

### Research Consolidation

**Output**: `specs/001-humanoid-robotics-book/research.md` with sections:
1. Docusaurus Project Structure (from R-001, verified via Context7)
2. Module Content Architecture (from R-002)
3. MDX Features and Syntax (from R-003, verified via Context7)
4. GitHub Pages Deployment (from R-004, verified via Context7)
5. Code Example Testing Strategy (from R-005)
6. Diagram Generation Workflow (from R-006)
7. Sidebar Navigation Structure (from R-007, verified via Context7)
8. Navbar Configuration (from R-008, verified via Context7)

**Decision Documentation**:
- **Docusaurus Version**: 3.x (latest stable, verified via Context7)
- **MDX Support**: Full MDX 3 with React components
- **Deployment**: GitHub Pages with GitHub Actions CI/CD
- **Diagram Approach**: Text-described first (in spec), SVG/PNG rendered later
- **Code Testing**: Docker containers with ROS 2 Humble + dependencies
- **Navigation**: Hierarchical sidebar with collapsible modules

## Phase 1: Design & Contracts

### Prerequisite: `research.md` complete

### Design Artifacts

**D-001**: Generate `data-model.md` - Content taxonomy and navigation model
**D-002**: Generate `contracts/module-structure.md` - Standardized module structure
**D-003**: Generate `contracts/chapter-template.md` - Reusable MDX chapter template
**D-004**: Generate `contracts/code-example-format.md` - Code example standards
**D-005**: Generate `quickstart.md` - Author workflow guide

### Agent Context Update

After completing research and design:

```bash
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
```

This updates `CLAUDE.md` with Docusaurus-specific guidance, MDX syntax, and project structure.

## Phase 1 Outputs

**Deliverables**:
1. `specs/001-humanoid-robotics-book/data-model.md` - Content taxonomy and navigation model
2. `specs/001-humanoid-robotics-book/contracts/module-structure.md` - Module structure contract
3. `specs/001-humanoid-robotics-book/contracts/chapter-template.md` - Reusable MDX chapter template
4. `specs/001-humanoid-robotics-book/contracts/code-example-format.md` - Code example standards
5. `specs/001-humanoid-robotics-book/quickstart.md` - Author workflow guide
6. `CLAUDE.md` (updated) - Agent context with Docusaurus guidance

## Re-Evaluate Constitution Check (Post-Design)

### Verification

- ✅ **Principle I (Precision & Depth)**: Research phase includes verification via official docs (Context7 MCP)
- ✅ **Principle II (Consistency)**: Module structure contract enforces standard chapter format
- ✅ **Principle III (Source-Aware, Context7)**: All Docusaurus decisions traced to Context7 MCP queries
- ✅ **Principle IV (Modularity)**: Navigation model supports independent module reading
- ✅ **Principle V (Pedagogy)**: Chapter structure enforces progressive complexity
- ✅ **Principle VI (Spec-First, Source-Grounded)**: This plan required before implementation; Context7 for all UI/config
- ✅ **Principle VII (Code Quality)**: Code example format includes testing requirements

**Post-Design Gate**: ✅ PASSED - All principles reinforced through design artifacts

## Phase 2: Task Generation

**Note**: Phase 2 (task generation) is handled by `/sp.tasks` command, NOT by `/sp.plan`.

After Phase 0 and Phase 1 completion, run:

```bash
/sp.tasks
```

This will generate `specs/001-humanoid-robotics-book/tasks.md` with concrete implementation tasks.

## Implementation Decisions (2025-12-16)

### ADR-001: Docusaurus BaseURL Configuration for Local Development

**Context**: Docusaurus `baseUrl` configuration affects routing. For GitHub Pages deployment at a subpath, `baseUrl` should be `/humanoid-robotics-book/`, but this breaks local development at `localhost:3000/`.

**Decision**: Set `baseUrl: '/'` for local development in `Frontend/docusaurus.config.js` with an inline comment indicating to change to `/humanoid-robotics-book/` for GitHub Pages deployment.

**Rationale**:
- **Local development UX**: Setting `baseUrl: '/'` allows developers to access the site at `http://localhost:3000/` without needing the subpath
- **Deployment flexibility**: Comment serves as documentation for production deployment configuration
- **Standard practice**: Separate development and production configurations is a common pattern

**Consequences**:
- ✅ Improved local development experience
- ⚠️ Requires manual configuration change before GitHub Pages deployment (future: use environment variables)
- ✅ Aligns with Docusaurus best practices for dual environments

**Implementation**: `Frontend/docusaurus.config.js:19`

---

### ADR-002: Landing Page Component vs. Redirect

**Context**: Original implementation used `<Redirect to="/docs/intro" />` in `Frontend/src/pages/index.js`, which created routing issues and a poor user experience (no visible landing page).

**Decision**: Replace redirect with a proper landing page React component featuring a hero banner, site title/tagline, and "Get Started" button linking to `/docs/`.

**Rationale**:
- **Better UX**: Users see a welcoming landing page instead of immediate redirect
- **SEO benefits**: Landing page provides content for search engines
- **Docusaurus conventions**: Docusaurus best practice is to have a custom landing page at `/src/pages/index.js`
- **Flexibility**: Landing page can showcase key features, testimonials, or navigation options

**Consequences**:
- ✅ Professional appearance with branded landing page
- ✅ Improved SEO and discoverability
- ✅ Better first-impression for new readers
- ⚠️ Requires maintaining `index.module.css` for styling

**Implementation**: `Frontend/src/pages/index.js:1-50`, `Frontend/src/pages/index.module.css:1-25`

---

### ADR-003: Docs Routing with Slug Configuration

**Context**: The `intro.mdx` file has `slug: /` which makes it accessible at `/docs/` (the root of the docs section), not at `/intro`. Initial "Get Started" button incorrectly linked to `/intro`.

**Decision**: Link "Get Started" button to `/docs/` to correctly route to the intro page with `slug: /`.

**Rationale**:
- **Docusaurus routing behavior**: With classic preset, docs are served at `/docs/` by default. A doc with `slug: /` becomes `/docs/`
- **Consistency**: Aligns with Docusaurus convention for docs routing
- **User expectation**: Clicking "Get Started" should take users to the beginning of documentation

**Consequences**:
- ✅ Correct routing to intro page
- ✅ Follows Docusaurus best practices
- ✅ Intuitive user navigation flow

**Implementation**: `Frontend/src/pages/index.js:18`

---

### ADR-004: Frontend Component Structure

**Context**: Need to organize React components, styles, and static assets within `/Frontend` directory.

**Decision**: Follow standard Docusaurus structure:
- React components and pages in `Frontend/src/pages/`
- CSS modules colocated with components (e.g., `index.module.css`)
- MDX documentation in `Frontend/docs/`
- Static assets in `Frontend/static/`

**Rationale**:
- **Docusaurus conventions**: Aligns with official Docusaurus project structure
- **Maintainability**: Clear separation of concerns (components, styles, content, assets)
- **Colocation**: CSS modules next to components for easier maintenance
- **Build optimization**: Docusaurus build system optimized for this structure

**Consequences**:
- ✅ Standard Docusaurus structure
- ✅ Easy onboarding for contributors familiar with Docusaurus
- ✅ Optimized build performance
- ✅ Clear file organization

**Implementation**: Current `Frontend/` directory structure

---

## Execution Summary

**Command Ends Here** - Implementation plan complete through Phase 1 design and initial implementation decisions.

**Completed**:
- ✅ Docusaurus initialization in `/Frontend`
- ✅ Landing page implementation
- ✅ Routing configuration for local development
- ✅ Documentation structure (intro, modules, capstone, references)
- ✅ Sidebar navigation configuration

**Next Steps**:
1. ~~Execute Phase 0 research tasks (R-001 through R-008)~~ - COMPLETED
2. ~~Execute Phase 1 design tasks (D-001 through D-005)~~ - COMPLETED
3. ~~Run `/sp.tasks` to generate task breakdown~~ - COMPLETED
4. ~~Begin Phase 2 implementation (Docusaurus initialization in `/Frontend`)~~ - COMPLETED
5. Continue Module content development (Module 1-4, Capstone)
6. Add diagrams and code examples per constitution requirements
7. Test all code examples in clean environments
8. Configure GitHub Pages deployment (update `baseUrl` before deployment)

**Branch**: `001-humanoid-robotics-book`
**Plan Location**: `F:\Book3\Humanoid-Robotics-Book\specs\001-humanoid-robotics-book\plan.md`
**Last Updated**: 2025-12-16
