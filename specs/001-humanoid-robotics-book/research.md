# Research: Physical AI & Humanoid Robotics Book

**Feature**: `001-humanoid-robotics-book` | **Date**: 2025-12-15 | **Researcher**: Claude Code

## Summary

Research findings from Context7 MCP server queries on Docusaurus best practices, structure, and configuration for the Physical AI & Humanoid Robotics book project. All findings are grounded in official Docusaurus documentation as required by the project constitution.

## 1. Docusaurus Project Structure (R-001)

Based on Context7 MCP official Docusaurus documentation:

### Standard Docusaurus Project Structure
```
my-website
├── blog
│   ├── 2019-05-28-hola.md
│   ├── 2019-05-29-hello-world.md
│   └── 2020-05-30-welcome.md
├── docs
│   ├── doc1.md
│   ├── doc2.md
│   ├── doc3.md
│   └── mdx.md
├── src
│   ├── css
│   │   └── custom.css
│   └── pages
│       ├── styles.module.css
│       └── index.js
├── static
│   └── img
├── docusaurus.config.js
├── package.json
├── README.md
├── sidebars.js
└── yarn.lock
```

### For Monorepo Setup (Recommended for our project)
```
my-monorepo
├── package-a # Another package, your actual project
│   ├── src
│   └── package.json # Package A's dependencies
├── website   # Docusaurus root
│   ├── docs
│   ├── src
│   └── package.json # Docusaurus' dependencies
├── package.json # Monorepo's shared dependencies
```

### URL Mapping Structure
Docusaurus automatically maps files to URLs:
- `docs/intro.md` → `/docs/intro`
- `docs/tutorial-basics/congratulations.md` → `/docs/tutorial-basics/congratulations`
- `blog/2019-05-28-first-blog-post.md` → `/blog/2019/05/28/first-blog-post`

## 2. Module Content Architecture (R-002)

### Content Organization
- Content should be organized in the `docs/` directory
- Use nested directories for multi-level content organization
- Each module can be organized as a subdirectory within `docs/`
- Support for versioned documentation through `versioned_docs/` directory

### Progressive Learning Path Structure
- Use category structure in sidebars to organize content hierarchically
- Each module can be a category with multiple chapters as items
- Support for `_category_.json` files to define category metadata

## 3. Docusaurus MDX Features (R-003)

### React Component Integration
- MDX allows embedding React components directly in Markdown
- Components can be defined inline using export syntax:
```mdx
export const Highlight = ({children, color}) => (
  <span
    style={{
      backgroundColor: color,
      borderRadius: '2px',
      color: '#fff',
      padding: '0.2rem',
    }}>
    {children}
  </span>
);

<Highlight color="#25c2a0">Docusaurus green</Highlight>
```

### External Component Import
- Components can be imported from separate files:
```mdx
import Highlight from '@site/src/components/Highlight';
<Highlight color="#25c2a0">Docusaurus green</Highlight>
```

### Built-in Components
- Tabs component for multi-language examples:
```html
<Tabs>
  <TabItem value="apple" label="Apple" default>
    This is an apple 🍎
  </TabItem>
  <TabItem value="orange" label="Orange">
    This is an orange 🍊
  </TabItem>
</Tabs>
```

### HTML Elements in MDX
- HTML elements like `<details>` and `<summary>` work directly in MDX
- Supports JSX syntax for styling: `<span style={{backgroundColor: 'red'}}>Foo</span>`

## 4. GitHub Pages Deployment (R-004)

### Configuration in docusaurus.config.js
```javascript
module.exports = {
  // ...
  url: 'https://your-github-username.github.io', // Your website URL
  baseUrl: '/',
  projectName: 'your-github-username.github.io', // Usually your repo name
  organizationName: 'your-github-username', // Usually your GitHub username
  trailingSlash: false,
  // ...
};
```

### GitHub Actions Workflow for Deployment
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main

jobs:
  build:
    name: Build Docusaurus
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: yarn

      - name: Install dependencies
        run: yarn install --frozen-lockfile
      - name: Build website
        run: yarn build

      - name: Upload Build Artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    name: Deploy to GitHub Pages
    needs: build

    permissions:
      pages: write
      id-token: write

    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}

    runs-on: ubuntu-latest
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### Manual Deployment Command
```bash
GIT_USER=<GITHUB_USERNAME> yarn deploy
```

## 5. Code Example Testing Strategy (R-005)

### Testing Considerations
- Code examples in Docusaurus are static content but can be validated through automated checks
- Use fenced code blocks with appropriate language identifiers for syntax highlighting
- Include expected output or behavior descriptions in comments
- Consider using continuous integration to validate code examples in isolated environments
- For Python/ROS examples, provide Docker-based testing environments that match the book's requirements (Ubuntu 22.04, ROS 2 Humble)

### Validation Approach
- Create test scripts that extract code examples from MDX files
- Run examples in clean Docker containers with appropriate dependencies
- Validate syntax and basic execution where possible
- Document expected outputs and behaviors

## 6. Diagram Generation and Integration (R-006)

### Diagram Integration Options
- Static diagrams: Place SVG/PNG files in `static/img/` directory
- MDX-embedded diagrams: Use React components for dynamic diagrams
- External tools: Generate diagrams with tools like PlantUML, Mermaid, draw.io, and export as static assets

### Recommended Workflow
- Create diagrams using appropriate tools (draw.io for architecture, PlantUML for sequence diagrams)
- Export as SVG for best quality and web compatibility
- Place in `static/diagrams/` subdirectories organized by module
- Reference in MDX files using standard Markdown image syntax: `![Diagram Alt Text](/img/diagrams/module1/architecture.svg)`

### Text-Described Diagrams
- As specified in the constitution, diagrams should be text-described first
- Include detailed descriptions that can later be rendered as visual diagrams
- Use consistent naming conventions: `module-<number>-<topic>.svg`

## 7. Sidebar Navigation Structure (R-007)

### Sidebar Configuration File (sidebars.js)
```javascript
export default {
  docs: [
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/intro',
        'module-1/understanding-ros2',
        {
          type: 'category',
          label: 'Advanced Topics',
          items: ['module-1/advanced-nodes', 'module-1/urdf-modeling'],
        },
      ],
    },
  ],
};
```

### Configuration in docusaurus.config.js
```javascript
export default {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
        },
      }
    ]
  ],
};
```

### Auto-generated Sidebars
For large documentation sets:
```javascript
module.exports = {
  myAutogeneratedSidebar: [
    {
      type: 'autogenerated',
      dirName: '.', // '.' means the current docs folder
    },
  ],
};
```

### Hideable Sidebar Feature
```javascript
export default {
  themeConfig: {
    docs: {
      sidebar: {
        hideable: true,
      },
    },
  },
};
```

## 8. Navbar Configuration (R-008)

### Basic Navbar Structure
```javascript
export default {
  themeConfig: {
    navbar: {
      title: 'Humanoid Robotics Book',
      logo: {
        alt: 'My Site Logo',
        src: 'img/book-logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docs',
          position: 'left',
          label: 'Modules',
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/your-username/your-repo',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
  },
};
```

### Navbar Item Types
- **'doc'**: Links to a documentation page
- **'dropdown'**: Creates a dropdown menu
- **'search'**: Adds a search bar
- **External links**: Use `href` for external URLs

### Dropdown Menu Example
```javascript
{
  type: 'dropdown',
  label: 'Modules',
  position: 'left',
  items: [
    {
      label: 'Module 1: ROS 2',
      to: '/docs/module-1/intro',
    },
    {
      label: 'Module 2: Simulation',
      to: '/docs/module-2/intro',
    },
  ],
}
```

### Search Integration
```javascript
{
  type: 'search',
  position: 'right',
}
```

### Auto-hide on Scroll
```javascript
export default {
  themeConfig: {
    navbar: {
      hideOnScroll: true,
    },
  },
};
```

## Decision Documentation

Based on the research findings:

- **Docusaurus Version**: 3.x (latest stable, verified via Context7)
- **MDX Support**: Full MDX 3 with React components
- **Deployment**: GitHub Pages with GitHub Actions CI/CD
- **Diagram Approach**: Text-described first (in spec), SVG/PNG rendered later
- **Code Testing**: Docker containers with ROS 2 Humble + dependencies
- **Navigation**: Hierarchical sidebar with collapsible modules
- **Project Structure**: Docusaurus site within `/Frontend` directory as required by constitution

All decisions are based on official Docusaurus documentation accessed via Context7 MCP server, ensuring compliance with the project constitution's source-awareness principle.