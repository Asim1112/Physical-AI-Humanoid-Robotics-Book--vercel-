# Quickstart Guide: Physical AI & Humanoid Robotics Book

**Feature**: `001-humanoid-robotics-book` | **Date**: 2025-12-15 | **Input**: plan.md, research.md, data-model.md

## Summary

Quickstart guide for authors and contributors to the Physical AI & Humanoid Robotics book project. Provides essential information for getting started with content creation, development workflows, and project conventions.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or WSL2 on Windows
- **Memory**: 16GB RAM minimum (32GB recommended for simulation work)
- **Storage**: 50GB free space for development environment
- **Processor**: Multi-core processor with virtualization support

### Software Dependencies
- **Node.js**: Version 18.x or higher
- **Yarn**: Package manager (v1.22+ or v3.x)
- **Git**: Version control system (v2.30+)
- **Python**: Version 3.10+ for code examples
- **Docker**: For isolated testing environments (optional but recommended)

## Project Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Frontend Directory
```bash
cd Frontend
```

### 3. Install Dependencies
```bash
yarn install
```

### 4. Start Development Server
```bash
yarn start
```
This will start the Docusaurus development server at `http://localhost:3000`

## Development Workflow

### Content Creation Process

#### 1. Create New Chapter
1. Navigate to the appropriate module directory in `Frontend/docs/`
2. Create a new `.mdx` file following the naming convention: `chapter-number-topic.mdx`
3. Use the chapter template from `contracts/chapter-template.md`
4. Add the new file to the sidebar in `sidebars.js`

#### 2. Add Code Examples
1. Follow the format standards in `contracts/code-example-format.md`
2. Include proper language specification and descriptive comments
3. Test examples in the target environment (Ubuntu 22.04 + ROS 2 Humble)
4. Document expected output and behavior

#### 3. Include Diagrams
1. Create diagrams using appropriate tools (draw.io, PlantUML, etc.)
2. Export as SVG for best web compatibility
3. Place in `Frontend/static/diagrams/module-x/` directory
4. Reference in MDX files with proper alt text

### Docusaurus Commands

#### Development
```bash
yarn start                    # Start development server
yarn build                    # Build static files to 'build' directory
yarn serve                    # Serve built files locally for testing
yarn swizzle [component]      # Copy component to 'src/theme' for customization
```

#### Documentation
```bash
yarn docusaurus --help        # Show available commands
yarn docusaurus write-translations # Extract translation strings
yarn docusaurus docs:version [version] # Create versioned docs
```

## Project Structure

### Frontend Directory (`/Frontend`)
```
Frontend/
├── docs/                     # All book content (modules, chapters)
│   ├── intro.mdx             # Book introduction
│   ├── lab-setup/            # Environment setup guide
│   ├── module-1-ros2/        # Module 1 content
│   ├── module-2-digital-twin/ # Module 2 content
│   ├── module-3-isaac/       # Module 3 content
│   ├── module-4-vla/         # Module 4 content
│   ├── capstone/             # Capstone project
│   └── references.mdx        # Global references
├── static/                   # Static assets (images, diagrams)
│   ├── diagrams/             # SVG/PNG diagrams organized by module
│   └── img/                  # Other image assets
├── src/                      # Custom React components
│   └── css/                  # Custom styles
├── docusaurus.config.js      # Main Docusaurus configuration
├── sidebars.js               # Navigation sidebar configuration
├── package.json              # Node.js dependencies
└── README.md                 # Frontend-specific documentation
```

### Configuration Files

#### `docusaurus.config.js`
- Site metadata (title, description, URL)
- Plugin configurations (docs, blog, sitemap)
- Theme customization
- Deployment settings (GitHub Pages)

#### `sidebars.js`
- Navigation hierarchy
- Module and chapter organization
- Collapsible sections
- Next/previous navigation

## Writing Guidelines

### Content Standards
1. **Follow the Chapter Template**: Use the template from `contracts/chapter-template.md`
2. **Maintain Consistent Voice**: Friendly, expressive, educational tone
3. **Reference Official Documentation**: All technical claims must be verifiable
4. **Include Practical Examples**: 2-4 runnable examples per chapter minimum

### Technical Requirements
1. **Code Example Standards**: Follow `contracts/code-example-format.md`
2. **Module Structure**: Adhere to `contracts/module-structure.md`
3. **Cross-Module References**: Use proper linking between related concepts
4. **Accessibility**: Include alt text for images and proper heading structure

## Testing and Validation

### Local Testing
1. **Build Validation**: Run `yarn build` to ensure site builds without errors
2. **Link Checking**: Verify all internal links resolve correctly
3. **Code Example Testing**: Test examples in target environment
4. **Cross-Module Navigation**: Ensure navigation between modules works

### Quality Checks
```bash
# Check for broken links
yarn docusaurus build
# Verify build completes successfully

# Check local server
yarn start
# Verify all pages load correctly
```

## Deployment

### GitHub Pages Deployment
1. Ensure `docusaurus.config.js` has correct GitHub Pages settings:
   ```javascript
   {
     url: 'https://your-username.github.io',
     baseUrl: '/repository-name/',
     projectName: 'repository-name',
     organizationName: 'your-username',
     trailingSlash: false,
   }
   ```

2. Use the GitHub Actions workflow for automated deployment (see `research.md` for workflow example)

3. Or deploy manually:
   ```bash
   GIT_USER=<GITHUB_USERNAME> yarn deploy
   ```

### Deployment Checks
- [ ] Site builds without errors (`yarn build`)
- [ ] All internal links resolve correctly
- [ ] Code examples are functional
- [ ] Images and diagrams load properly
- [ ] Mobile responsiveness verified

## Authoring Tools and Tips

### Recommended Editors
- **VS Code**: With MDX and Docusaurus extensions
- **Vim/Neovim**: With markdown plugins
- **Any text editor**: That supports syntax highlighting

### MDX Features in Use
- React components in Markdown
- Tabs for multi-language examples
- JSX for custom elements
- Front matter for metadata

### Version Control Best Practices
- **Branch Strategy**: Feature branches for new content
- **Commit Messages**: Clear, descriptive messages following conventional format
- **Pull Requests**: Review process for all content changes
- **File Organization**: Follow established directory structure

## Troubleshooting

### Common Issues

#### Build Errors
- **Symptom**: `yarn build` fails with errors
- **Solution**: Check for syntax errors in MDX files, verify all referenced files exist

#### Local Development Issues
- **Symptom**: `yarn start` doesn't work
- **Solution**: Clear cache with `yarn start --clear-cache`, reinstall dependencies

#### Link Resolution Problems
- **Symptom**: Broken internal links
- **Solution**: Verify file paths match sidebar configuration

### Performance Tips
- **Fast Refresh**: Use `yarn start` for real-time editing
- **Selective Building**: Edit one file at a time to minimize rebuild time
- **Image Optimization**: Compress images before adding to static directory

## Quality Assurance

### Pre-Submission Checklist
- [ ] Chapter follows standard template structure
- [ ] All code examples are runnable and well-documented
- [ ] Diagrams have proper alt text and descriptions
- [ ] Cross-references link to correct locations
- [ ] Content maintains consistent tone and style
- [ ] Technical claims reference official documentation
- [ ] Chapter length is within specified range (1,500-3,000 words)
- [ ] Exercises have clear success criteria

### Review Process
1. **Self-Review**: Author verifies checklist items
2. **Technical Review**: Verify code examples and technical accuracy
3. **Content Review**: Check pedagogical flow and clarity
4. **Integration Review**: Ensure compatibility with overall book structure

## Getting Help

### Documentation References
- **Docusaurus Documentation**: https://docusaurus.io/docs (accessed via Context7 MCP)
- **ROS 2 Documentation**: https://docs.ros.org/ (official reference)
- **NVIDIA Isaac Documentation**: https://docs.nvidia.com/isaac/ (official reference)
- **Project Constitution**: `.specify/memory/constitution.md` (development principles)

### Support Channels
- **GitHub Issues**: For technical problems and feature requests
- **Project Wiki**: For detailed setup and configuration guides
- **Code Reviews**: For content feedback and suggestions

This quickstart guide provides the essential information needed to begin contributing to the Physical AI & Humanoid Robotics book project. For detailed information on specific aspects, refer to the other documentation files in the `specs/001-humanoid-robotics-book/` directory.