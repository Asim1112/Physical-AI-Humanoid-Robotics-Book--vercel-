# Physical AI & Humanoid Robotics Book - Frontend

This directory contains the Docusaurus-based frontend for the Physical AI & Humanoid Robotics educational textbook. The book is designed to teach students how to create AI-powered humanoid robots that can perceive, reason, and act in physical and simulated environments.

## Project Structure

```
Frontend/
├── docs/                     # All book content (modules, chapters)
│   ├── intro.mdx             # Book introduction
│   ├── lab-setup/            # Environment setup guide
│   ├── module-1-ros2/        # Module 1: Robotic Nervous System (ROS 2)
│   ├── module-2-digital-twin/ # Module 2: Digital Twin (Gazebo & Unity)
│   ├── module-3-isaac/       # Module 3: AI-Robot Brain (NVIDIA Isaac)
│   ├── module-4-vla/         # Module 4: Vision-Language-Action (VLA)
│   ├── capstone/             # Capstone project integrating all concepts
│   └── references.mdx        # Global references section
├── static/                   # Static assets (images, diagrams)
│   └── img/                  # Image assets
├── src/                      # Custom React components and styles
│   └── css/                  # Custom styles
├── docusaurus.config.js      # Main Docusaurus configuration
├── sidebars.js               # Navigation sidebar configuration
├── package.json              # Node.js dependencies and scripts
└── README.md                 # This file
```

## Getting Started

### Prerequisites

- Node.js (version 18.x or higher)
- Yarn package manager
- Git

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>/Frontend
   ```

2. Install dependencies:
   ```bash
   yarn install
   ```

3. Start the development server:
   ```bash
   yarn start
   ```

This will start the Docusaurus development server at `http://localhost:3000`.

### Available Scripts

- `yarn start` - Start development server
- `yarn build` - Build static files for production
- `yarn serve` - Serve built files locally
- `yarn deploy` - Deploy to GitHub Pages (if configured)
- `yarn clear` - Clear Docusaurus cache

## Content Organization

The book is organized into four progressive modules:

1. **Module 1: Robotic Nervous System (ROS 2)** - Fundamentals of ROS 2 architecture
2. **Module 2: Digital Twin (Gazebo & Unity)** - Simulation and digital twin concepts
3. **Module 3: AI-Robot Brain (NVIDIA Isaac)** - AI perception and navigation
4. **Module 4: Vision-Language-Action (VLA)** - Natural language robot control

Each module builds upon the previous ones, but can also be read independently with appropriate prerequisites noted.

## Writing Guidelines

### Chapter Structure

Each chapter follows the standard template:
1. Overview with learning objectives
2. Deep explanation of concepts
3. Practical examples with code
4. Exercises and checkpoints
5. Summary and key takeaways

### Code Examples

All code examples follow the format standards:
- Proper language specification for syntax highlighting
- Descriptive comments explaining key concepts
- Expected output or behavior documentation
- Connection to humanoid robotics applications

### Content Standards

- Maintain friendly, expressive, educational tone
- Ensure technical accuracy with official documentation references
- Include 2-4 runnable examples per chapter
- Provide clear success criteria for exercises

## Deployment

This site is designed for deployment to GitHub Pages. The configuration in `docusaurus.config.js` includes settings for GitHub Pages deployment.

To deploy manually:
```bash
GIT_USER=<GITHUB_USERNAME> yarn deploy
```

For automated deployment, use the GitHub Actions workflow (see documentation in the project repository).

## Contributing

1. Create a feature branch for your changes
2. Follow the established content and code standards
3. Test changes locally before submitting
4. Submit a pull request with clear description of changes

## License

This educational content is made available under [appropriate educational content license]. See the main repository for full license information.