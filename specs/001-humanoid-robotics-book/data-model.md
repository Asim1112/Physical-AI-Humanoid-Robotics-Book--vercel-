# Data Model: Physical AI & Humanoid Robotics Book

**Feature**: `001-humanoid-robotics-book` | **Date**: 2025-12-15 | **Input**: research.md + spec.md

## Summary

Content taxonomy and navigation model for the Physical AI & Humanoid Robotics book. Defines the hierarchical structure, relationships between content entities, and navigation patterns that support both linear progression and independent module consumption.

## Content Taxonomy

### Core Entities

#### 1. Module
- **Definition**: Major content section covering a specific domain (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- **Structure**: Contains 3-5 chapters with a module overview/index page
- **Purpose**: Provides comprehensive coverage of a single domain with clear prerequisites
- **Navigation**: Collapsible sidebar category with nested chapters
- **Relationships**: May depend on previous modules; clearly indicates prerequisites

#### 2. Chapter
- **Definition**: Focused unit within a module covering a specific topic
- **Structure**: Follows standard template (Overview → Deep Explanation → Practical Examples → Summary)
- **Length**: 1,500-3,000 words
- **Purpose**: Delivers specific learning objectives within the broader module context
- **Navigation**: Sidebar item under parent module category

#### 3. Section
- **Definition**: Subdivision within a chapter covering specific aspects of the topic
- **Structure**: Uses H2 and H3 headings for organization
- **Purpose**: Breaks down complex topics into digestible parts
- **Navigation**: Table of contents within the chapter page

#### 4. Code Example
- **Definition**: Runnable Python code demonstrating a concept
- **Structure**: Fenced code block with language identifier and description
- **Purpose**: Provides practical implementation of theoretical concepts
- **Relationships**: Linked to specific concepts explained in the text

#### 5. Diagram
- **Definition**: Visual representation of concepts (architecture, flowcharts, system interactions)
- **Structure**: Text-described first, rendered as SVG/PNG
- **Purpose**: Enhances understanding of complex relationships and systems
- **Navigation**: Referenced in text with figure numbers

#### 6. Exercise/Checkpoint
- **Definition**: Learning validation activity at the end of sections or chapters
- **Structure**: Questions or tasks with expected outcomes
- **Purpose**: Validates understanding and reinforces learning
- **Relationships**: Connected to specific learning objectives

## Navigation Model

### Primary Navigation Structure
```
Home
├── Introduction
├── Lab Setup
│   ├── ROS 2 Installation
│   ├── Gazebo Setup
│   └── Isaac Setup
├── Module 1: Robotic Nervous System (ROS 2)
│   ├── Overview
│   ├── Understanding ROS 2
│   ├── Nodes and Topics
│   ├── Services and Actions
│   ├── URDF Modeling
│   └── System Integration
├── Module 2: Digital Twin (Gazebo & Unity)
│   ├── Overview
│   ├── Digital Twin Concepts
│   ├── Gazebo Physics
│   ├── Unity Visualization
│   └── ROS 2 Integration
├── Module 3: AI-Robot Brain (NVIDIA Isaac)
│   ├── Overview
│   ├── AI Pipeline Concepts
│   ├── Perception and SLAM
│   ├── Navigation and Planning
│   └── Learning and Deployment
├── Module 4: Vision-Language-Action (VLA)
│   ├── Overview
│   ├── VLA Paradigm
│   ├── Language Understanding
│   ├── Vision-Guided Actions
│   └── Multi-Modal Integration
├── Capstone Project
│   ├── System Design
│   ├── Implementation Guide
│   └── Edge Deployment
└── References
```

### Breadcrumb Navigation
- Shows hierarchical path: Home → Module → Chapter → Section
- Allows easy navigation back to parent sections

### Next/Previous Chapter Navigation
- Horizontal navigation at bottom of each chapter
- "Continue to Module 2: Chapter 1" or "Back to Module 1: Chapter 3"
- Clear indication of progress through the book

### Module Prerequisites Indicator
- Visual indicators showing prerequisite modules
- "Prerequisites: Module 1" displayed on Module 2+ overview pages
- Option to review prerequisite concepts before proceeding

## Content Relationships

### Prerequisite Relationships
- Module 2 (Simulation) requires Module 1 (ROS 2) concepts
- Module 3 (AI Perception) requires Module 1 (ROS 2) and Module 2 (Simulation)
- Module 4 (VLA) requires all previous modules
- Capstone requires all modules

### Cross-Module References
- Links to related concepts in other modules
- "See also: Module 2, Chapter 3 for simulation integration"
- Contextual tooltips with brief explanations

### Learning Path Options
- Linear path: Follow modules in sequence (recommended for beginners)
- Jump-in path: Advanced users can start with specific modules
- Review path: Navigate back to foundational concepts when needed

## Metadata Schema

### Module Metadata
```yaml
title: "Module X: Module Title"
description: "Brief description of the module"
prerequisites: ["module-1", "module-2"]  # List of required modules
learningObjectives:
  - "Objective 1"
  - "Objective 2"
  - "Objective 3"
estimatedTime: "15-20 hours"
```

### Chapter Metadata
```yaml
title: "Chapter Title"
description: "Brief description of the chapter"
module: "module-1"
learningObjectives:
  - "Specific learning objective"
prerequisites: ["chapter-id-from-same-or-previous-module"]
estimatedTime: "3-4 hours"
tags: ["tag1", "tag2", "tag3"]
```

### Code Example Metadata
```yaml
title: "Example Title"
description: "Brief description of what the example demonstrates"
language: "python"
difficulty: "beginner|intermediate|advanced"
prerequisites: ["concept-id"]
expectedOutput: "Description of expected results"
```

## Search and Discovery

### Content Indexing
- Full-text search across all modules and chapters
- Tag-based filtering for specific topics
- Concept-based search returning related content

### Semantic Relationships
- "Related Concepts" section on each page
- "See Also" recommendations based on content similarity
- Cross-references to related topics in other modules

## Accessibility Considerations

### Navigation Aids
- Skip-to-content links
- Semantic heading structure (H1 → H6)
- ARIA labels for interactive elements
- Keyboard navigation support

### Content Structure
- Consistent layout across all pages
- Clear visual hierarchy
- Sufficient color contrast
- Alternative text for diagrams and images

## Progressive Disclosure

### Module-Level Progression
- Start with overview and motivation
- Progress to detailed explanations
- End with integration and application

### Chapter-Level Progression
- Overview → Deep Explanation → Examples → Summary
- Each section builds on previous concepts
- Clear transitions between topics

### Concept-Level Progression
- Start with simple concepts
- Gradually introduce complexity
- Provide multiple examples for reinforcement