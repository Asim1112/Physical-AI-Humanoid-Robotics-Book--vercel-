// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Lab Setup',
      items: [
        'lab-setup/index',
        'lab-setup/ros2-installation',
        'lab-setup/gazebo-setup',
        'lab-setup/isaac-setup'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1-ros2/index',
        'module-1-ros2/understanding-ros2',
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/urdf-modeling',
        'module-1-ros2/system-integration',
        'module-1-ros2/exercises-checkpoints'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/digital-twin-concepts',
        'module-2-digital-twin/gazebo-physics',
        'module-2-digital-twin/unity-visualization',
        'module-2-digital-twin/ros2-integration',
        'module-2-digital-twin/gazebo-integration',
        'module-2-digital-twin/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3-isaac/index',
        'module-3-isaac/ai-pipeline-concepts',
        'module-3-isaac/perception-vslam',
        'module-3-isaac/navigation-bipedal',
        'module-3-isaac/learning-sim-to-real',
        'module-3-isaac/isaac-integration',
        'module-3-isaac/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4-vla/index',
        'module-4-vla/vla-paradigm',
        'module-4-vla/language-intent',
        'module-4-vla/vision-grounding',
        'module-4-vla/multimodal-integration',
        'module-4-vla/vla-deployment',
        'module-4-vla/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsible: true,
      collapsed: false,
      items: [
        'capstone/index',
        'capstone/system-design',
        'capstone/implementation-guide',
        'capstone/edge-deployment'
      ],
    },
    'references'
  ],
};

export default sidebars;