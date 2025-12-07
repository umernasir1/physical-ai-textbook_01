// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
        title: 'Module 1 Overview',
      },
      items: [
        'module1-ros2/introduction-to-physical-ai',
        'module1-ros2/ros2-nodes-topics-services',
        'module1-ros2/python-agents-and-rclpy',
        'module1-ros2/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'generated-index',
        title: 'Module 2 Overview',
      },
      items: [
        'module2-digital-twin/physics-simulation-environment-building',
        'module2-digital-twin/high-fidelity-rendering-human-robot-interaction',
        'module2-digital-twin/simulating-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {
        type: 'generated-index',
        title: 'Module 3 Overview',
      },
      items: [
        'module3-ai-robot-brain/advanced-perception-training',
        'module3-ai-robot-brain/nvidia-isaac-sim',
        'module3-ai-robot-brain/isaac-ros-vslam-navigation',
        'module3-ai-robot-brain/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'generated-index',
        title: 'Module 4 Overview',
      },
      items: [
        'module4-vla/llms-and-robotics-convergence',
        'module4-vla/voice-to-action-openai-whisper',
        'module4-vla/cognitive-planning-ros2-actions',
        'module4-vla/capstone-autonomous-humanoid',
      ],
    },
    'tutorial-basics/create-a-document', // Retain existing tutorial content for now, will be reviewed later
    'tutorial-extras/manage-docs-versions', // Retain existing tutorial content for now, will be reviewed later
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
