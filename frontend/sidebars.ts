import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2/table-of-contents',
        'module-1-ros2/introduction-physical-ai',
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/launch-files',
        'module-1-ros2/urdf-robot-description',
        'module-1-ros2/tf-transformations',
        'module-1-ros2/best-practices',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity Simulation',
      items: [
        'module-2-simulation/gazebo-basics',
        'module-2-simulation/sdf-world-building',
        'module-2-simulation/sensor-simulation',
        'module-2-simulation/unity-robotics-hub',
        'module-2-simulation/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      items: [
        'module-3-isaac/isaac-sim-intro',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/vslam-cuVSLAM',
        'module-3-isaac/nav2-navigation',
        'module-3-isaac/isaac-manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vla/vla-overview',
        'module-4-vla/whisper-voice-to-action',
        'module-4-vla/llm-cognitive-planning',
        'module-4-vla/multimodal-interaction',
        'module-4-vla/vla-training-deployment',
        'module-4-vla/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/installation-guide',
        'appendices/ros2-command-reference',
        'appendices/troubleshooting',
        'appendices/resources',
        'appendices/instructor-guide',
      ],
    },
  ],
};

export default sidebars;
