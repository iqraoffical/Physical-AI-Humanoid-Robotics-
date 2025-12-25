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
  // Sidebar for the main tutorial content
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Tutorial',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/markdown-features',
        'tutorial-basics/deploy-your-site',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
  ],

  // Sidebar for the book content
  bookSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        'intro',
        'course-assessments-hardware',
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            'module-1-ros2/week-03-ros2-introduction/nervous-system',
            'module-1-ros2/week-04-ros2-core-concepts/ros2-core-concepts',
            'module-1-ros2/week-04-ros2-packages-python',
            'module-1-ros2/week-05-ros2-packages-launch/ros2-packages-launch',
            'module-1-ros2/week-06-urdf-robot-description',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Digital Twin Implementation',
          items: [
            'module-2-digital-twin/week-06-gazebo-setup/gazebo-setup',
            'module-2-digital-twin/week-07-gazebo-sensors-unity/gazebo-sensors-unity',
            'module-2-digital-twin/week-08-physics-sensor-simulation',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'module-3-ai-robot-brain/week-08-isaac-sdk-sim/intro',
            'module-3-ai-robot-brain/week-08-isaac-sdk-sim/isaac-ecosystem',
            'module-3-ai-robot-brain/week-09-isaac-perception-rl/perception-pipeline',
            'module-3-ai-robot-brain/week-09-isaac-perception-rl/vslam-concepts',
            'module-3-ai-robot-brain/week-10-sim-to-real/sim-to-real-concepts',
            'module-3-ai-robot-brain/week-10-sim-to-real/end-to-end-pipeline',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action Systems',
          items: [
            'module-4-vla/week-01-physical-ai-foundations',
            'module-4-vla/week-02-embodied-intelligence',
            'module-4-vla/week-03-ros2-fundamentals',
            'module-3-ai-robot-brain/week-10-ai-perception-manipulation',
            'module-4-vla/week-11-humanoid-kinematics-dynamics/humanoid-kinematics-dynamics',
            'module-4-vla/week-12-bipedal-manipulation-interaction/bipedal-manipulation-interaction',
            'module-4-vla/week-12-conversational-ai',
            'module-4-vla/week-13-conversational-robotics/conversational-robotics',
          ],
        },
        {
          type: 'category',
          label: 'Capstone Project',
          items: [
            'chapter-06-capstone-infra/capstone-project/capstone-project',
            'chapter-06-capstone-infra/hardware-requirements/hardware-requirements',
            'chapter-06-capstone-infra/cloud-lab/cloud-lab',
          ],
        }
      ],
    },
  ],
};

export default sidebars;
