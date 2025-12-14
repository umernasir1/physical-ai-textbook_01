// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Bridging the gap between the digital brain and the physical body',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://umernasir1.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/-physical-ai-textbook_01/',

  // Add custom fields
  customFields: {
    BACKEND_API_URL: process.env.BACKEND_API_URL || 'http://localhost:8000/api/v1',
  },

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'umernasir1', // Usually your GitHub org/user name.
  projectName: '-physical-ai-textbook_01', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/umernasir1/physical-ai-textbook/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: '📚 Modules',
          },
          {to: '/blog', label: '📝 Blog', position: 'left'},
          {to: '/auth', label: '🔐 Login', position: 'right'},
          {
            href: 'https://github.com/umernasir1/physical-ai-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Resources',
            items: [
              {
                label: 'Getting Started',
                to: '/docs/intro',
              },
              {
                label: 'ROS 2 Fundamentals',
                to: '/docs/module1-ros2/introduction-to-ros2',
              },
              {
                label: 'Digital Twin & Simulation',
                to: '/docs/module2-digital-twin/physics-simulation-environment-building',
              },
              {
                label: 'VLA Integration',
                to: '/docs/module4-vla/llms-and-robotics-convergence',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub Discussions',
                href: 'https://github.com/umernasir1/physical-ai-textbook/discussions',
              },
              {
                label: 'Discord Community',
                href: 'https://discord.gg/robotics',
              },
              {
                label: 'Twitter/X',
                href: 'https://twitter.com/physical_ai',
              },
              {
                label: 'LinkedIn',
                href: 'https://linkedin.com/company/physical-ai',
              },
            ],
          },
          {
            title: 'More Resources',
            items: [
              {
                label: 'Blog & Tutorials',
                to: '/blog',
              },
              {
                label: 'Project Showcase',
                href: 'https://github.com/umernasir1/physical-ai-textbook/discussions/categories/show-and-tell',
              },
              {
                label: 'Contribute',
                href: 'https://github.com/umernasir1/physical-ai-textbook/blob/main/CONTRIBUTING.md',
              },
              {
                label: 'Report Issues',
                href: 'https://github.com/umernasir1/physical-ai-textbook/issues',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with ❤️ and Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
