import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: '🤖 AI-Powered Learning',
    icon: '🧠',
    description: (
      <>
        Master robotics with AI assistance. Get instant answers, code explanations,
        and personalized guidance through our intelligent chatbot trained on the entire curriculum.
      </>
    ),
    link: '/docs/intro',
  },
  {
    title: '📚 Comprehensive Curriculum',
    icon: '📖',
    description: (
      <>
        From ROS 2 fundamentals to advanced VLA integration. Learn digital twins,
        URDF modeling, physics simulation, and cognitive planning for humanoid robots.
      </>
    ),
    link: '/docs/module1-ros2/introduction-to-ros2',
  },
  {
    title: '⚡ Hands-On Projects',
    icon: '🛠️',
    description: (
      <>
        Build real-world robotics applications. Work with NVIDIA Isaac Sim,
        implement vision-language-action models, and create fully autonomous humanoid systems.
      </>
    ),
    link: '/docs/module4-vla/llms-and-robotics-convergence',
  },
  {
    title: '🎯 Interactive Simulations',
    icon: '🎮',
    description: (
      <>
        Practice in safe virtual environments before deploying to hardware.
        Test algorithms, debug issues, and iterate faster with digital twin technology.
      </>
    ),
    link: '/docs/module2-digital-twin/physics-simulation-environment-building',
  },
  {
    title: '🌟 Real-World Applications',
    icon: '🏭',
    description: (
      <>
        Learn from industry use cases. Explore how Physical AI is transforming
        manufacturing, healthcare, logistics, and service robotics across the globe.
      </>
    ),
    link: '/docs/module3-ai-robot-brain/advanced-perception-training',
  },
  {
    title: '💬 Community Support',
    icon: '👥',
    description: (
      <>
        Join a vibrant community of robotics enthusiasts. Share projects,
        get help from experts, and collaborate on cutting-edge humanoid robotics research.
      </>
    ),
    link: '/blog',
  },
];

function Feature({title, icon, description, link, index}) {
  return (
    <div className={clsx('col col--4', styles.featureCol)} style={{ animationDelay: `${index * 0.1}s` }}>
      <Link to={link} className={styles.featureCard}>
        <div className={styles.featureIcon}>{icon}</div>
        <div className={styles.featureContent}>
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
          <div className={styles.featureArrow}>
            <span>Explore →</span>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <Heading as="h2" className={styles.featuresTitle}>
            Why Choose This Textbook?
          </Heading>
          <p className={styles.featuresSubtitle}>
            Everything you need to master Physical AI and Humanoid Robotics in one comprehensive platform
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}
