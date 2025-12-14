import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      {/* Animated Background Elements */}
      <div className={styles.heroBackground}>
        <div className={styles.gradientOrb1}></div>
        <div className={styles.gradientOrb2}></div>
        <div className={styles.gradientOrb3}></div>
      </div>

      <div className={clsx('container', styles.heroContainer)}>
        <div className={styles.heroContent}>
          <div className={styles.heroTextContent}>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>

            {/* Quick Stats */}
            <div className={styles.heroStats}>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>12+</div>
                <div className={styles.statLabel}>Modules</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>50+</div>
                <div className={styles.statLabel}>Lessons</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>30+</div>
                <div className={styles.statLabel}>Hours</div>
              </div>
            </div>

            {/* CTA Buttons */}
            <div className={styles.heroButtons}>
              <Link
                className={clsx('button button--primary button--lg', styles.primaryButton)}
                to="/docs/intro">
                Start Learning 🚀
              </Link>
              <Link
                className={clsx('button button--secondary button--lg', styles.secondaryButton)}
                to="/docs/module1-ros2/introduction-to-ros2">
                Explore Modules
              </Link>
            </div>
          </div>

          {/* Hero Illustration/Icon */}
          <div className={styles.heroIllustration}>
            <div className={styles.robotIcon}>
              <span className={styles.robotEmoji}>🤖</span>
              <div className={styles.floatingIcons}>
                <span className={styles.floatingIcon}>⚙️</span>
                <span className={styles.floatingIcon}>🧠</span>
                <span className={styles.floatingIcon}>📡</span>
              </div>
            </div>
          </div>
        </div>

        {/* Scroll Indicator */}
        <div className={styles.scrollIndicator}>
          <span>Scroll to explore</span>
          <div className={styles.scrollArrow}>↓</div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics with comprehensive tutorials, interactive simulations, and hands-on projects. Learn ROS 2, Digital Twins, VLA integration, and more.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
