import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/">
            Get Started →
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive guide to building AI-powered humanoid robots">
      <HomepageHeader />
      <main>
        <div className="container" style={{marginTop: '2rem', marginBottom: '2rem'}}>
          <div className="row">
            <div className="col col--12">
              <h2>Welcome to Physical AI & Humanoid Robotics</h2>
              <p>
                This comprehensive educational textbook bridges the gap between purely digital AI systems
                and embodied intelligence, teaching you how to build humanoid robots that can perceive,
                understand, reason, and act in the physical world.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
