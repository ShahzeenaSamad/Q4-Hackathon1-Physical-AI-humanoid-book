import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import HeroRobot from '@site/src/components/HeroRobot';

function TextbookHomepage() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <div className={styles.mainContainer}>
      {/* Hero Section - Main Content with Robot */}
      <section className={styles.heroSection}>
        <div className="container">
          <div className={styles.heroLayout}>
            {/* Left side - Text content */}
            <div className={styles.heroContent}>
              <Heading as="h1" className="hero__title">
                {siteConfig.title}
              </Heading>
              <p className="hero__subtitle" style={{color: '#a8b2d1', fontSize: '1.4rem', fontWeight: 500}}>
                {siteConfig.tagline}
              </p>

              <div className={styles.heroDescription}>
                <p>Complete learning platform with ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems.</p>

                <div className={styles.heroButtons}>
                  {/* Button 1: Explore Textbook */}
                  <Link
                    className="button button--lg"
                    to="/docs/module-1-ros2/introduction-physical-ai"
                    style={{
                      background: 'linear-gradient(135deg, #26d0ce 0%, #1a2980 100%)',
                      color: 'white',
                      border: 'none',
                      padding: '1rem 2.5rem',
                      fontSize: '1.1rem',
                      fontWeight: '600',
                      borderRadius: '10px',
                      boxShadow: '0 8px 25px rgba(38, 208, 206, 0.4)',
                      transition: 'all 0.3s ease',
                      display: 'inline-block',
                      textDecoration: 'none'
                    }}
                    onMouseOver={(e) => {
                      e.currentTarget.style.transform = 'translateY(-3px)';
                      e.currentTarget.style.boxShadow = '0 12px 35px rgba(38, 208, 206, 0.6)';
                    }}
                    onMouseOut={(e) => {
                      e.currentTarget.style.transform = 'translateY(0)';
                      e.currentTarget.style.boxShadow = '0 8px 25px rgba(38, 208, 206, 0.4)';
                    }}>
                    Explore Textbook
                  </Link>

                  {/* Button 2: View Contents */}
                  <Link
                    className="button button--lg"
                    to="/docs/module-1-ros2/table-of-contents"
                    style={{
                      background: 'transparent',
                      color: '#26d0ce',
                      border: '2px solid #26d0ce',
                      padding: '1rem 2.5rem',
                      fontSize: '1.1rem',
                      fontWeight: '600',
                      borderRadius: '10px',
                      transition: 'all 0.3s ease',
                      display: 'inline-block',
                      textDecoration: 'none',
                      marginLeft: '1rem'
                    }}
                    onMouseOver={(e) => {
                      e.currentTarget.style.background = 'rgba(38, 208, 206, 0.1)';
                      e.currentTarget.style.transform = 'translateY(-3px)';
                    }}
                    onMouseOut={(e) => {
                      e.currentTarget.style.background = 'transparent';
                      e.currentTarget.style.transform = 'translateY(0)';
                    }}>
                    View Contents
                  </Link>
                </div>
              </div>
            </div>

            {/* Right side - Movable Robot Display */}
            <div className={styles.heroVisual}>
              <div className={`${styles.robotContainer} hero-robot-container`} style={{
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                position: 'relative',
                width: '400px',  /* Increased from 300px to 400px */
                height: '400px'  /* Increased from 300px to 400px */
              }}>
                <div className={styles.robotDisplay} style={{
                  background: 'rgba(10, 25, 41, 0.3)',
                  borderRadius: '15px',
                  padding: '2rem',  /* Increased padding */
                  border: '1px solid rgba(96, 211, 209, 0.2)',
                  boxShadow: '0 10px 40px rgba(0, 0, 0, 0.3)',
                  backdropFilter: 'blur(8px)',
                  width: '100%',
                  height: '100%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  overflow: 'visible' // Allow the robot to move freely within the container
                }}>
                  {/* Movable Hero Robot */}
                  <HeroRobot />

                  {/* Decorative background elements */}
                  <div style={{
                    position: 'absolute',
                    width: '200px',  /* Increased from 150px */
                    height: '200px', /* Increased from 150px */
                    borderRadius: '50%',
                    background: 'radial-gradient(circle, rgba(96, 211, 209, 0.1) 0%, transparent 70%)',
                    zIndex: 0
                  }}></div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>


      {/* Textbook Features Section */}
      <section className={styles.featuresSection}>
        <div className="container">
          <div className={styles.featuresHeader}>
            <h2>Textbook Features</h2>
            <p>A comprehensive learning resource that combines cutting-edge AI technology with educational excellence to create an unparalleled learning experience.</p>
          </div>

          <div className={styles.featuresGrid}>
            <div className={styles.featureCard}>
              <span className={styles.icon}>📚</span>
              <h3>Comprehensive Content</h3>
              <p>Detailed chapters covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems</p>
            </div>
            <div className={styles.featureCard}>
              <span className={styles.icon}>🤖</span>
              <h3>AI Assistant</h3>
              <p>Intelligent chatbot that answers questions about textbook content and concepts</p>
            </div>
            <div className={styles.featureCard}>
              <span className={styles.icon}>🎯</span>
              <h3>Hands-on Exercises</h3>
              <p>Practical examples and exercises to reinforce theoretical concepts</p>
            </div>
            <div className={styles.featureCard}>
              <span className={styles.icon}>📊</span>
              <h3>Progress Tracking</h3>
              <p>Monitor learning progress with detailed analytics and personalized insights</p>
            </div>
            <div className={styles.featureCard}>
              <span className={styles.icon}>🌐</span>
              <h3>Real-World Applications</h3>
              <p>Case studies and projects demonstrating practical implementations</p>
            </div>
            <div className={styles.featureCard}>
              <span className={styles.icon}>🔄</span>
              <h3>Interactive Learning</h3>
              <p>Engaging modules with real-time feedback and adaptive content</p>
            </div>
          </div>
        </div>
      </section>

      {/* Book Preview Section */}
      <section className={styles.demoSection}>
        <div className="container">
          <div className={styles.demoLayout}>
            <div className={styles.demoContent}>
              <h2>Interactive Textbook Experience</h2>
              <p>Our textbook combines traditional learning with modern AI technology to create an engaging and effective educational experience.</p>
              <p>Students can read comprehensive content, ask questions to the AI assistant, and practice with hands-on exercises to enhance their learning journey.</p>
              <p>This innovative approach bridges the gap between theoretical knowledge and practical application, making complex robotics and AI concepts more accessible and understandable.</p>
              
              {/* Demo Section Button */}
              <Link
                className="button button--lg"
                to="/docs/module-1-ros2/introduction-physical-ai"
                style={{
                  background: 'linear-gradient(135deg, #26d0ce 0%, #1a2980 100%)',
                  color: 'white',
                  border: 'none',
                  padding: '1rem 2.5rem',
                  fontSize: '1.1rem',
                  fontWeight: '600',
                  borderRadius: '10px',
                  boxShadow: '0 8px 25px rgba(38, 208, 206, 0.4)',
                  transition: 'all 0.3s ease',
                  display: 'inline-block',
                  textDecoration: 'none',
                  marginTop: '1.5rem'
                }}
                onMouseOver={(e) => {
                  e.currentTarget.style.transform = 'translateY(-3px)';
                  e.currentTarget.style.boxShadow = '0 12px 35px rgba(38, 208, 206, 0.6)';
                }}
                onMouseOut={(e) => {
                  e.currentTarget.style.transform = 'translateY(0)';
                  e.currentTarget.style.boxShadow = '0 8px 25px rgba(38, 208, 206, 0.4)';
                }}>
                Start Learning Now
              </Link>
            </div>
            <div className={styles.demoPreview}>
              <div style={{textAlign: 'center', color: '#a8b2d1'}}>
                <div style={{fontSize: '4rem', marginBottom: '1rem'}}>📖</div>
                <h3 style={{color: '#60d3d1', marginBottom: '1rem'}}>Textbook Preview</h3>
                <p>Interactive learning with AI-powered assistance</p>
              </div>
            </div>
          </div>
        </div>
      </section>

    </div>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive interactive textbook for ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems">
      <TextbookHomepage />
    </Layout>
  );
}