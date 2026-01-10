import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

// Simple SVG Icons
const BookIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="80" height="80" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20" />
    <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z" />
  </svg>
);

const RobotIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="80" height="80" fill="none" stroke="currentColor" strokeWidth="2">
    <rect x="3" y="11" width="18" height="10" rx="2" />
    <circle cx="9" cy="6" r="2" />
    <circle cx="15" cy="6" r="2" />
    <path d="M7 11V6a5 5 0 0 1 10 0v5" />
  </svg>
);

const BrainIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="80" height="80" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M12 5a3 3 0 1 0-2.995 2.7A4 4 0 0 0 5 11.5a4 4 0 0 0 2.505 3.69A3 3 0 1 0 12 17a3 3 0 1 0 4.495-3.69A4 4 0 0 0 19 11.5a4 4 0 0 0-2.505-3.69A3 3 0 1 0 12 5" />
  </svg>
);

const FeatureList: FeatureItem[] = [
  {
    title: 'Learn ROS 2 Concepts',
    Svg: BookIcon,
    description: (
      <>
        Master ROS 2 fundamentals, architecture, and practical implementations
        for robotics applications.
      </>
    ),
  },
  {
    title: 'Robot Simulation',
    Svg: RobotIcon,
    description: (
      <>
        Explore Gazebo simulation environments and learn how to develop and
        test robotic systems virtually.
      </>
    ),
  },
  {
    title: 'AI Integration',
    Svg: BrainIcon,
    description: (
      <>
        Discover Vision-Language-Action systems and NVIDIA Isaac for
        intelligent robotics applications.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
