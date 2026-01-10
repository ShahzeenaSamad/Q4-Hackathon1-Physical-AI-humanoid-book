import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

const MovableRobot: React.FC = () => {
  const [position, setPosition] = useState({ x: 100, y: 100 });
  const [isDragging, setIsDragging] = useState(false);
  const [isWaving, setIsWaving] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });

  // Auto-wave every 10 seconds
  useEffect(() => {
    const waveInterval = setInterval(() => {
      setIsWaving(true);
      setTimeout(() => setIsWaving(false), 2000);
    }, 10000);

    return () => clearInterval(waveInterval);
  }, []);

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    setDragOffset({
      x: e.clientX - position.x,
      y: e.clientY - position.y,
    });
  };

  const handleMouseMove = (e: MouseEvent) => {
    if (isDragging) {
      setPosition({
        x: e.clientX - dragOffset.x,
        y: e.clientY - dragOffset.y,
      });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  useEffect(() => {
    if (isDragging) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, dragOffset]);

  return (
    <div
      className={`${styles.robot} ${isDragging ? styles.dragging : ''}`}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
      onMouseDown={handleMouseDown}
      title="Drag me around! I'm your friendly robot assistant 🤖"
    >
      <div className={styles.robotBody}>
        {/* Robot Head */}
        <div className={styles.robotHead}>
          <div className={styles.antenna}></div>
          <div className={styles.eyes}>
            <div className={`${styles.eye} ${styles.eyeLeft}`}></div>
            <div className={`${styles.eye} ${styles.eyeRight}`}></div>
          </div>
          <div className={styles.mouth}></div>
        </div>

        {/* Robot Body */}
        <div className={styles.robotTorso}>
          <div className={styles.panel}></div>
          <div className={styles.ledIndicator}></div>
        </div>

        {/* Robot Arms */}
        <div className={`${styles.arm} ${styles.armLeft} ${isWaving ? styles.waving : ''}`}>
          <div className={styles.hand}></div>
        </div>
        <div className={`${styles.arm} ${styles.armRight}`}>
          <div className={styles.hand}></div>
        </div>

        {/* Robot Base/Wheels */}
        <div className={styles.robotBase}>
          <div className={`${styles.wheel} ${styles.wheelLeft}`}></div>
          <div className={`${styles.wheel} ${styles.wheelRight}`}></div>
        </div>
      </div>

      {/* Speech Bubble (appears on hover) */}
      <div className={styles.speechBubble}>
        <p>Hi! I'm RoboBot 🤖<br/>Drag me anywhere!</p>
      </div>
    </div>
  );
};

export default MovableRobot;
