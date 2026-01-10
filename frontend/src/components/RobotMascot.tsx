import React, { useState, useEffect } from 'react';

const RobotMascot: React.FC = () => {
  const [position, setPosition] = useState({ x: window.innerWidth - 150, y: window.innerHeight - 250 });
  const [isDragging, setIsDragging] = useState(false);
  const [offset, setOffset] = useState({ x: 0, y: 0 });

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    setOffset({
      x: e.clientX - position.x,
      y: e.clientY - position.y,
    });
  };

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (isDragging) {
        setPosition({
          x: e.clientX - offset.x,
          y: e.clientY - offset.y,
        });
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    if (isDragging) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, offset]);

  return (
    <div
      className="robot-container"
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
      onMouseDown={handleMouseDown}
    >
      <div className="robot-body">
        {/* Robot Head */}
        <div className="robot-head">
          <div className="robot-antenna"></div>
          <div className="robot-eyes">
            <div className="robot-eye"></div>
            <div className="robot-eye"></div>
          </div>
          <div className="robot-mouth"></div>
        </div>

        {/* Robot Torso */}
        <div className="robot-torso">
          <div className="robot-panel">
            <div className="robot-led"></div>
            <div className="robot-led"></div>
            <div className="robot-led"></div>
          </div>
        </div>

        {/* Robot Arms */}
        <div className="robot-arms">
          <div className="robot-arm robot-arm-left">
            <div className="robot-hand"></div>
          </div>
          <div className="robot-arm robot-arm-right">
            <div className="robot-hand"></div>
          </div>
        </div>

        {/* Robot Base */}
        <div className="robot-base">
          <div className="robot-wheel"></div>
          <div className="robot-wheel"></div>
        </div>
      </div>

      {/* Speech Bubble */}
      <div className="robot-speech">
        Hi! Drag me 🤖
      </div>
    </div>
  );
};

export default RobotMascot;
