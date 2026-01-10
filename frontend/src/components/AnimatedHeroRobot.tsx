import React from 'react';

const AnimatedHeroRobot: React.FC = () => {
  return (
    <div className="animated-hero-robot" style={{
      position: 'absolute',
      width: '120px',
      height: '160px',
      zIndex: 10,
      animation: 'robotFloat 3s ease-in-out infinite',
      left: '50%',
      top: '50%',
      transform: 'translate(-50%, -50%)'
    }}>
      {/* Robot Body */}
      <div className="robot-body" style={{
        width: '100%',
        height: '100%',
        position: 'relative',
      }}>
        {/* Robot Head */}
        <div className="robot-head" style={{
          width: '50px',
          height: '50px',
          background: 'linear-gradient(180deg, #60d3d1 0%, #1a2980 100%)',
          borderRadius: '12px',
          position: 'absolute',
          margin: '0 auto',
          boxShadow: '0 5px 20px rgba(96, 211, 209, 0.4)',
          border: '2px solid #60d3d1',
          left: '35px',
          top: '0',
          transition: 'transform 0.3s ease'
        }}>
          {/* Antenna with improved animation */}
          <div className="robot-antenna" style={{
            position: 'absolute',
            top: '-18px',
            left: '50%',
            transform: 'translateX(-50%)',
            width: '3px',
            height: '18px',
            background: 'linear-gradient(to top, #60d3d1, #26d0ce)',
            borderRadius: '2px',
          }}>
            <div style={{
              content: '',
              position: 'absolute',
              top: '-6px',
              left: '-5px',
              width: '13px',
              height: '13px',
              background: 'radial-gradient(circle, #ff6b6b, #ee5a52)',
              borderRadius: '50%',
              animation: 'antennaBlink 1.5s infinite',
              boxShadow: '0 0 8px rgba(255, 107, 107, 0.7)'
            }}></div>
          </div>

          {/* Eyes with enhanced animation */}
          <div className="robot-eyes" style={{
            display: 'flex',
            gap: '12px',
            justifyContent: 'center',
            marginTop: '14px'
          }}>
            <div className="robot-eye" style={{
              width: '10px',
              height: '10px',
              background: '#ffffff',
              borderRadius: '50%',
              position: 'relative',
              animation: 'eyeBlink 4s infinite',
              boxShadow: 'inset 0 0 5px rgba(0,0,0,0.3)'
            }}>
              <div style={{
                content: '',
                position: 'absolute',
                width: '5px',
                height: '5px',
                background: '#000',
                borderRadius: '50%',
                top: '2px',
                left: '2px',
                animation: 'eyeMove 3s infinite'
              }}></div>
            </div>
            <div className="robot-eye" style={{
              width: '10px',
              height: '10px',
              background: '#ffffff',
              borderRadius: '50%',
              position: 'relative',
              animation: 'eyeBlink 4.5s infinite',
              animationDelay: '0.5s',
              boxShadow: 'inset 0 0 5px rgba(0,0,0,0.3)'
            }}>
              <div style={{
                content: '',
                position: 'absolute',
                width: '5px',
                height: '5px',
                background: '#000',
                borderRadius: '50%',
                top: '2px',
                left: '2px',
                animation: 'eyeMove 3.5s infinite',
                animationDelay: '0.5s'
              }}></div>
            </div>
          </div>

          {/* Enhanced mouth/smile */}
          <div className="robot-mouth" style={{
            width: '25px',
            height: '3px',
            background: 'linear-gradient(to right, #60d3d1, #26d0ce)',
            borderRadius: '2px',
            margin: '10px auto 0',
            animation: 'smile 2.5s ease-in-out infinite',
            animationDelay: '0.2s'
          }}></div>
        </div>

        {/* Robot Torso with better design */}
        <div className="robot-torso" style={{
          width: '45px',
          height: '40px',
          background: 'linear-gradient(180deg, #1a2980 0%, #0d1b2a 100%)',
          borderRadius: '8px',
          position: 'absolute',
          border: '2px solid #60d3d1',
          boxShadow: '0 3px 15px rgba(96, 211, 209, 0.3)',
          left: '37px',
          top: '46px'
        }}>
          {/* Enhanced Control Panel */}
          <div className="robot-panel" style={{
            display: 'flex',
            gap: '4px',
            justifyContent: 'center',
            marginTop: '10px'
          }}>
            <div className="robot-led" style={{
              width: '6px',
              height: '6px',
              borderRadius: '50%',
              background: '#10b981',
              animation: 'ledPulse 1.2s infinite',
              animationDelay: '0s',
              boxShadow: '0 0 6px rgba(16, 185, 129, 0.7)'
            }}></div>
            <div className="robot-led" style={{
              width: '6px',
              height: '6px',
              borderRadius: '50%',
              background: '#f59e0b',
              animation: 'ledPulse 1.2s infinite',
              animationDelay: '0.4s',
              boxShadow: '0 0 6px rgba(245, 158, 11, 0.7)'
            }}></div>
            <div className="robot-led" style={{
              width: '6px',
              height: '6px',
              borderRadius: '50%',
              background: '#ef4444',
              animation: 'ledPulse 1.2s infinite',
              animationDelay: '0.8s',
              boxShadow: '0 0 6px rgba(239, 68, 68, 0.7)'
            }}></div>
          </div>
        </div>

        {/* Robot Arms with improved animation */}
        <div className="robot-arms" style={{
          position: 'absolute',
          width: '100%',
          top: '65px',
          left: '0'
        }}>
          <div className="robot-arm robot-arm-left" style={{
            position: 'absolute',
            width: '6px',
            height: '25px',
            background: 'linear-gradient(to bottom, #60d3d1, #1a2980)',
            borderRadius: '3px',
            left: '28px',
            transformOrigin: 'top center',
            animation: 'armWaveLeft 3.5s ease-in-out infinite',
            animationDelay: '0.3s',
            border: '1px solid #26d0ce'
          }}>
            <div className="robot-hand" style={{
              position: 'absolute',
              bottom: '-8px',
              left: '-3px',
              width: '10px',
              height: '10px',
              background: 'radial-gradient(circle, #60d3d1, #1a2980)',
              borderRadius: '50%',
              border: '1px solid #26d0ce'
            }}></div>
          </div>
          <div className="robot-arm robot-arm-right" style={{
            position: 'absolute',
            width: '6px',
            height: '25px',
            background: 'linear-gradient(to bottom, #60d3d1, #1a2980)',
            borderRadius: '3px',
            right: '28px',
            transformOrigin: 'top center',
            animation: 'armWaveRight 3.5s ease-in-out infinite',
            animationDelay: '0.3s',
            border: '1px solid #26d0ce'
          }}>
            <div className="robot-hand" style={{
              position: 'absolute',
              bottom: '-8px',
              left: '-3px',
              width: '10px',
              height: '10px',
              background: 'radial-gradient(circle, #60d3d1, #1a2980)',
              borderRadius: '50%',
              border: '1px solid #26d0ce'
            }}></div>
          </div>
        </div>

        {/* Robot Legs with improved animation */}
        <div style={{
          display: 'flex',
          gap: '10px',
          position: 'absolute',
          left: '33px',
          top: '86px'
        }}>
          <div style={{
            width: '8px',
            height: '25px',
            background: 'linear-gradient(to bottom, #1a2980, #0d1b2a)',
            borderRadius: '4px',
            border: '1px solid #60d3d1',
            animation: 'legMove 1.8s ease-in-out infinite',
            animationDelay: '0s'
          }}></div>
          <div style={{
            width: '8px',
            height: '25px',
            background: 'linear-gradient(to bottom, #1a2980, #0d1b2a)',
            borderRadius: '4px',
            border: '1px solid #60d3d1',
            animation: 'legMove 1.8s ease-in-out infinite',
            animationDelay: '0.9s'
          }}></div>
        </div>

        {/* Robot Feet */}
        <div style={{
          display: 'flex',
          gap: '10px',
          position: 'absolute',
          left: '33px',
          top: '113px'
        }}>
          <div style={{
            width: '12px',
            height: '5px',
            background: 'linear-gradient(to right, #60d3d1, #26d0ce)',
            borderRadius: '2px',
            animation: 'footTap 1.8s ease-in-out infinite',
            animationDelay: '0s'
          }}></div>
          <div style={{
            width: '12px',
            height: '5px',
            background: 'linear-gradient(to right, #60d3d1, #26d0ce)',
            borderRadius: '2px',
            animation: 'footTap 1.8s ease-in-out infinite',
            animationDelay: '0.9s'
          }}></div>
        </div>

        {/* Subtle glow effect around robot */}
        <div style={{
          position: 'absolute',
          top: '30px',
          left: '10px',
          width: '100px',
          height: '100px',
          borderRadius: '50%',
          background: 'radial-gradient(circle, rgba(96, 211, 209, 0.15) 0%, transparent 70%)',
          zIndex: -1,
          animation: 'pulse 3s ease-in-out infinite'
        }}></div>
      </div>
    </div>
  );
};

export default AnimatedHeroRobot;