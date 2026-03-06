import React, { useState, useEffect } from 'react';
import './MotionControl.css';
import { api } from '../utils/apiConfig';

/**
 * MotionControl Component
 *
 * Main control panel for the motion planning web app.
 * Allows users to:
 * 1. Input start and end waypoints (theta angles or transformation matrices)
 * 2. Choose motion type (MoveJ or MoveL)
 * 3. Set motion duration
 * 4. Generate and visualize trajectories
 */

export function MotionControl({ robotVisualizer }) {
  // State for waypoint input
  const [startTheta, setStartTheta] = useState([0, 0, 0, 0, 0, 0]);
  const [endTheta, setEndTheta] = useState([0, 45, -90, 0, 0, 0]);

  const [inputMode, setInputMode] = useState('theta'); // 'theta' or 'matrix'
  const [motionType, setMotionType] = useState('MoveJ'); // 'MoveJ' or 'MoveL'
  const [motionTime, setMotionTime] = useState(5); // seconds
  const [numPoints, setNumPoints] = useState(100);

  // State for trajectory data and animation
  const [trajectory, setTrajectory] = useState(null);
  const [isAnimating, setIsAnimating] = useState(false);
  const [currentFrame, setCurrentFrame] = useState(0);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // Handle start theta input
  const handleStartThetaChange = (index, value) => {
    const newTheta = [...startTheta];
    newTheta[index] = parseFloat(value) || 0;
    setStartTheta(newTheta);
  };

  // Handle end theta input
  const handleEndThetaChange = (index, value) => {
    const newTheta = [...endTheta];
    newTheta[index] = parseFloat(value) || 0;
    setEndTheta(newTheta);
  };

  // Update robot visualization based on current frame
  useEffect(() => {
    if (!trajectory || !robotVisualizer) return;

    const currentTheta = trajectory.theta[currentFrame];
    robotVisualizer.updateRobotPose(currentTheta);

  }, [currentFrame, trajectory, robotVisualizer]);

  // Animation loop
  useEffect(() => {
    if (!isAnimating || !trajectory) return;

    const interval = setInterval(() => {
      setCurrentFrame(prev => {
        if (prev >= trajectory.theta.length - 1) {
          setIsAnimating(false);
          return prev;
        }
        return prev + 1;
      });
    }, 50); // ~20 FPS for smooth animation

    return () => clearInterval(interval);
  }, [isAnimating, trajectory]);

  // Generate trajectory
  const generateTrajectory = async () => {
    try {
      setLoading(true);
      setError(null);

      let response;

      if (motionType === 'MoveJ') {
        response = await api.moveJ(startTheta, endTheta, motionTime, numPoints);
      } else {
        // For MoveL, we need the end transformation matrix
        // First, get it from the end theta
        const fkResponse = await api.fk(endTheta);
        if (!fkResponse.success) throw new Error('Failed to compute FK for end pose');

        response = await api.moveL(startTheta, fkResponse.T, motionTime, numPoints);
      }

      if (!response.success) {
        throw new Error(response.error || 'Failed to generate trajectory');
      }

      setTrajectory(response.trajectory);
      setCurrentFrame(0);
      setIsAnimating(false);

    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  // Play animation
  const playAnimation = () => {
    if (!trajectory) return;
    setCurrentFrame(0);
    setIsAnimating(true);
  };

  // Pause animation
  const pauseAnimation = () => {
    setIsAnimating(false);
  };

  // Reset to start pose
  const resetPose = () => {
    setCurrentFrame(0);
    setIsAnimating(false);
  };

  return (
    <div className="motion-control">
      <h2>Motion Planning Control Panel</h2>

      {error && <div className="error-message">{error}</div>}

      {/* Input Mode Selection */}
      <div className="control-section">
        <h3>Input Mode</h3>
        <div className="radio-group">
          <label>
            <input
              type="radio"
              value="theta"
              checked={inputMode === 'theta'}
              onChange={(e) => setInputMode(e.target.value)}
            />
            Joint Angles (θ)
          </label>
          <label>
            <input
              type="radio"
              value="matrix"
              checked={inputMode === 'matrix'}
              onChange={(e) => setInputMode(e.target.value)}
            />
            Transformation Matrix
          </label>
        </div>
      </div>

      {/* Start Pose Input */}
      <div className="control-section">
        <h3>Start Pose</h3>
        <div className="theta-input">
          {startTheta.map((angle, idx) => (
            <div key={`start-${idx}`} className="input-group">
              <label>θ{idx + 1} (deg)</label>
              <input
                type="number"
                value={angle}
                onChange={(e) => handleStartThetaChange(idx, e.target.value)}
                step="1"
              />
            </div>
          ))}
        </div>
      </div>

      {/* End Pose Input */}
      <div className="control-section">
        <h3>End Pose</h3>
        <div className="theta-input">
          {endTheta.map((angle, idx) => (
            <div key={`end-${idx}`} className="input-group">
              <label>θ{idx + 1} (deg)</label>
              <input
                type="number"
                value={angle}
                onChange={(e) => handleEndThetaChange(idx, e.target.value)}
                step="1"
              />
            </div>
          ))}
        </div>
      </div>

      {/* Motion Configuration */}
      <div className="control-section">
        <h3>Motion Configuration</h3>

        <div className="config-group">
          <label>Motion Type:</label>
          <select value={motionType} onChange={(e) => setMotionType(e.target.value)}>
            <option value="MoveJ">Joint Interpolation (MoveJ)</option>
            <option value="MoveL">Linear Motion (MoveL)</option>
          </select>
        </div>

        <div className="config-group">
          <label>Motion Duration: {motionTime} s</label>
          <input
            type="range"
            min="1"
            max="30"
            value={motionTime}
            onChange={(e) => setMotionTime(parseFloat(e.target.value))}
          />
        </div>

        <div className="config-group">
          <label>Trajectory Points: {numPoints}</label>
          <input
            type="range"
            min="10"
            max="500"
            value={numPoints}
            onChange={(e) => setNumPoints(parseInt(e.target.value))}
          />
        </div>
      </div>

      {/* Generate Trajectory Button */}
      <div className="control-section">
        <button
          onClick={generateTrajectory}
          disabled={loading}
          className="btn-primary"
        >
          {loading ? 'Generating...' : 'Generate Trajectory'}
        </button>
      </div>

      {/* Animation Controls */}
      {trajectory && (
        <div className="control-section">
          <h3>Animation Controls</h3>

          <div className="animation-info">
            <p>Frame: {currentFrame} / {trajectory.theta.length - 1}</p>
            <p>Time: {trajectory.time[currentFrame].toFixed(2)}s</p>
          </div>

          <div className="button-group">
            <button onClick={playAnimation} disabled={isAnimating}>
              ▶ Play
            </button>
            <button onClick={pauseAnimation} disabled={!isAnimating}>
              ⏸ Pause
            </button>
            <button onClick={resetPose}>
              ⏮ Reset
            </button>
          </div>

          <div className="slider-group">
            <input
              type="range"
              min="0"
              max={trajectory.theta.length - 1}
              value={currentFrame}
              onChange={(e) => {
                setCurrentFrame(parseInt(e.target.value));
                setIsAnimating(false);
              }}
              className="timeline-slider"
            />
          </div>

          {/* Current Joint Angles Display */}
          <div className="current-pose">
            <h4>Current Joint Angles:</h4>
            <div className="pose-values">
              {trajectory.theta[currentFrame].map((angle, idx) => (
                <div key={idx} className="pose-value">
                  <span>θ{idx + 1}: {angle.toFixed(2)}°</span>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default MotionControl;
