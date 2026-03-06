import { useState, useRef, useEffect } from 'react';
import RobotVisualizer from './components/RobotVisualizer';
import TrajectoryPlotter from './components/TrajectoryPlotter';
import MotionControl from './components/MotionControl';
import { api } from './utils/apiConfig';
import './App.css';

/**
 * Main Application Component
 *
 * Integrates:
 * - MotionControl: User input for motion planning (start/end poses, motion type, time)
 * - RobotVisualizer: 3D visualization of robot + animation
 * - TrajectoryPlotter: Joint angle trajectory plots
 */

function App() {
  // Refs to child components
  const robotVisualizerRef = useRef(null);
  const trajectoryPlotterRef = useRef(null);

  // State
  const [trajectory, setTrajectory] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [apiStatus, setApiStatus] = useState('checking');

  // Check API health on mount
  useEffect(() => {
    const checkAPI = async () => {
      try {
        const response = await api.health();
        if (response.success) {
          setApiStatus('connected');
        } else {
          setApiStatus('error');
          setError('Backend API is not responding correctly');
        }
      } catch (err) {
        setApiStatus('error');
        setError(`Cannot connect to backend: ${err.message}`);
      }
    };

    checkAPI();
  }, []);

  // Expose trajectory update to child components through refs
  useEffect(() => {
    if (robotVisualizerRef.current && trajectory) {
      robotVisualizerRef.current.setTrajectory(trajectory);
    }
    if (trajectoryPlotterRef.current && trajectory) {
      trajectoryPlotterRef.current.setTrajectory(trajectory);
    }
  }, [trajectory]);

  return (
    <div className="app-container">
      {/* Header */}
      <header className="app-header">
        <p>Interactive Forward/Inverse Kinematics with Trajectory Visualization</p>

        {/* API Status Indicator */}
        <div className="api-status">
          <span className={`status-badge ${apiStatus}`}>
            {apiStatus === 'connected' && '✓ Backend Connected'}
            {apiStatus === 'checking' && '⏳ Checking Backend...'}
            {apiStatus === 'error' && '✗ Backend Offline'}
          </span>
        </div>
      </header>

      {/* Global Error Message */}
      {error && apiStatus === 'error' && (
        <div className="error-banner">
          <h3>❌ Connection Error</h3>
          <p>{error}</p>
          <p style={{ fontSize: '12px', marginTop: '8px' }}>
            Make sure the Flask backend is running: <code>python app.py</code>
          </p>
        </div>
      )}

      {/* Main Layout */}
      <div className="app-layout">
        {/* Control Panel (Sidebar) */}
        <aside className="sidebar">
          {apiStatus === 'connected' ? (
            <MotionControl
              robotVisualizer={robotVisualizerRef.current}
              onTrajectoryGenerated={setTrajectory}
            />
          ) : (
            <div className="offline-notice">
              <h3>⚠️ Backend Offline</h3>
              <p>Please start the Flask backend server:</p>
              <code>cd RoboDK_Motion_Planning_Simu<br/>python app.py</code>
              <p style={{ marginTop: '12px', fontSize: '12px' }}>
                The web app will auto-connect once the server is running.
              </p>
            </div>
          )}
        </aside>

        {/* Main Content */}
        <main className="main-content">
          {/* Robot Visualization */}
          <section className="visualizer-section">
            <h2>3D Robot Visualization</h2>
            <RobotVisualizer ref={robotVisualizerRef} />
          </section>

          {/* Trajectory Plots */}
          <section className="plotter-section">
            <h2>Joint Angle Trajectories</h2>
            {trajectory ? (
              <TrajectoryPlotter
                ref={trajectoryPlotterRef}
                trajectory={trajectory}
              />
            ) : (
              <div className="empty-state">
                <p>Generate a trajectory to see joint angle plots</p>
              </div>
            )}
          </section>
        </main>
      </div>

      {/* Footer */}
      <footer className="app-footer">
        <p>ME5463 Real-Time Motion Planning | KUKA KR-10 Robot</p>
      </footer>
    </div>
  );
}

export default App;
