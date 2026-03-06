import { useState, useEffect } from 'react'
import RobotVisualizer from './components/RobotVisualizer'
import WaypointInput from './components/WaypointInput'
import TrajectoryPlotter from './components/TrajectoryPlotter'
import { planMoveJ, planMoveL } from './utils/planners'
import './App.css'

function rotationMatrixToEuler(R) {
  // Convert rotation matrix to Euler angles (roll, pitch, yaw) in degrees
  // Using XYZ extrinsic convention (to match backend's convertCartesianToThetas)
  // This corresponds to: Rz(yaw) @ Ry(pitch) @ Rx(roll)
  let roll, pitch, yaw

  // Extract pitch (rotation about Y)
  pitch = Math.asin(-R[2][0]) * 180 / Math.PI

  // Check for gimbal lock
  if (Math.abs(Math.cos(pitch * Math.PI / 180)) < 1e-6) {
    // Gimbal lock case
    roll = 0
    yaw = Math.atan2(-R[0][1], R[1][1]) * 180 / Math.PI
  } else {
    // Normal case
    roll = Math.atan2(R[2][1], R[2][2]) * 180 / Math.PI
    yaw = Math.atan2(R[1][0], R[0][0]) * 180 / Math.PI
  }

  return { roll, pitch, yaw }
}

async function convertThetasToCartesian(thetas) {
  try {
    const response = await fetch('http://localhost:5000/api/fk', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ theta: thetas })
    })
    const result = await response.json()

    if (result.T) {
      const T = result.T
      // Backend returns T with position in mm (DH params are in mm)
      const position = {
        x: T[0][3],  // Already in mm
        y: T[1][3],
        z: T[2][3]
      }
      const R = [
        [T[0][0], T[0][1], T[0][2]],
        [T[1][0], T[1][1], T[1][2]],
        [T[2][0], T[2][1], T[2][2]]
      ]
      const orientation = rotationMatrixToEuler(R)
      const cartesian = { ...position, ...orientation }
      return cartesian
    }
  } catch (error) {
    // FK conversion error
  }
  return null
}

async function convertCartesianToThetas(cartesian) {
  try {
    // Create transformation matrix from cartesian pose
    const { x, y, z, roll, pitch, yaw } = cartesian

    // Convert Euler angles to rotation matrix (intrinsic ZYX)
    const r = roll * Math.PI / 180
    const p = pitch * Math.PI / 180
    const w = yaw * Math.PI / 180

    const Rx = [
      [1, 0, 0],
      [0, Math.cos(r), -Math.sin(r)],
      [0, Math.sin(r), Math.cos(r)]
    ]

    const Ry = [
      [Math.cos(p), 0, Math.sin(p)],
      [0, 1, 0],
      [-Math.sin(p), 0, Math.cos(p)]
    ]

    const Rz = [
      [Math.cos(w), -Math.sin(w), 0],
      [Math.sin(w), Math.cos(w), 0],
      [0, 0, 1]
    ]

    // Matrix multiplication helper
    const matmul = (A, B) => {
      const result = [[0,0,0], [0,0,0], [0,0,0]]
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          for (let k = 0; k < 3; k++) {
            result[i][j] += A[i][k] * B[k][j]
          }
        }
      }
      return result
    }

    // Intrinsic rotations: Rx @ Ry @ Rz
    const R = matmul(matmul(Rx, Ry), Rz)

    // Create 4x4 transformation matrix
    const T = [
      [R[0][0], R[0][1], R[0][2], x],
      [R[1][0], R[1][1], R[1][2], y],
      [R[2][0], R[2][1], R[2][2], z],
      [0, 0, 0, 1]
    ]

    // Call IK endpoint
    const response = await fetch('http://localhost:5000/api/ik', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ T })
    })
    const result = await response.json()

    if (result.success && result.q) {
      return result.q
    }
  } catch (error) {
    // IK conversion error
  }
  return null
}

function createWaypoint(inputType = 'joint') {
  if (inputType === 'pose') {
    return {
      inputType,
      T: [
        [1, 0, 0, 1190],
        [0, 1, 0, 0],
        [0, 0, 1, 425],
        [0, 0, 0, 1]
      ],
      q: [0, 0, 0, 0, 0, 0]  // Will be filled in by IK
    }
  }

  return {
    inputType,
    q: [0, 0, 0, 0, 0, 0]
  }
}

function updateWaypointField(waypoint, field, index, rawValue) {
  const value = Number(rawValue)

  if (field === 'q' && index !== null) {
    const q = [...waypoint.q]
    q[index] = Number.isNaN(value) ? 0 : value
    return { ...waypoint, q }
  }

  if (field === 'T' && waypoint.inputType === 'pose') {
    // Entire T matrix is being set
    return { ...waypoint, T: rawValue }
  }

  return {
    ...waypoint,
    [field]: Number.isNaN(value) ? 0 : value
  }
}

export default function App() {
  const [mode, setMode] = useState('motion') // 'motion' or 'pose'
  const [plannerMode, setPlannerMode] = useState('MoveJ')
  const [samples, setSamples] = useState(20)
  const [duration, setDuration] = useState(4)
  const [startWaypoint, setStartWaypoint] = useState(createWaypoint('joint'))
  const [goalWaypoint, setGoalWaypoint] = useState(createWaypoint('joint'))
  const [previewWaypoint, setPreviewWaypoint] = useState(null) // For real-time preview when editing
  const [planResult, setPlanResult] = useState(null)
  const [isPlanning, setIsPlanning] = useState(false)
  const [statusMessage, setStatusMessage] = useState('')
  const [leftTab, setLeftTab] = useState('control') // 'control' or 'plots'
  const [isAnimating, setIsAnimating] = useState(false)
  const [currentFrame, setCurrentFrame] = useState(0)
  const [animatedJointAngles, setAnimatedJointAngles] = useState(null)

  // Static pose control
  const [staticThetas, setStaticThetas] = useState([0, 0, 0, 0, 0, 0])
  const [tMatrix, setTMatrix] = useState([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ])
  const [ikSolutions, setIkSolutions] = useState(null)
  const [solvingMode, setSolvingMode] = useState('fk') // 'fk' or 'ik'
  const [alertMessage, setAlertMessage] = useState(null)

  const handleWaypointTypeChange = async (setter, nextType) => {
    if (nextType === 'pose') {
      // When switching to pose mode, convert current thetas to T matrix via FK
      setter((prev) => {
        // First update type to pose
        const updated = { ...prev, inputType: nextType }

        // Then convert thetas to cartesian and build T matrix
        if (prev.inputType === 'joint' && prev.q) {
          convertThetasToCartesian(prev.q).then((cart) => {
            if (cart) {
              // Reconstruct T matrix from Cartesian pose
              const { x, y, z, roll, pitch, yaw } = cart
              const r = roll * Math.PI / 180
              const p = pitch * Math.PI / 180
              const w = yaw * Math.PI / 180

              const Rx = [
                [1, 0, 0],
                [0, Math.cos(r), -Math.sin(r)],
                [0, Math.sin(r), Math.cos(r)]
              ]
              const Ry = [
                [Math.cos(p), 0, Math.sin(p)],
                [0, 1, 0],
                [-Math.sin(p), 0, Math.cos(p)]
              ]
              const Rz = [
                [Math.cos(w), -Math.sin(w), 0],
                [Math.sin(w), Math.cos(w), 0],
                [0, 0, 1]
              ]

              const matmul = (A, B) => {
                const result = [[0,0,0], [0,0,0], [0,0,0]]
                for (let i = 0; i < 3; i++) {
                  for (let j = 0; j < 3; j++) {
                    for (let k = 0; k < 3; k++) {
                      result[i][j] += A[i][k] * B[k][j]
                    }
                  }
                }
                return result
              }

              const R = matmul(matmul(Rx, Ry), Rz)
              const T = [
                [R[0][0], R[0][1], R[0][2], x],
                [R[1][0], R[1][1], R[1][2], y],
                [R[2][0], R[2][1], R[2][2], z],
                [0, 0, 0, 1]
              ]

              setter((current) => ({
                ...current,
                T
              }))
            }
          })
        }
        return updated
      })
    } else if (nextType === 'joint') {
      // When switching to joint mode, solve IK from current T matrix
      setter((prev) => {
        // First update type to joint
        const updated = { ...prev, inputType: nextType }

        // Then solve IK from T matrix
        if (prev.inputType === 'pose' && prev.T) {
          fetch('http://localhost:5000/api/ik', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ T: prev.T })
          })
            .then(res => res.json())
            .then(result => {
              if (result.success && result.q) {
                setter((current) => ({
                  ...current,
                  q: result.q.map(t => Math.round(t * 100) / 100)
                }))
              }
            })
            .catch(err => {
              // IK error handled silently
            })
        }
        return updated
      })
    } else {
      setter((prev) => ({ ...prev, inputType: nextType }))
    }
  }

  const handleWaypointChange = (setter, field, index, value) => {
    setter((prev) => {
      const updated = updateWaypointField(prev, field, index, value)

      // Auto-convert to mirror representation
      if (prev.inputType === 'joint' && field === 'q') {
        // When user changes joint angles, update T matrix
        convertThetasToCartesian(updated.q).then((cart) => {
          if (cart) {
            const { x, y, z, roll, pitch, yaw } = cart
            const r = roll * Math.PI / 180
            const p = pitch * Math.PI / 180
            const w = yaw * Math.PI / 180

            const Rx = [
              [1, 0, 0],
              [0, Math.cos(r), -Math.sin(r)],
              [0, Math.sin(r), Math.cos(r)]
            ]
            const Ry = [
              [Math.cos(p), 0, Math.sin(p)],
              [0, 1, 0],
              [-Math.sin(p), 0, Math.cos(p)]
            ]
            const Rz = [
              [Math.cos(w), -Math.sin(w), 0],
              [Math.sin(w), Math.cos(w), 0],
              [0, 0, 1]
            ]

            const matmul = (A, B) => {
              const result = [[0,0,0], [0,0,0], [0,0,0]]
              for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                  for (let k = 0; k < 3; k++) {
                    result[i][j] += A[i][k] * B[k][j]
                  }
                }
              }
              return result
            }

            const R = matmul(matmul(Rx, Ry), Rz)
            const T = [
              [R[0][0], R[0][1], R[0][2], x],
              [R[1][0], R[1][1], R[1][2], y],
              [R[2][0], R[2][1], R[2][2], z],
              [0, 0, 0, 1]
            ]

            setter((current) => ({ ...current, T }))
          }
        })
        // For joint waypoints, show preview immediately
        setPreviewWaypoint(updated)
      } else if (prev.inputType === 'pose' && field === 'T') {
        // When user changes T matrix, solve IK
        fetch('http://localhost:5000/api/ik', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ T: updated.T })
        })
          .then(res => res.json())
          .then(result => {
            if (result.success && result.q) {
              const updatedWithIK = {
                ...updated,
                q: result.q.map(t => Math.round(t * 100) / 100)
              }
              setter((current) => updatedWithIK)
              setPreviewWaypoint(updatedWithIK)
            }
          })
          .catch(err => {
            // IK error handled silently
          })
      }

      return updated
    })
  }

  const handleGenerateTrajectory = async () => {
    setIsPlanning(true)
    setStatusMessage('Planning trajectory...')
    setIsAnimating(false)
    setCurrentFrame(0)
    setAnimatedJointAngles(null)

    const options = {
      samples: Math.max(10, Number(samples) || 20),
      duration: Math.max(0.5, Number(duration) || 4)
    }

    try {
      const result =
        plannerMode === 'MoveJ'
          ? await planMoveJ(startWaypoint, goalWaypoint, options)
          : await planMoveL(startWaypoint, goalWaypoint, options)

      setPlanResult(result)
      setStatusMessage(
        `${result.mode} complete: ${result.points.length} samples generated.`
      )
    } catch (error) {
      setStatusMessage(`Planning error: ${error.message}`)
      setPlanResult(null)
    } finally {
      setIsPlanning(false)
    }
  }

  const handleSolve = async () => {
    if (solvingMode === 'fk') {
      // Call FK with current thetas
      try {
        const response = await fetch('http://localhost:8000/api/forward_kinematics', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ q: staticThetas })
        })
        const result = await response.json()
        if (result.error) {
          setAlertMessage(`❌ FK Computation Failed: ${result.error}`)
        } else if (result.T) {
          // Convert FK result from mm to meters for display
          const T_meters = result.T.map((row, i) => [
            ...row.slice(0, 3),
            i < 3 ? row[3] / 1000 : row[3]  // Convert position (column 3) from mm to m
          ])
          setTMatrix(T_meters)
          setIkSolutions(null)
          setStatusMessage('FK computed successfully')
          setAlertMessage('✓ FK computation successful')
        } else {
          setAlertMessage('❌ Invalid FK response from server')
        }
      } catch (error) {
        setAlertMessage(`❌ FK Error: ${error.message}`)
      }
    } else {
      // Call IK with current matrix
      try {
        const response = await fetch('http://localhost:8000/api/inverse_kinematics', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ T: tMatrix })
        })
        const result = await response.json()
        if (result.error) {
          setAlertMessage(`❌ IK Failed: ${result.error}\n\nThe transformation matrix may not be achievable by this robot's workspace.`)
        } else if (result.solutions && Array.isArray(result.solutions)) {
          if (result.solutions.length === 0) {
            setAlertMessage(`❌ No IK Solutions Found\n\nThe requested transformation matrix is outside the robot's workspace or kinematically unreachable.`)
            setIkSolutions([])
          } else {
            setIkSolutions(result.solutions)
            setStatusMessage(`IK found ${result.solutions.length} solution(s)`)
            setAlertMessage(`✓ Found ${result.solutions.length} IK solution(s)`)
          }
        } else {
          setAlertMessage('❌ Invalid IK response from server')
        }
      } catch (error) {
        setAlertMessage(`❌ IK Error: ${error.message}`)
      }
    }
  }

  // Handle FK solve button click
  const handleSolveFK = async () => {
    try {
      const response = await fetch('http://localhost:5000/api/fk', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ theta: staticThetas })
      })
      const result = await response.json()
      if (result.error) {
        setAlertMessage(`❌ FK Computation Failed: ${result.error}`)
      } else if (result.T) {
        // Convert FK result from mm to meters for display
        const T_meters = result.T.map((row, i) => [
          ...row.slice(0, 3),
          i < 3 ? row[3] / 1000 : row[3]  // Convert position (column 3) from mm to m
        ])
        setTMatrix(T_meters)
        setAlertMessage('✓ FK computation successful')
      } else if (!result.success) {
        setAlertMessage(`❌ FK Error: ${result.error || 'Unknown error'}`)
      } else {
        // Convert FK result from mm to meters for display
        const T_meters = result.T.map((row, i) => [
          ...row.slice(0, 3),
          i < 3 ? row[3] / 1000 : row[3]  // Convert position (column 3) from mm to m
        ])
        setTMatrix(T_meters)
        setAlertMessage('✓ FK computation successful')
      }
    } catch (error) {
      setAlertMessage(`❌ Connection Error: Make sure Flask backend is running on port 5000`)
    }
  }

  // Animation loop
  useEffect(() => {
    if (!isAnimating || !planResult?.points || planResult.points.length === 0) return

    const interval = setInterval(() => {
      setCurrentFrame((prev) => {
        const next = prev + 1
        if (next >= planResult.points.length) {
          setIsAnimating(false)
          return 0
        }
        setAnimatedJointAngles(planResult.points[next].q)
        return next
      })
    }, 1000 / 30) // 30 FPS

    return () => clearInterval(interval)
  }, [isAnimating, planResult])

  // Clear preview waypoint when animation starts
  useEffect(() => {
    if (isAnimating) {
      setPreviewWaypoint(null)
    }
  }, [isAnimating])

  // Clear animation values when switching to pose mode
  useEffect(() => {
    if (mode === 'pose') {
      setAnimatedJointAngles(null)
      setIsAnimating(false)
    }
  }, [mode])

  const handleStartAnimation = () => {
    if (!planResult?.points || planResult.points.length === 0) return
    setIsAnimating(true)
    setCurrentFrame(0)
    setAnimatedJointAngles(planResult.points[0].q)
  }

  return (
    <div className="app-container">
      {alertMessage && (
        <div style={{
          position: 'fixed',
          top: '50%',
          left: '50%',
          transform: 'translate(-50%, -50%)',
          background: '#1a1a2e',
          border: '2px solid rgba(102, 126, 234, 0.6)',
          borderRadius: '8px',
          padding: '24px',
          maxWidth: '400px',
          zIndex: 1000,
          boxShadow: '0 8px 32px rgba(0, 0, 0, 0.3)'
        }}>
          <p style={{
            color: '#e0e0e0',
            fontSize: '14px',
            margin: '0 0 16px 0',
            whiteSpace: 'pre-wrap',
            lineHeight: '1.5'
          }}>
            {alertMessage}
          </p>
          <button
            onClick={() => setAlertMessage(null)}
            style={{
              padding: '8px 16px',
              background: 'rgba(102, 126, 234, 0.7)',
              border: '1px solid rgba(102, 126, 234, 0.8)',
              borderRadius: '4px',
              color: '#e0e0e0',
              cursor: 'pointer',
              fontWeight: '500',
              width: '100%'
            }}
            onMouseEnter={(e) => e.target.style.background = 'rgba(102, 126, 234, 0.9)'}
            onMouseLeave={(e) => e.target.style.background = 'rgba(102, 126, 234, 0.7)'}
          >
            OK
          </button>
        </div>
      )}
      {alertMessage && (
        <div style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          background: 'rgba(0, 0, 0, 0.5)',
          zIndex: 999
        }}
        onClick={() => setAlertMessage(null)}
        />
      )}
      <div className="app-layout">
        {/* LEFT COLUMN */}
        <div className="left-panel">
          {/* MODE TABS */}
          <div className="mode-tabs">
            <button
              className={`mode-tab ${mode === 'motion' ? 'active' : ''}`}
              onClick={() => setMode('motion')}
            >
              Motion Planning
            </button>
            <button
              className={`mode-tab ${mode === 'pose' ? 'active' : ''}`}
              onClick={() => setMode('pose')}
            >
              Static Pose
            </button>
          </div>
          {mode === 'motion' ? (
            <>
              <div className="tab-buttons">
                <button
                  className={`tab-btn ${leftTab === 'control' ? 'active' : ''}`}
                  onClick={() => setLeftTab('control')}
                >
                  Controls
                </button>
                {planResult && planResult.points && planResult.points.length > 0 && (
                  <button
                    className={`tab-btn ${leftTab === 'plots' ? 'active' : ''}`}
                    onClick={() => setLeftTab('plots')}
                  >
                    Plots
                  </button>
                )}
              </div>

              {leftTab === 'control' ? (
                <section className="control-panel">
                  <h2>Motion Settings</h2>
                  <WaypointInput
                    plannerMode={plannerMode}
                    onPlannerModeChange={setPlannerMode}
                    samples={samples}
                    onSamplesChange={setSamples}
                    duration={duration}
                    onDurationChange={setDuration}
                    startWaypoint={startWaypoint}
                    goalWaypoint={goalWaypoint}
                    onStartWaypointTypeChange={(nextType) =>
                      handleWaypointTypeChange(setStartWaypoint, nextType)
                    }
                    onGoalWaypointTypeChange={(nextType) =>
                      handleWaypointTypeChange(setGoalWaypoint, nextType)
                    }
                    onStartWaypointChange={(field, index, value) =>
                      handleWaypointChange(setStartWaypoint, field, index, value)
                    }
                    onGoalWaypointChange={(field, index, value) =>
                      handleWaypointChange(setGoalWaypoint, field, index, value)
                    }
                    onGenerateTrajectory={handleGenerateTrajectory}
                    isPlanning={isPlanning}
                    statusMessage={statusMessage}
                  />

                  {planResult && planResult.points && planResult.points.length > 0 && (
                    <section className="animation-section">
                      <button
                        className="animate-btn"
                        onClick={handleStartAnimation}
                        disabled={isAnimating}
                      >
                        {isAnimating ? `Playing... (${currentFrame}/${planResult.points.length})` : 'Play Animation'}
                      </button>
                    </section>
                  )}
                </section>
              ) : (
                <section className="plots-panel">
                  <h2>Joint Trajectories</h2>
                  <TrajectoryPlotter points={planResult?.points || []} duration={planResult?.duration || 4} />
                </section>
              )}
            </>
          ) : (
            <section className="pose-control-panel">
              <h2>Joint Angles</h2>
              <div className="joint-inputs">
                {staticThetas.map((val, idx) => {
                  const limits = [
                    [-170, 170],   // Joint 1
                    [-190, 45],    // Joint 2
                    [-120, 156],   // Joint 3
                    [-185, 185],   // Joint 4
                    [-120, 120],   // Joint 5
                    [-350, 350]    // Joint 6
                  ]
                  const [min, max] = limits[idx]
                  return (
                    <div key={`theta-${idx}`} className="joint-slider-group">
                      <div className="joint-slider-header">
                        <label>θ{idx + 1}:</label>
                        <input
                          type="number"
                          min={-999}
                          max={999}
                          step="0.1"
                          value={val.toFixed(1)}
                          onChange={(e) => {
                            const newThetas = [...staticThetas]
                            newThetas[idx] = Number(e.target.value)
                            setStaticThetas(newThetas)
                          }}
                          className="joint-value-input"
                          title="Click to edit"
                        />
                      </div>
                      <input
                        type="range"
                        min={min}
                        max={max}
                        step="0.1"
                        value={val}
                        onChange={(e) => {
                          const newThetas = [...staticThetas]
                          newThetas[idx] = Number(e.target.value)
                          setStaticThetas(newThetas)
                        }}
                        className="joint-slider"
                      />
                      <div className="slider-limits">
                        <span className="min">{min}°</span>
                        <span className="max">{max}°</span>
                      </div>
                    </div>
                  )
                })}
              </div>

              <h2 style={{ marginTop: '1rem' }}>Transformation Matrix (meters)</h2>
              <button
                onClick={handleSolveFK}
                style={{
                  padding: '8px 16px',
                  background: 'rgba(102, 126, 234, 0.7)',
                  border: '1px solid rgba(102, 126, 234, 0.8)',
                  borderRadius: '6px',
                  color: '#e0e0e0',
                  fontWeight: '500',
                  cursor: 'pointer',
                  marginBottom: '0.8rem',
                  width: '100%'
                }}
              >
                Solve FK
              </button>

              {/* 4x4 Matrix Grid */}
              <div style={{
                display: 'grid',
                gridTemplateColumns: 'repeat(4, 1fr)',
                gap: '3px',
                marginBottom: '1rem',
                padding: '6px',
                background: 'rgba(20, 20, 35, 0.5)',
                borderRadius: '6px',
                width: 'fit-content'
              }}>
                {tMatrix.map((row, i) =>
                  row.map((val, j) => (
                    <div
                      key={`t-${i}-${j}`}
                      style={{
                        padding: '8px 8px',
                        background: '#1a1a2e',
                        border: '1px solid rgba(102, 126, 234, 0.4)',
                        color: '#e0e0e0',
                        borderRadius: '3px',
                        fontSize: '11px',
                        textAlign: 'center',
                        fontFamily: 'monospace',
                        minWidth: '100px'
                      }}
                    >
                      {typeof val === 'number' ? val.toFixed(5) : val}
                    </div>
                  ))
                )}
              </div>

              {/* Send to Waypoint Buttons */}
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.8rem', marginBottom: '1rem' }}>
                <button
                  onClick={() => {
                    // Convert T matrix from meters to mm for cartesian waypoints
                    const T_mm = [
                      [tMatrix[0][0], tMatrix[0][1], tMatrix[0][2], tMatrix[0][3] * 1000],
                      [tMatrix[1][0], tMatrix[1][1], tMatrix[1][2], tMatrix[1][3] * 1000],
                      [tMatrix[2][0], tMatrix[2][1], tMatrix[2][2], tMatrix[2][3] * 1000],
                      [0, 0, 0, 1]
                    ]
                    setStartWaypoint({ inputType: 'pose', T: T_mm, q: staticThetas })
                  }}
                  style={{
                    padding: '8px 12px',
                    background: 'rgba(76, 175, 80, 0.6)',
                    border: '1px solid rgba(76, 175, 80, 0.8)',
                    borderRadius: '6px',
                    color: '#e0e0e0',
                    fontWeight: '500',
                    cursor: 'pointer',
                    fontSize: '14px'
                  }}
                >
                  → Start Waypoint
                </button>
                <button
                  onClick={() => {
                    // Convert T matrix from meters to mm for cartesian waypoints
                    const T_mm = [
                      [tMatrix[0][0], tMatrix[0][1], tMatrix[0][2], tMatrix[0][3] * 1000],
                      [tMatrix[1][0], tMatrix[1][1], tMatrix[1][2], tMatrix[1][3] * 1000],
                      [tMatrix[2][0], tMatrix[2][1], tMatrix[2][2], tMatrix[2][3] * 1000],
                      [0, 0, 0, 1]
                    ]
                    setGoalWaypoint({ inputType: 'pose', T: T_mm, q: staticThetas })
                  }}
                  style={{
                    padding: '8px 12px',
                    background: 'rgba(244, 67, 54, 0.6)',
                    border: '1px solid rgba(244, 67, 54, 0.8)',
                    borderRadius: '6px',
                    color: '#e0e0e0',
                    fontWeight: '500',
                    cursor: 'pointer',
                    fontSize: '14px'
                  }}
                >
                  → Goal Waypoint
                </button>
              </div>

              {/* IK Solutions Table */}
              {ikSolutions && ikSolutions.length > 0 && (
                <div style={{ marginTop: '1rem' }}>
                  <h3>IK Solutions</h3>
                  <div style={{
                    overflowX: 'auto',
                    background: 'rgba(20, 20, 35, 0.5)',
                    borderRadius: '6px',
                    maxHeight: '300px',
                    overflowY: 'auto'
                  }}>
                    <table style={{
                      width: '100%',
                      borderCollapse: 'collapse',
                      fontSize: '11px'
                    }}>
                      <thead>
                        <tr style={{ borderBottom: '1px solid rgba(102, 126, 234, 0.3)' }}>
                          {['θ1', 'θ2', 'θ3', 'θ4', 'θ5', 'θ6'].map(label => (
                            <th key={label} style={{ padding: '8px', textAlign: 'center', color: '#66ccff' }}>
                              {label} (°)
                            </th>
                          ))}
                          <th style={{ padding: '8px', textAlign: 'center', color: '#66ccff' }}>Use</th>
                        </tr>
                      </thead>
                      <tbody>
                        {ikSolutions.map((solution, idx) => (
                          <tr
                            key={idx}
                            style={{
                              borderBottom: '1px solid rgba(102, 126, 234, 0.2)',
                              ':hover': { background: 'rgba(102, 126, 234, 0.1)' }
                            }}
                          >
                            {solution.q.map((angle, j) => (
                              <td
                                key={`sol-${idx}-${j}`}
                                style={{
                                  padding: '6px',
                                  textAlign: 'center',
                                  color: '#e0e0e0'
                                }}
                              >
                                {angle.toFixed(1)}
                              </td>
                            ))}
                            <td style={{ padding: '6px', textAlign: 'center' }}>
                              <button
                                onClick={() => setStaticThetas(solution.q)}
                                style={{
                                  padding: '4px 8px',
                                  background: 'rgba(68, 170, 255, 0.6)',
                                  border: '1px solid rgba(68, 170, 255, 0.8)',
                                  borderRadius: '3px',
                                  color: '#e0e0e0',
                                  cursor: 'pointer',
                                  fontSize: '10px'
                                }}
                              >
                                Use
                              </button>
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                </div>
              )}
            </section>
          )}
        </div>

        {/* RIGHT COLUMN */}
        <div className="right-panel">
          <section className="visualizer-section">
            <RobotVisualizer
              jointAngles={
                mode === 'pose'
                  ? staticThetas
                  : planResult && planResult.points && planResult.points.length > 0
                    ? planResult.points[0].q || [0, 0, 0, 0, 0, 0]
                    : [0, 0, 0, 0, 0, 0]
              }
              animatedJointAngles={animatedJointAngles}
              trajectoryPoints={planResult?.points || []}
              currentFrameIndex={currentFrame}
              startWaypoint={startWaypoint}
              goalWaypoint={goalWaypoint}
              isAnimating={isAnimating}
              mode={mode}
              previewWaypoint={previewWaypoint}
            />
          </section>
        </div>
      </div>
    </div>
  )
}
