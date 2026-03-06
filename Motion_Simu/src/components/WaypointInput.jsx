import './WaypointInput.css'
import React from 'react'

function JointInputs({ value, onChange }) {
  const limits = [
    [-170, 170],   // Joint 1
    [-190, 45],    // Joint 2
    [-120, 156],   // Joint 3
    [-185, 185],   // Joint 4
    [-120, 120],   // Joint 5
    [-350, 350]    // Joint 6
  ]

  return (
    <div className="input-grid">
      {value.q.map((jointValue, index) => {
        const [min, max] = limits[index]
        return (
          <div className="joint-slider-group" key={`q-${index + 1}`}>
            <div className="joint-slider-header">
              <label htmlFor={`q-${index + 1}`}>q{index + 1}:</label>
              <input
                type="number"
                min={-999}
                max={999}
                step="0.1"
                value={Number(jointValue).toFixed(1)}
                onChange={(e) => {
                  const numValue = Number(e.target.value)
                  onChange('q', index, numValue.toString())
                }}
                className="joint-value-input"
                title="Click to edit"
              />
            </div>
            <input
              id={`q-${index + 1}`}
              type="range"
              step="0.1"
              min={min}
              max={max}
              value={jointValue}
              onChange={(e) => {
                const numValue = Number(e.target.value)
                onChange('q', index, numValue.toString())
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
  )
}

function MatrixInputs({ value, onChange }) {
  const matrix = value.T || [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

  const handleCellChange = (row, col, newValue) => {
    const newMatrix = matrix.map(r => [...r])
    newMatrix[row][col] = parseFloat(newValue) || 0
    onChange('T', null, newMatrix)
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.8rem' }}>
      <label style={{ fontSize: '0.9rem', color: '#aaa' }}>
        Transformation Matrix (4x4) — Position in mm
      </label>

      {/* 4x4 Matrix Grid with editable inputs */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: 'repeat(4, 1fr)',
        gap: '3px',
        padding: '6px',
        background: 'rgba(20, 20, 35, 0.5)',
        borderRadius: '6px',
        width: 'fit-content'
      }}>
        {matrix.map((row, i) =>
          row.map((val, j) => (
            <input
              key={`t-${i}-${j}`}
              type="number"
              value={typeof val === 'number' ? val.toFixed(5) : val}
              onChange={(e) => handleCellChange(i, j, e.target.value)}
              step="1"
              style={{
                padding: '6px 4px',
                background: '#1a1a2e',
                border: '1px solid rgba(102, 126, 234, 0.4)',
                color: '#e0e0e0',
                borderRadius: '3px',
                fontSize: '10px',
                textAlign: 'center',
                fontFamily: 'monospace',
                minWidth: '60px',
                width: '100%'
              }}
            />
          ))
        )}
      </div>

      {/* Position indicator */}
      {matrix[0] && matrix[1] && matrix[2] && (
        <div style={{ fontSize: '0.8rem', color: '#88ff88', fontFamily: 'monospace' }}>
          ✓ Position (mm): [{matrix[0][3]?.toFixed(1)}, {matrix[1][3]?.toFixed(1)}, {matrix[2][3]?.toFixed(1)}]
        </div>
      )}
    </div>
  )
}

function WaypointCard({ title, value, onChange }) {
  return (
    <section className="waypoint-card">
      <h3>{title}</h3>

      {value.inputType === 'joint' ? (
        <JointInputs value={value} onChange={onChange} />
      ) : (
        <MatrixInputs value={value} onChange={onChange} />
      )}
    </section>
  )
}

export default function WaypointInput({
  plannerMode,
  onPlannerModeChange,
  samples,
  onSamplesChange,
  duration,
  onDurationChange,
  startWaypoint,
  goalWaypoint,
  onStartWaypointTypeChange,
  onGoalWaypointTypeChange,
  onStartWaypointChange,
  onGoalWaypointChange,
  onGenerateTrajectory,
  isPlanning,
  statusMessage
}) {
  return (
    <div className="waypoint-input">
      <section className="waypoint-card">
        <h3>Planning Mode</h3>
        <div className="mode-buttons">
          <button
            onClick={() => onPlannerModeChange('MoveJ')}
            className={`btn ${plannerMode === 'MoveJ' ? 'btn-primary' : 'btn-outline'}`}
          >
            MoveJ
          </button>
          <button
            onClick={() => onPlannerModeChange('MoveL')}
            className={`btn ${plannerMode === 'MoveL' ? 'btn-primary' : 'btn-outline'}`}
          >
            MoveL
          </button>
        </div>
      </section>

      <section className="waypoint-card">
        <h3>Trajectory Settings</h3>
        <div className="input-grid">
          <div className="input-group">
            <label htmlFor="samples-input">Samples</label>
            <input
              id="samples-input"
              type="number"
              min="10"
              max="400"
              value={samples}
              onChange={(e) => onSamplesChange(e.target.value)}
            />
          </div>
          <div className="input-group">
            <label htmlFor="duration-input">Duration (s)</label>
            <input
              id="duration-input"
              type="number"
              min="0.5"
              max="30"
              step="0.1"
              value={duration}
              onChange={(e) => onDurationChange(e.target.value)}
            />
          </div>
          <div className="input-group">
            <label htmlFor="input-type-select">Waypoint Type</label>
            <select
              id="input-type-select"
              value={startWaypoint.inputType}
              onChange={(e) => {
                onStartWaypointTypeChange(e.target.value)
                onGoalWaypointTypeChange(e.target.value)
              }}
            >
              <option value="joint">Joint Configuration (q1..q6)</option>
              <option value="pose">Cartesian Pose (Transformation Matrix)</option>
            </select>
          </div>
        </div>
      </section>

      <WaypointCard
        title="Start Waypoint"
        value={startWaypoint}
        onChange={onStartWaypointChange}
      />

      <WaypointCard
        title="Goal Waypoint"
        value={goalWaypoint}
        onChange={onGoalWaypointChange}
      />

      <button onClick={onGenerateTrajectory} className="btn btn-success" disabled={isPlanning}>
        {isPlanning ? 'Planning...' : `Generate ${plannerMode}`}
      </button>

      {statusMessage ? <p className="status-text">{statusMessage}</p> : null}
    </div>
  )
}
