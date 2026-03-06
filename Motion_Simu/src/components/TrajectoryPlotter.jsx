import { useState } from 'react'
import { Line } from 'react-chartjs-2'
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
} from 'chart.js'
import './TrajectoryPlotter.css'

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
)

const JOINT_COLORS = ['#ff6b6b', '#ffa94d', '#ffd43b', '#69db7c', '#4dabf7', '#b197fc']

export default function TrajectoryPlotter({ points, duration = 4 }) {
  const [plotType, setPlotType] = useState('position') // 'position', 'velocity', 'acceleration'

  if (!points || points.length === 0) {
    return (
      <div className="trajectory-plotter">
        <div className="empty-state">
          <p>Generate MoveJ or MoveL to see joint trajectories</p>
        </div>
      </div>
    )
  }

  // Calculate velocity and acceleration from position data
  const calculateVelocity = () => {
    return Array.from({ length: 6 }, (_, jointIndex) => {
      const velocities = []
      for (let i = 1; i < points.length; i++) {
        const dq = points[i].q[jointIndex] - points[i - 1].q[jointIndex]
        const dt = points[i].time - points[i - 1].time
        velocities.push(dt > 0 ? dq / dt : 0)
      }
      return [0, ...velocities] // Prepend 0 for first point
    })
  }

  const calculateAcceleration = () => {
    const velocities = calculateVelocity()
    return Array.from({ length: 6 }, (_, jointIndex) => {
      const accelerations = []
      for (let i = 1; i < points.length; i++) {
        const dv = velocities[jointIndex][i] - velocities[jointIndex][i - 1]
        const dt = points[i].time - points[i - 1].time
        accelerations.push(dt > 0 ? dv / dt : 0)
      }
      return [0, ...accelerations] // Prepend 0 for first point
    })
  }

  const labels = points.map((p) => p.time.toFixed(2))
  const maxTime = duration + 1  // Add 1 second to the trajectory duration for leeway

  let datasets, yAxisLabel, yAxisMin, yAxisMax

  if (plotType === 'position') {
    datasets = Array.from({ length: 6 }, (_, jointIndex) => ({
      label: `θ${jointIndex + 1}`,
      data: points.map((p) => ({ x: p.time, y: p.q[jointIndex] })),
      borderColor: JOINT_COLORS[jointIndex],
      backgroundColor: `${JOINT_COLORS[jointIndex]}22`,
      tension: 0.25,
      borderWidth: 2,
      pointRadius: 0,
      pointHoverRadius: 3
    }))
    yAxisLabel = 'Position (deg)'
    yAxisMin = -350
    yAxisMax = 350
  } else if (plotType === 'velocity') {
    const velocities = calculateVelocity()
    datasets = Array.from({ length: 6 }, (_, jointIndex) => ({
      label: `θ̇${jointIndex + 1}`,
      data: velocities[jointIndex].map((v, i) => ({ x: points[i].time, y: v })),
      borderColor: JOINT_COLORS[jointIndex],
      backgroundColor: `${JOINT_COLORS[jointIndex]}22`,
      tension: 0.25,
      borderWidth: 2,
      pointRadius: 0,
      pointHoverRadius: 3
    }))
    yAxisLabel = 'Velocity (deg/s)'
    yAxisMin = 'auto'
    yAxisMax = 'auto'
  } else {
    // acceleration
    const accelerations = calculateAcceleration()
    datasets = Array.from({ length: 6 }, (_, jointIndex) => ({
      label: `θ̈${jointIndex + 1}`,
      data: accelerations[jointIndex].map((a, i) => ({ x: points[i].time, y: a })),
      borderColor: JOINT_COLORS[jointIndex],
      backgroundColor: `${JOINT_COLORS[jointIndex]}22`,
      tension: 0.25,
      borderWidth: 2,
      pointRadius: 0,
      pointHoverRadius: 3
    }))
    yAxisLabel = 'Acceleration (deg/s²)'
    yAxisMin = 'auto'
    yAxisMax = 'auto'
  }

  const data = {
    datasets
  }

  const options = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top',
        labels: {
          color: '#888',
          font: {
            size: 10
          },
          padding: 5,
          boxHeight: 8
        }
      },
      title: {
        display: false
      }
    },
    scales: {
      x: {
        type: 'linear',
        title: {
          display: true,
          text: 'Time (s)',
          color: '#666',
          font: {
            size: 10
          }
        },
        grid: {
          color: 'rgba(255, 255, 255, 0.05)'
        },
        ticks: {
          color: '#666',
          font: {
            size: 9
          }
        },
        offset: false,
        grace: '0%',
        max: maxTime
      },
      y: {
        ...(yAxisMin !== 'auto' && yAxisMax !== 'auto' && { min: yAxisMin, max: yAxisMax }),
        title: {
          display: true,
          text: yAxisLabel,
          color: '#666',
          font: {
            size: 10
          }
        },
        grid: {
          color: 'rgba(255, 255, 255, 0.05)'
        },
        ticks: {
          color: '#666',
          font: {
            size: 9
          }
        }
      }
    }
  }

  return (
    <div className="trajectory-plotter" style={{ height: '100%', width: '100%', display: 'flex', flexDirection: 'column' }}>
      <div style={{
        display: 'flex',
        gap: '4px',
        marginBottom: '0.3rem',
        justifyContent: 'space-between',
        flex: '0 0 calc(100% / 9)',
        alignItems: 'center'
      }}>
        {['position', 'velocity', 'acceleration'].map((type) => (
          <button
            key={type}
            onClick={() => setPlotType(type)}
            style={{
              flex: 1,
              height: '100%',
              padding: '2px 4px',
              background: plotType === type ? 'rgba(102, 126, 234, 0.8)' : 'rgba(102, 126, 234, 0.4)',
              border: '1px solid rgba(102, 126, 234, 0.8)',
              borderRadius: '3px',
              color: '#e0e0e0',
              fontSize: '8px',
              lineHeight: '1',
              fontWeight: plotType === type ? '600' : '400',
              cursor: 'pointer',
              textTransform: 'capitalize',
              transition: 'all 0.2s'
            }}
          >
            {type}
          </button>
        ))}
      </div>
      <div style={{ flex: 1, minHeight: 0 }}>
        <Line data={data} options={options} />
      </div>
    </div>
  )
}
