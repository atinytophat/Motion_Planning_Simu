/**
 * Trajectory Generation Utilities
 *
 * This module provides various trajectory generation algorithms
 * for motion planning applications.
 */

/**
 * Linear interpolation between two waypoints
 * @param {Object} start - Start waypoint {x, y, z}
 * @param {Object} end - End waypoint {x, y, z}
 * @param {number} t - Interpolation parameter [0, 1]
 * @returns {Object} Interpolated point
 */
export function lerp(start, end, t) {
  return {
    x: start.x + (end.x - start.x) * t,
    y: start.y + (end.y - start.y) * t,
    z: start.z + (end.z - start.z) * t
  }
}

/**
 * Generate linear trajectory between waypoints
 * @param {Array} waypoints - Array of waypoint objects
 * @param {number} segments - Number of segments per waypoint pair
 * @returns {Array} Trajectory points
 */
export function generateLinearTrajectory(waypoints, segments = 50) {
  if (waypoints.length < 2) return []

  const trajectory = []

  for (let i = 0; i < waypoints.length - 1; i++) {
    const start = waypoints[i]
    const end = waypoints[i + 1]

    for (let j = 0; j <= segments; j++) {
      const t = j / segments
      const point = lerp(start, end, t)
      point.time = (i * (segments + 1) + j) / (segments + 1)
      trajectory.push(point)
    }
  }

  return trajectory
}

/**
 * Calculate distance between two points
 * @param {Object} p1 - Point 1 {x, y, z}
 * @param {Object} p2 - Point 2 {x, y, z}
 * @returns {number} Euclidean distance
 */
export function distance(p1, p2) {
  const dx = p2.x - p1.x
  const dy = p2.y - p1.y
  const dz = p2.z - p1.z
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

/**
 * Calculate total path length
 * @param {Array} trajectory - Trajectory points
 * @returns {number} Total path length
 */
export function calculatePathLength(trajectory) {
  if (trajectory.length < 2) return 0

  let totalLength = 0
  for (let i = 0; i < trajectory.length - 1; i++) {
    totalLength += distance(trajectory[i], trajectory[i + 1])
  }
  return totalLength
}

/**
 * Calculate velocity at each point (approximated)
 * @param {Array} trajectory - Trajectory points
 * @param {number} dt - Time step
 * @returns {Array} Velocity at each point
 */
export function calculateVelocities(trajectory, dt = 1.0) {
  const velocities = []

  for (let i = 0; i < trajectory.length; i++) {
    if (i === 0) {
      velocities.push({
        x: 0,
        y: 0,
        z: 0,
        magnitude: 0
      })
    } else {
      const prev = trajectory[i - 1]
      const curr = trajectory[i]
      const vx = (curr.x - prev.x) / dt
      const vy = (curr.y - prev.y) / dt
      const vz = (curr.z - prev.z) / dt
      const magnitude = Math.sqrt(vx * vx + vy * vy + vz * vz)

      velocities.push({
        x: vx,
        y: vy,
        z: vz,
        magnitude: magnitude
      })
    }
  }

  return velocities
}

/**
 * Get trajectory statistics
 * @param {Array} trajectory - Trajectory points
 * @returns {Object} Statistics including min, max, average values
 */
export function getTrajectoryStats(trajectory) {
  if (trajectory.length === 0) return null

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
  let minZ = Infinity, maxZ = -Infinity
  let sumX = 0, sumY = 0, sumZ = 0

  trajectory.forEach(p => {
    minX = Math.min(minX, p.x)
    maxX = Math.max(maxX, p.x)
    minY = Math.min(minY, p.y)
    maxY = Math.max(maxY, p.y)
    minZ = Math.min(minZ, p.z)
    maxZ = Math.max(maxZ, p.z)
    sumX += p.x
    sumY += p.y
    sumZ += p.z
  })

  const len = trajectory.length
  return {
    x: { min: minX, max: maxX, avg: sumX / len, range: maxX - minX },
    y: { min: minY, max: maxY, avg: sumY / len, range: maxY - minY },
    z: { min: minZ, max: maxZ, avg: sumZ / len, range: maxZ - minZ },
    totalPoints: len,
    pathLength: calculatePathLength(trajectory)
  }
}

/**
 * Cubic Hermite spline interpolation
 * @param {Object} p0 - Start point
 * @param {Object} p1 - End point
 * @param {Object} m0 - Start tangent/velocity
 * @param {Object} m1 - End tangent/velocity
 * @param {number} t - Parameter [0, 1]
 * @returns {Object} Interpolated point
 */
export function cubicHermite(p0, p1, m0, m1, t) {
  const t2 = t * t
  const t3 = t2 * t

  const h00 = 2 * t3 - 3 * t2 + 1
  const h10 = t3 - 2 * t2 + t
  const h01 = -2 * t3 + 3 * t2
  const h11 = t3 - t2

  return {
    x: h00 * p0.x + h10 * m0.x + h01 * p1.x + h11 * m1.x,
    y: h00 * p0.y + h10 * m0.y + h01 * p1.y + h11 * m1.y,
    z: h00 * p0.z + h10 * m0.z + h01 * p1.z + h11 * m1.z
  }
}

export default {
  lerp,
  generateLinearTrajectory,
  distance,
  calculatePathLength,
  calculateVelocities,
  getTrajectoryStats,
  cubicHermite
}
