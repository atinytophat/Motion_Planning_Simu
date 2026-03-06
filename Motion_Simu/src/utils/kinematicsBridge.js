const API_BASE_URL = import.meta.env.VITE_KINEMATICS_API_URL || 'http://localhost:5000/api'

function toHomogeneousMatrix(orientation, position) {
  return [
    [orientation[0][0], orientation[0][1], orientation[0][2], position.x],
    [orientation[1][0], orientation[1][1], orientation[1][2], position.y],
    [orientation[2][0], orientation[2][1], orientation[2][2], position.z],
    [0, 0, 0, 1]
  ]
}

function fallbackFk(q) {
  const l1 = 400
  const l2 = 560
  const l3 = 515
  const r = ((q[1] + q[2]) * Math.PI) / 180
  const yaw = (q[0] * Math.PI) / 180

  const radial = l2 * Math.cos(r) + l3 * Math.cos(((q[1] + q[2] + q[3]) * Math.PI) / 180)
  const x = radial * Math.cos(yaw)
  const y = radial * Math.sin(yaw)
  const z = l1 + l2 * Math.sin(r) + l3 * Math.sin(((q[1] + q[2] + q[3]) * Math.PI) / 180)

  return {
    success: true,
    q,
    pose: {
      position: { x, y, z },
      orientation: { roll: q[3] || 0, pitch: q[4] || 0, yaw: q[5] || 0 },
      T: null
    },
    source: 'fallback'
  }
}

function fallbackIk(pose, seedQ = [0, 0, 0, 0, 0, 0]) {
  const x = pose.position.x
  const y = pose.position.y
  const z = pose.position.z
  const yaw = (Math.atan2(y, x) * 180) / Math.PI
  const q = [...seedQ]
  q[0] = yaw
  q[1] = ((z - 400) / 20) * 0.2
  q[2] = Math.hypot(x, y) / 20 - q[1]
  q[3] = pose.orientation.roll
  q[4] = pose.orientation.pitch
  q[5] = pose.orientation.yaw
  return { success: true, q, source: 'fallback' }
}

export async function computeFk(q) {
  try {
    const response = await fetch(`${API_BASE_URL}/fk`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ theta: q })  // Backend expects "theta", not "q"
    })

    if (!response.ok) {
      throw new Error(`FK request failed (${response.status})`)
    }

    const data = await response.json()

    // Normalize response: convert position list to {x, y, z} object and mm to m
    if (Array.isArray(data.position)) {
      data.pose = {
        position: { x: data.position[0] / 1000, y: data.position[1] / 1000, z: data.position[2] / 1000 },  // mm to m
        orientation: { roll: 0, pitch: 0, yaw: 0 },
        T: data.T  // T stays in mm (backend returns in mm)
      }
    } else if (!data.pose) {
      data.pose = {
        position: data.position ? {
          x: data.position.x / 1000,
          y: data.position.y / 1000,
          z: data.position.z / 1000
        } : { x: 0, y: 0, z: 0 },  // mm to m
        orientation: data.orientation || { roll: 0, pitch: 0, yaw: 0 },
        T: data.T  // T stays in mm (backend returns in mm)
      }
    }

    return { ...data, source: 'api' }
  } catch {
    return fallbackFk(q)
  }
}

export async function computeIk(pose, seedQ) {
  try {
    // Pass cartesian waypoints directly without transformation
    const response = await fetch(`${API_BASE_URL}/ik`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        T: pose.T || toHomogeneousMatrix(pose.orientationMatrix, pose.position),
        seedQ
      })
    })

    if (!response.ok) {
      throw new Error(`IK request failed (${response.status})`)
    }

    const data = await response.json()
    return { ...data, source: 'api' }
  } catch {
    return fallbackIk(pose, seedQ)
  }
}

export async function computeIkAll(pose, seedQ) {
  try {
    // Get all IK solutions and pick closest to seed (handles multi-solution IK)
    const response = await fetch(`${API_BASE_URL}/ik/all`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        T: pose.T || toHomogeneousMatrix(pose.orientationMatrix, pose.position),
        seedQ: seedQ || [0, 0, 0, 0, 0, 0]
      })
    })

    if (!response.ok) {
      throw new Error(`IK-all request failed (${response.status})`)
    }

    const data = await response.json()

    if (!data.success || !data.q) {
      throw new Error(data.error || 'Unknown error from IK-all')
    }

    return { ...data, source: 'api' }
  } catch (error) {
    // Fallback to regular IK
    return await computeIk(pose, seedQ)
  }
}

