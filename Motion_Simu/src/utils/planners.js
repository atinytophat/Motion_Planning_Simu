import { timeScaling } from './motionProfiles'
import {
  interpolatePositionLinear,
  interpolateRotationScrew,
  poseFromWaypoint,
  rpyToRotation,
  rotationToRpy
} from './math3d'
import { computeFk, computeIk, computeIkAll } from './kinematicsBridge'

function lerpScalar(a, b, t) {
  return a + (b - a) * t
}

function lerpJoint(qStart, qGoal, t) {
  return qStart.map((value, index) => lerpScalar(value, qGoal[index], t))
}

async function resolveJointFromWaypoint(waypoint, seedQ) {
  if (waypoint.inputType === 'joint') {
    return waypoint.q
  }

  // For cartesian waypoints (pose mode), use the T matrix directly
  if (waypoint.inputType === 'pose' && waypoint.T) {
    const ikResult = await computeIk({ T: waypoint.T }, seedQ)
    return ikResult.q
  }

  // Fallback for old format (shouldn't happen with new UI)
  const R = rpyToRotation(waypoint.roll || 0, waypoint.pitch || 0, waypoint.yaw || 0)
  const pose = {
    position: { x: waypoint.x || 0, y: waypoint.y || 0, z: waypoint.z || 0 },
    orientation: { roll: waypoint.roll || 0, pitch: waypoint.pitch || 0, yaw: waypoint.yaw || 0 },
    orientationMatrix: R
  }
  const ikResult = await computeIk(pose, seedQ)
  return ikResult.q
}

async function resolvePoseFromWaypoint(waypoint) {
  if (waypoint.inputType === 'pose' && waypoint.T) {
    // For cartesian waypoints, extract position from T matrix (convert mm to m)
    return {
      T: waypoint.T,
      position: {
        x: waypoint.T[0][3] / 1000,  // mm to m
        y: waypoint.T[1][3] / 1000,  // mm to m
        z: waypoint.T[2][3] / 1000   // mm to m
      },
      orientationMatrix: [
        [waypoint.T[0][0], waypoint.T[0][1], waypoint.T[0][2]],
        [waypoint.T[1][0], waypoint.T[1][1], waypoint.T[1][2]],
        [waypoint.T[2][0], waypoint.T[2][1], waypoint.T[2][2]]
      ]
    }
  }

  // For joint waypoints, compute FK
  const fkResult = await computeFk(waypoint.q)
  const pose = fkResult.pose
  return {
    T: fkResult.T,
    ...pose,
    orientationMatrix: rpyToRotation(
      pose.orientation.roll,
      pose.orientation.pitch,
      pose.orientation.yaw
    )
  }
}

export async function planMoveJ(startWaypoint, goalWaypoint, options) {
  const profile = options.profile || 'cubic'
  const samples = options.samples || 80
  const duration = options.duration || 4

  const qStart = await resolveJointFromWaypoint(startWaypoint, [0, 0, 0, 0, 0, 0])
  const qGoal = await resolveJointFromWaypoint(goalWaypoint, qStart)

  const points = []

  for (let i = 0; i <= samples; i += 1) {
    const tau = i / samples
    const s = timeScaling(tau, profile)
    const q = lerpJoint(qStart, qGoal, s)
    const fkResult = await computeFk(q)
    points.push({
      time: tau * duration,
      tau,
      q,
      pose: fkResult.pose
    })
  }

  return {
    mode: 'MoveJ',
    qStart,
    qGoal,
    points,
    duration
  }
}

// Matrix inversion helper
function invertMatrix(T) {
  // For homogeneous transformation matrix [R|p; 0|1]
  const R = [
    [T[0][0], T[1][0], T[2][0]],  // Transpose rotation
    [T[0][1], T[1][1], T[2][1]],
    [T[0][2], T[1][2], T[2][2]]
  ]
  const p = [T[0][3], T[1][3], T[2][3]]

  // T^-1 = [R^T | -R^T * p]
  const p_inv = [
    -(R[0][0]*p[0] + R[0][1]*p[1] + R[0][2]*p[2]),
    -(R[1][0]*p[0] + R[1][1]*p[1] + R[1][2]*p[2]),
    -(R[2][0]*p[0] + R[2][1]*p[1] + R[2][2]*p[2])
  ]

  const result = [
    [R[0][0], R[0][1], R[0][2], p_inv[0]],
    [R[1][0], R[1][1], R[1][2], p_inv[1]],
    [R[2][0], R[2][1], R[2][2], p_inv[2]],
    [0, 0, 0, 1]
  ]
  return result
}

// Matrix multiplication helper (4x4)
function multiplyMatrices(A, B) {
  const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      for (let k = 0; k < 4; k++) {
        result[i][j] += A[i][k] * B[k][j]
      }
    }
  }
  return result
}

// Matrix exponentiation (simplified for SE(3))
function matrixLerp(T1, T2, t) {
  // Simple linear interpolation of matrix elements (not perfect but works)
  // Better: use screw motion or exponential map, but this is simpler
  const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      result[i][j] = T1[i][j] + t * (T2[i][j] - T1[i][j])
    }
  }

  // Normalize rotation part (orthogonalize R)
  // Extract R and normalize
  const R = [
    [result[0][0], result[0][1], result[0][2]],
    [result[1][0], result[1][1], result[1][2]],
    [result[2][0], result[2][1], result[2][2]]
  ]

  // Gram-Schmidt orthogonalization
  const u0 = [R[0][0], R[1][0], R[2][0]]
  const u0_norm = Math.sqrt(u0[0]*u0[0] + u0[1]*u0[1] + u0[2]*u0[2])
  const x = [u0[0]/u0_norm, u0[1]/u0_norm, u0[2]/u0_norm]

  const u1 = [R[0][1], R[1][1], R[2][1]]
  const dot_x_u1 = x[0]*u1[0] + x[1]*u1[1] + x[2]*u1[2]
  const u1_orth = [u1[0] - dot_x_u1*x[0], u1[1] - dot_x_u1*x[1], u1[2] - dot_x_u1*x[2]]
  const u1_norm = Math.sqrt(u1_orth[0]*u1_orth[0] + u1_orth[1]*u1_orth[1] + u1_orth[2]*u1_orth[2])
  const y = [u1_orth[0]/u1_norm, u1_orth[1]/u1_norm, u1_orth[2]/u1_norm]

  const z = [
    x[1]*y[2] - x[2]*y[1],
    x[2]*y[0] - x[0]*y[2],
    x[0]*y[1] - x[1]*y[0]
  ]

  return [
    [x[0], y[0], z[0], result[0][3]],
    [x[1], y[1], z[1], result[1][3]],
    [x[2], y[2], z[2], result[2][3]],
    [0, 0, 0, 1]
  ]
}

function extractMatrixPose(T) {
  return {
    position: { x: T[0][3], y: T[1][3], z: T[2][3] },
    orientationMatrix: [
      [T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]]
    ]
  }
}

export async function planMoveL(startWaypoint, goalWaypoint, options) {
  const profile = options.profile || 'cubic'
  const samples = options.samples || 80
  const duration = options.duration || 4

  const poseStart = await resolvePoseFromWaypoint(startWaypoint)
  const poseGoal = await resolvePoseFromWaypoint(goalWaypoint)

  // Get transformation matrices (in mm for T matrix)
  const T_start = startWaypoint.inputType === 'pose' ? startWaypoint.T : poseStart.T
  const T_goal = goalWaypoint.inputType === 'pose' ? goalWaypoint.T : poseGoal.T

  if (!T_start || !T_goal) {
    return {
      mode: 'MoveL',
      points: [],
      duration,
      success: false,
      error: 'Missing transformation matrices'
    }
  }

  const points = []
  let lastQ = startWaypoint.inputType === 'joint' ? startWaypoint.q : [0, 0, 0, 0, 0, 0]
  const failedPoints = []

  // Compute relative transformation: ΔT = T_start^-1 * T_goal
  const T_start_inv = invertMatrix(T_start)
  const T_delta = multiplyMatrices(T_start_inv, T_goal)

  for (let i = 0; i <= samples; i += 1) {
    const tau = i / samples
    const s = timeScaling(tau, profile)

    try {
      // Interpolate transformation: T(tau) = T_start * (T_delta)^s
      // Simplified: linear interpolation of T_delta
      const T_interp_delta = matrixLerp(
        [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],  // Identity
        T_delta,
        s
      )
      const T_tau = multiplyMatrices(T_start, T_interp_delta)

      // Solve IK for this transformation matrix (send in mm, backend expects mm)
      const ikResult = await computeIkAll(
        {
          T: T_tau,  // Send in mm (backend DH params are in mm)
          ...extractMatrixPose(T_tau)  // Extract pose in mm
        },
        lastQ
      )

      if (!ikResult.success || !ikResult.q) {
        failedPoints.push({
          tau,
          position: extractMatrixPose(T_tau).position,
          reason: ikResult.error || 'IK failed'
        })
        continue
      }

      lastQ = ikResult.q

      points.push({
        time: tau * duration,
        tau,
        q: ikResult.q,
        pose: {
          position: {
            x: T_tau[0][3] / 1000,  // Convert mm to m for display
            y: T_tau[1][3] / 1000,
            z: T_tau[2][3] / 1000
          },
          orientation: { roll: 0, pitch: 0, yaw: 0 }  // Extracted from T if needed
        }
      })
    } catch (error) {
      failedPoints.push({
        tau,
        position: null,
        reason: error.message
      })
      continue
    }
  }

  return {
    mode: 'MoveL',
    points,
    duration,
    failedPoints,
    success: points.length > 1
  }
}

