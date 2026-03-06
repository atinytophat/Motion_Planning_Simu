function degToRad(deg) {
  return (deg * Math.PI) / 180
}

function radToDeg(rad) {
  return (rad * 180) / Math.PI
}

export function rpyToRotation(rollDeg, pitchDeg, yawDeg) {
  const r = degToRad(rollDeg)
  const p = degToRad(pitchDeg)
  const y = degToRad(yawDeg)

  const cr = Math.cos(r)
  const sr = Math.sin(r)
  const cp = Math.cos(p)
  const sp = Math.sin(p)
  const cy = Math.cos(y)
  const sy = Math.sin(y)

  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr]
  ]
}

export function rotationToRpy(R) {
  const pitch = Math.asin(-R[2][0])
  const cp = Math.cos(pitch)

  let roll
  let yaw

  if (Math.abs(cp) < 1e-8) {
    roll = 0
    yaw = Math.atan2(-R[0][1], R[1][1])
  } else {
    roll = Math.atan2(R[2][1], R[2][2])
    yaw = Math.atan2(R[1][0], R[0][0])
  }

  return { roll: radToDeg(roll), pitch: radToDeg(pitch), yaw: radToDeg(yaw) }
}

function matMul3(A, B) {
  const out = Array.from({ length: 3 }, () => [0, 0, 0])
  for (let i = 0; i < 3; i += 1) {
    for (let j = 0; j < 3; j += 1) {
      out[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
    }
  }
  return out
}

function matTranspose3(A) {
  return [
    [A[0][0], A[1][0], A[2][0]],
    [A[0][1], A[1][1], A[2][1]],
    [A[0][2], A[1][2], A[2][2]]
  ]
}

function matAdd3(A, B) {
  return [
    [A[0][0] + B[0][0], A[0][1] + B[0][1], A[0][2] + B[0][2]],
    [A[1][0] + B[1][0], A[1][1] + B[1][1], A[1][2] + B[1][2]],
    [A[2][0] + B[2][0], A[2][1] + B[2][1], A[2][2] + B[2][2]]
  ]
}

function matScale3(A, s) {
  return [
    [A[0][0] * s, A[0][1] * s, A[0][2] * s],
    [A[1][0] * s, A[1][1] * s, A[1][2] * s],
    [A[2][0] * s, A[2][1] * s, A[2][2] * s]
  ]
}

function skew(v) {
  return [
    [0, -v[2], v[1]],
    [v[2], 0, -v[0]],
    [-v[1], v[0], 0]
  ]
}

function identity3() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
  ]
}

export function matrixLog3(R) {
  const tr = R[0][0] + R[1][1] + R[2][2]
  const cosTheta = Math.min(1, Math.max(-1, (tr - 1) / 2))
  const theta = Math.acos(cosTheta)

  if (Math.abs(theta) < 1e-8) {
    return { omega: [0, 0, 0], theta: 0 }
  }

  const omega = [
    (R[2][1] - R[1][2]) / (2 * Math.sin(theta)),
    (R[0][2] - R[2][0]) / (2 * Math.sin(theta)),
    (R[1][0] - R[0][1]) / (2 * Math.sin(theta))
  ]

  return { omega, theta }
}

export function matrixExp3(omega, theta) {
  const I = identity3()
  const wx = skew(omega)
  const wx2 = matMul3(wx, wx)
  const s = Math.sin(theta)
  const c = Math.cos(theta)
  return matAdd3(matAdd3(I, matScale3(wx, s)), matScale3(wx2, 1 - c))
}

export function interpolateRotationScrew(R1, R2, tau) {
  const t = Math.min(Math.max(tau, 0), 1)
  const Rrel = matMul3(matTranspose3(R1), R2)
  const { omega, theta } = matrixLog3(Rrel)

  if (Math.abs(theta) < 1e-8) {
    return R1
  }

  const Rstep = matrixExp3(omega, theta * t)
  return matMul3(R1, Rstep)
}

export function interpolatePositionLinear(p0, p1, tau) {
  const t = Math.min(Math.max(tau, 0), 1)
  return {
    x: p0.x + (p1.x - p0.x) * t,
    y: p0.y + (p1.y - p0.y) * t,
    z: p0.z + (p1.z - p0.z) * t
  }
}

export function poseFromWaypoint(waypoint) {
  return {
    position: {
      x: waypoint.x,
      y: waypoint.y,
      z: waypoint.z
    },
    orientation: {
      roll: waypoint.roll,
      pitch: waypoint.pitch,
      yaw: waypoint.yaw
    }
  }
}

